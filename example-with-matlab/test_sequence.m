clear all;

function packet = buildMotorControlPacket(angle, scaler, throttle)
    %for some reason, these all need to be specified as uint32
    cleanAngle = uint32(bitand(angle, 0x01FF));
    cleanScaler = uint32(bitand(scaler, 0x03FF));
    cleanThrottle = uint32(bitand(throttle, 0x07FF));
    
    packet = cleanThrottle;
    packet = bitor(packet, bitshift(cleanScaler, 11));
    packet = bitor(packet, bitshift(cleanAngle, 21));
    disp(dec2bin(packet));
end

function measurementData = performUnitTestSegment(angle, scaler, throttle, length, instrumentSerialObject, instrumentReadingRate)
    %fwrite(instrumentSerialObject, buildMotorControlPacket(angle, scaler, throttle));
    write(instrumentSerialObject, buildMotorControlPacket(angle, scaler, throttle), 'uint32');
    flush(instrumentSerialObject);
    %25 is number of bytes per reading
    rawData = read(instrumentSerialObject, uint64(instrumentReadingRate * 25 * length), "string");
    measurementData = cell2mat(textscan(rawData, '%u16 %u16 %u16 %u16 %u16', 'Delimiter', ','));
end

function haltRotor(instrumentSerialObject)
    write(instrumentSerialObject, buildMotorControlPacket(0, 0, 800), 'uint32');
    pause(0.4);
    write(instrumentSerialObject, buildMotorControlPacket(0, 0, 600), 'uint32');
    pause(0.4);
    write(instrumentSerialObject, buildMotorControlPacket(0, 0, 400), 'uint32');
    pause(0.4);
    write(instrumentSerialObject, buildMotorControlPacket(0, 0, 200), 'uint32');
    pause(0.4);
    write(instrumentSerialObject, buildMotorControlPacket(0, 0, 100), 'uint32');
    pause(0.4);
    write(instrumentSerialObject, buildMotorControlPacket(0, 0, 60), 'uint32');
    pause(0.4);
    write(instrumentSerialObject, uint32(0), 'uint32');
    flush(instrumentSerialObject, 'output');
    return
end

function emergencyStop(instrumentSerialObject)
    write(instrumentSerialObject, uint32(0), 'uint32');
    flush(instrumentSerialObject, 'output');
return
end

function responseData = runAndProcessUnitTestSegment(angle, scaler, throttle, length, instrumentSerialObject, instrumentReadingRate, csvFileName, startTimeStripAmount, LPFFrequency)
    %run the test segment
    data = performUnitTestSegment(angle, scaler, throttle, length, instrumentSerialObject, instrumentReadingRate);

    %save the data (append if already exists) including angle, scaler and throttle
    %for archival purposes
    if (csvFileName ~= "")
        dataSize = size(data);
        numDataPoints = dataSize(1);
        writematrix([(zeros(numDataPoints, 1)+angle), (zeros(numDataPoints, 1)+scaler), (zeros(numDataPoints, 1)+throttle), data],csvFileName,'WriteMode','append');
    end
    
    %strip away ramp time
    data = data(uint64(instrumentReadingRate * startTimeStripAmount):end,:);
    
    %convert data to doubles
    data = double(data);
    
    %low pass filter everything (LPFFrequency)
    %not compatible with GNU Octave
    data = lowpass(data, LPFFrequency, instrumentReadingRate);
    
    %calculate thrust and torque arrays (as a vector) (try both mine and rajatha's methods)
    %rajatha method:
    r=1;
    f = data(:,1) + data(:,2) + data(:,3);
    Mx = (r/2) * (((-2)*data(:,1)) + data(:,2) + data(:,3));
    My = ((-(sqrt(3)/2))*r) * (data(:,2) - data(:,3));
    %my method:

    P = data(:,4) .* data(:,5);
    
    responseData = [Mx, My, f, P];
end

%---------------------------------------------------------------------------

% Runs n iterations of the test sequence, compensates for external torques
% or load cell offsets and averages all the values to one output
function rotorResponse = evaluateRotor(name, scaler, throttle, length, instrumentSerialObject, instrumentReadingRate, startTimeStripAmount, LPFFrequency)

    testSequence = [0, scaler, throttle
                    90, scaler, throttle
                    180, scaler, throttle
                    270, scaler, throttle];
    
    disp("-----------")
    
    % archive file name
    csvFile = "";
    if (name ~= "")
        csvFile = "data_archive/" + name + "_data.csv";
    end

    runResults = cell2mat(cellfun(@(testParameters) mean(runAndProcessUnitTestSegment(testParameters(1),testParameters(2),testParameters(3), length, instrumentSerialObject, instrumentReadingRate, csvFile, startTimeStripAmount, LPFFrequency)), num2cell(testSequence, 2), 'UniformOutput', false));
    %do we really need multiple iterations? All the results are pretty close to
    %each other...
    %for c = 1:2
    %    disp("-----------");
    %    rawTorqueResults = cell2mat(cellfun(@(testParameters) mean(runAndProcessUnitTestSegment(testParameters(1),testParameters(2),testParameters(3), length, instrumentSerialObject, instrumentReadingRate, "meep", startTimeStripAmount, LPFFrequency)), num2cell(testSequence, 2), 'UniformOutput', false));
    %end
    
    %use something better than average (mean) to reject outliers more?
    %mean seems better than median tho

    %dynamically compensate for external torques / load cell offsets
    torqueVectors = runResults(:,1:2);
    offsetPosition = mean(torqueVectors);
    correctedtorqueVectors = torqueVectors - offsetPosition;
    correctedtorqueVectorMagnitudes = vecnorm(transpose(correctedtorqueVectors));
    
    rotorResponse = [mean(correctedtorqueVectorMagnitudes); mean(runResults(:,4))];

end

%-------------------------------------------------------

%----- Test Sequence Parameters -----
unitTestlength = 3; % in seconds (ideally above 7 ish)
runScaler = 200;
runThrottle = 1000;

%----- Instrumentation parameters -----
readingRate = 1500; % in Hz (Sps)
rampTime = 0.6; % in seconds (ideally 2)
lowPassFilterFrequency = 200; % in Hz
port = "/dev/ttyACM0";
%although baudrate doesnt matter, specify anyways
%wait... it has to be 9600 to work? straaaange....
measurementInstrumentation = serialport(port, 9600);

%for archival purposes
hingeName = input("Please specify hinge file name: ", "s");

% Run Tests ----------------------

%before testing, get the motor spinning to ensure everything is mostly
%stable
disp("Starting motor...");
write(measurementInstrumentation, buildMotorControlPacket(0, 0, runThrottle), 'uint32');
pause(2);

disp("Begining test sequences");

%data = performUnitTestSegment(0,0,1000,unitTestlength, measurementInstrumentation, readingRate);
%meep = runAndProcessUnitTestSegment(270,200,800, unitTestlength, measurementInstrumentation, readingRate, "meep", rampTime, lowPassFilterFrequency);

% !!! WARNING - DO NOT GO ABOVE 400 FOR PROLOGNED PERIODS OF TIME - RISK OF SEVERE MOTOR OVERHEATING !!!
scalerValuesToTest = 0:10:500;
%scalerValuesToTest = [200, 200, 200, 250, 250, 250, 300, 300, 300];
scalerValuesToTest = flip(scalerValuesToTest);  % Remember, test high scaler values first for motor overheating reasons
rotorResponseValues = cell2mat(cellfun(@(scalerValue) evaluateRotor(hingeName, scalerValue, runThrottle, unitTestlength, measurementInstrumentation, readingRate, rampTime, lowPassFilterFrequency), num2cell(scalerValuesToTest), 'UniformOutput', false));

disp("Finished Test sequences, Slowing and halting motor...");
haltRotor(measurementInstrumentation);
disp("Motor Halted!");
disp("Done! :D");

plot(rotorResponseValues(2,:), rotorResponseValues(1,:), '.r', 'MarkerSize',20);

