clear all;

function packet = buildMotorControlPacket(angle, scaler, throttle)
  #for some reason, these all need to be specified as uint32
  cleanAngle = uint32(bitand(angle, 0x01FF));
  cleanScaler = uint32(bitand(scaler, 0x03FF));
  cleanThrottle = uint32(bitand(throttle, 0x07FF));

  packet = cleanThrottle;
  packet = bitor(packet, bitshift(cleanScaler, 11));
  packet = bitor(packet, bitshift(cleanAngle, 21));
  dec2bin(packet)
end

function measurementData = performUnitTestSegment(angle, scaler, throttle, length, instrumentSerialObject, instrumentReadingRate)
  fwrite(instrumentSerialObject, buildMotorControlPacket(angle, scaler, throttle));
  flush(instrumentSerialObject);
  #25 is number of bytes per reading
  rawData = read(instrumentSerialObject, uint64(instrumentReadingRate * 25 * length), "string");
  measurementData = cell2mat(textscan(rawData, '%u16 %u16 %u16 %u16 %u16', 'Delimiter', ','));
end

function haltRotor(instrumentSerialObject)
  fwrite(instrumentSerialObject, uint32(0));
  flush(instrumentSerialObject, 'output');
  return
end

function responseData = runAndProcessUnitTestSegment(angle, scaler, throttle, length, instrumentSerialObject, instrumentReadingRate, csvFileName, startTimeStripAmount, LPFFrequency)
  #run the test segment
  data = performUnitTestSegment(angle, scaler, throttle, length, instrumentSerialObject, instrumentReadingRate);
  #save the data (append if already exists) including angle, scaler and throttle

  #strip away ramp time
  data = data(uint64(instrumentReadingRate * startTimeStripAmount):end,:);

  #low pass filter everything (LPFFrequency)
  #data = lowpass;

  #calculate thrust and torque arrays (as a vector) (try both mine and rajatha's methods)
  #rajatha method:
  r=1;
  Mx = (r/2) * (((-2)*double(data(:,1))) + double(data(:,2)) + double((data(:,3))));
  My = ((-(sqrt(3)/2))*r) * (double(data(:,2)) - double(data(:,3)));
  responseData = [Mx, My];
end

#---------------------------------------------------------------------------

#----- Bayesian optimisation parameters -----
numStartPoints = 4
#optimvar
#generate the latin hypercube arrays

#----- Test Sequence Parameters -----
unitTestlength = 5; # in seconds
runScaler = 300;
testSequence = [0, 0, 0
                0, 0, 0
                0, runScaler, 1000
                90, runScaler, 1000
                180, runScaler, 1000
                270, runScaler, 1000];

#----- Instrumentation parameters -----
readingRate = 1500; # in Hz (Sps)
rampTime = 0.6; # in seconds
lowPassFilterFrequency = 200 # in Hz
port = "/dev/ttyACM0";
#although baudrate doesnt matter, specify anyways
#wait... it has to be 9600 to work? straaaange....
measurementInstrumentation = serialport(port, 9600);


#data = performUnitTestSegment(0,800,1200,unitTestlength, measurementInstrumentation, readingRate);
#csvwrite(data, "data.csv");
#meep = runAndProcessUnitTestSegment(0,0,0, unitTestlength, measurementInstrumentation, readingRate, "meep", rampTime, lowPassFilterFrequency);

runResult = mean(cellfun(@(testParameters) norm(mean(runAndProcessUnitTestSegment(testParameters(1),testParameters(2),testParameters(3), unitTestlength, measurementInstrumentation, readingRate, "meep", rampTime, lowPassFilterFrequency))), num2cell(testSequence, 2))) / runScaler

haltRotor(measurementInstrumentation);
