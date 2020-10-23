function rGetStatusRawDataFull(drone)
% cStatus = Current Status
% cRawData   = Current Data from Nav_Data
%
% if length(cStatus) == 32 or length(cRawData) == 8
%  then the returned values are valid.
%
% A valid output is an 8*1 array in which the structure looks like:
% drone.pCom.cRawData =
% [batteryLevel unit: %                     1
%  Pitch        unit: rad      world ref    2 
%  Roll         unit: rad      world ref    3
%  Yaw          unit: rad      world ref    4
%  Altitude     unit: meter    surface ref  5
%  Vx           unit: m/s      body ref     6
%  Vy           unit: m/s      body ref     7
%  Vz           unit: m/s      body ref     8
%  Accx         unit: m/s^2    body ref     9
%  Accy         unit: m/s^2    body ref     10
%  Accz         unit: m/s^2    body ref     11
%  gyrox        unit: angle/s  world ref    12
%  gyroy        unit: angle/s  world ref    13
%  gyroz        unit: angle/s  world ref    14
%  motor1       unit: pwm(0-255)           15
%  motor2       unit: pwm(0-255)           16
%  motor3       unit: pwm(0-255)           17
%  motor4       unit: pwm(0-255)           18
% ];
%  --------------- input
%
% The input SequenceNumber is the Sequence for instance use;
%   the output SequenceNumber is the Sequence for instance use in the next
%   command.
% This command execution time is always bounded within 0.022 sec
% (after a successful connection) detail:
% the first connection can take 0.13 sec after that, the return time
% can less than 0.007 sec.

drone.pCom.cStatus = zeros(1,32);
drone.pCom.cRawData = zeros(1,14);

% it seems the minimum UDP timeout that allowed in matlab is about 1 sec
drone.pCom.stateChannel.Timeout = 0.001;

% According to the documentation SDK 1_7
% to sync nav data stream
% first: send a some packet to NavData port
try
    fprintf(drone.pCom.stateChannel, 1);
catch
end
% then Send the request for navdata_demo
try 
    % navdata_demo = TRUE >> some data at 15Hz /  FALSE >> all data at 200Hz
    fprintf(drone.pCom.controlChannel, ...
        sprintf('AT*CONFIG=%d,"general:navdata_demo","FALSE"\r',drone.pCom.SequenceNumber)); %TRUE/FALSE
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;
    
    
    % All data configuration
    fprintf(drone.pCom.controlChannel, ...
        sprintf( 'AT*CONFIG=%d,"general:navdata_options",%s\r', drone.pCom.SequenceNumber, '290718208'));
catch
end
drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;

try
    % Tentativa de configuração
    % Pede pacotes específicos
    % 117440526 
    % 34815
    
    fprintf(drone.pCom.controlChannel, ...
        sprintf('AT*CONFIG=%d,"general:navdata_options","2047"\r',drone.pCom.SequenceNumber)); 
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;
catch
end
drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;

% only ask for once
try
    fprintf(drone.pCom.controlChannel, ...
        sprintf('AT*COMWDG=%d\r',drone.pCom.SequenceNumber));
catch
end

drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
drone.pCom.nav_data = fread(drone.pCom.stateChannel,299,'uint8');
fclose(drone.pCom.stateChannel);

if(length(drone.pCom.nav_data) > 299)
    [drone.pCom.cStatus, drone.pCom.cRawData] =  Interpret_NavData(drone.pCom.nav_data);
    
    % Battery Test. 
    % Low frequency blink   (5Hz) -> battery < 50%
    % High frequency blink (10Hz)-> battery < 40%
    % Below 30% -> Mandatory Landing
    if drone.pCom.cRawData(1) < 30 && drone.pFlag.BatteryLevel == 2       
        drone.rLand;
    elseif drone.pCom.cRawData(1) < 40  && drone.pFlag.BatteryLevel == 1
        drone.pFlag.BatteryLevel = 2;
        disp('Battery level below 40%');
        drone.rSetLed(20,10,5);
    elseif drone.pCom.cRawData(1) < 50  && drone.pFlag.BatteryLevel == 0
        drone.pFlag.BatteryLevel = 1;
        disp('Battery level below 50%');
        drone.rSetLed(20,5,5);
    end
end

% =========================================================================
% The reason that add the following loop is:
% 1. solve the conflicts when running this
%  function on different matlab version, and also on different os (Mac os & windows 7)
%  2. running on matlab 2013b
% win 7 @ i5-2450 2.5GHZ
% this loop takes 0.5-1.5 ms (0.0005-0.0015 secondes) normally
% sometimes worst condition could be 10 ms (e.g. try to open the udp 3 times)
%
% running on matlab 2013a 64 bit, Mac OS 10.8.3, 1.8GHz Intel Core i5
% this loop usually takes 1-2 ms (0.001-0.002 secondes)

mark = 1;
while mark
    try
        fopen(drone.pCom.stateChannel);
        mark = 0;
    catch
        mark = 1;
    end
end

end


% =========================================================================
% =========================================================================
% =========================================================================
% [status, output] = Interpret_NavData(NDS)
% This function interprets the NavData
% the interpretation is performed for firmware 2.3.3 platform
% if you downgrade or upgrade the Drone firmware, you need to make sure if the structure of
% NavData changes.
% --------------------- input & output -------------------
% NDS (Nav Data Sample) is a response from Drone. It must be a 12*1,24*1 or 500*1 array.
% the function returns [status, output]
% if status == 0, then the status is invalid
% if output == 0, then the output is invalid
%
% a valid status is an array consisting of 32 elements in the index order
% [31,30,29,28,...1,0]
% meaning of each element can be found at the end of the comment.
%  e.g. : status(32) ---> indicates landing/flying state
%
% A valid output is an 8*1 array in which the structure looks like:
% [batteryLevel unit: %                     1
%  pitch        unit: angle    world ref    2 
%  roll         unit: angle    world ref    3
%  yaw          unit: angle    world ref    4
%  altitude     unit: meter    surface ref  5
%  Vx           unit: m/s      body ref     6
%  Vy           unit: m/s      body ref     7
%  Vz           unit: m/s      body ref     8
%  Accx         unit: m/s^2    body ref     9
%  Accy         unit: m/s^2    body ref     10
%  Accz         unit: m/s^2    body ref     11
%  gyrox        unit: angle/s  world ref    12
%  gyroy        unit: angle/s  world ref    13
%  gyroz        unit: angle/s  world ref    14
% ]; % Phys Filtered values after control processing
%----------------------------------------------------------
%       // Define masks for ARDrone state
%  0,   // FLY MASK                  : (0) Ardrone is landed, (1) Ardrone is flying
%  1    // VIDEO MASK                : (0) Video disable, (1) Video enable
%  2,   // VISION MASK               : (0) Vision disable, (1) Vision enable
%  3,   // CONTROL ALGO              : (0) Euler angles control, (1) Angular speed control
%  4,   // ALTITUDE CONTROL ALGO     : (0) Altitude control inactive (1) Altitude control active
%  5,   // USER feedback             :     Start button state
%  6,   // Control command ACK       : (0) None, (1) One received
%  7,   // CAMERA MASK               : (0) Camera not ready, (1) Camera ready
%  8,   // Travelling mask           : (0) Disable, (1) Enable
%  9,   // USB key                   : (0) Usb key not ready, (1) Usb key ready
% 10,   // Navdata demo              : (0) All navdata, (1) Only navdata demo
% 11,   // Navdata bootstrap         : (0) Options sent in all or demo mode, (1) No navdata options sent
% 12,   // Motors status             : (0) Ok, (1) Motors problem
% 13,   // Communication Lost        : (1) Com problem, (0) Com is ok
% 15,   // VBat low                  : (1) Too low, (0) Ok
% 16,   // User Emergency Landing    : (1) User EL is ON, (0) User EL is OFF
% 17,   // Timer elapsed             : (1) Elapsed, (0) Not elapsed
% 19,   // Angles                    : (0) Ok, (1) Out of range
% 21,   // Ultrasonic sensor         : (0) Ok, (1) Deaf
% 22,   // Cutout system detection   : (0) Not detected, (1) Detected
% 23,   // PIC Version number OK     : (0) A bad version number, (1) Version number is OK */
% 24,   // ATCodec thread ON         : (0) Thread OFF (1) thread ON
% 25,   // Navdata thread ON         : (0) Thread OFF (1) thread ON
% 26,   // Video thread ON           : (0) Thread OFF (1) thread ON
% 27,   // Acquisition thread ON     : (0) Thread OFF (1) thread ON
% 28,   // CTRL watchdog             : (1) Delay in control execution (> 5ms), (0) Control is well scheduled
% 29,   // ADC Watchdog              : (1) Delay in uart2 dsr (> 5ms), (0) Uart2 is good
% 30,   // Communication Watchdog    : (1) Com problem, (0) Com is ok
% 31    // Emergency landing         : (0) No emergency, (1) Emergency
%
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
%
function [status, output] = Interpret_NavData(NDS)

% programming...
% several fixed offset

DroneStateOffSet = 5;
SequenceNoOffSet = 9;
% ---- temporarily comment the following 2 lines ...
% HeadOffSet = 1;
% VisionFlagOffSet = 13;

% the first option block follows exactly the VisionFlag block
% ---- temporarily comment the following 2 lines ...
% Option_id_OS = 17; % OS is OffSet
% Option_size_OS = 19;

% from byte #20 - #24: the meaning is unknow.
% then: the data part follows
data_length = 4; % Size of the data
data_start  = 25; % Start of the Option(Message) Navdata_Demo
PhysMeasures_start = 235; % Start of the Acc Measures in the Option(Message) Phys_Measures
motorpwms_start = 431; % Start of the pwm motor in the Option PWM.
Vz_start = 527; % Start of the Vz variable in the Altitude Option.
% Each option is a message, the ArDrone is composed of 27 options or
% messages, that can be selected by the general:navdata_options command,
% with a mask of 32 bits, each one representing the desired message.
% Is important to advise that, if the vision_tag is selected in the 
% state mask, the drone will always send the option 16 mixed with the 
% last option you asked for, but in separate strings.
% 
% The values below are the beginning of the variable in the ar drone message. 
% Float values must be converted (mask =0), char and int dont need (mask=1).
% Battery_OS  = 25; unsigned int
% Pitch_OS    = 29; float
% Roll_OS     = 33; float
% Yaw_OS      = 37; float
% Altitude_OS = 41; int
% Vx_OS       = 45; float
% Vy_OS       = 49; float
% Vz_OS       = 527; float
% Accx        = 235; float
% Accy        = 239; float
% Accz        = 243; float
% gyrox       = 247; float
% gyroy       = 251; float
% gyroz       = 255; float
% Motor1      = 431; unsigned char
% Motor2      = 432; unsigned char
% Motor3      = 433; unsigned char
% Motor4      = 434; unsigned char
mark = [1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1];
output = zeros(size(mark)); 


if length(NDS) == 24
    % navData.Mode.BOOTSTRAP
    status = detectDroneState(NDS(DroneStateOffSet:SequenceNoOffSet-1));
    output = 0;
elseif length(NDS) >= 258 % Minimum value of the message
    % navData.MOde.DEMO
    % ------------------- start -----------------------
    
    output(1) = reArrange_and_decodeValue(NDS(25:28),1); %Battery %
    
    for ii = 2:7 % The first 8 values are in a sequence %length(mark)
            output(ii) = reArrange_and_decodeValue(...
                NDS(data_start+(ii-2)*data_length:data_start+(ii-1)*data_length-1),mark(ii))/1000;
    end
    
    % The V_z is part of other option of navdata
    output(8) = reArrange_and_decodeValue(NDS(Vz_start:Vz_start+4),mark(ii))/1000;
            
    % This loop catch the filtered values of accelerometer and gyroscope.
    for ii = 9:14 % The following 6 values are in a sequence but in a different message location
            output(ii) = reArrange_and_decodeValue(...
                NDS(PhysMeasures_start+(ii-9)*data_length:PhysMeasures_start+(ii-8)*data_length-1),mark(ii))/1000;
    end
    
    % This loop catch the values of the PWM motors, values from 0 to 255.
    for ii = 15:18 % The following 6 values are in a sequence but in a different message location
            output(ii) = NDS(motorpwms_start+(ii-15):motorpwms_start+(ii-14)-1);
    end
    
    status = detectDroneState(NDS(DroneStateOffSet:SequenceNoOffSet-1));
    % ------------------- end -----------------------
else
    % the length could be 12
    % neglect this data
    status = 0;
    output = 0;
end
end

function res = reArrange_and_decodeValue(input, mark)
% if mark = 1; then output a int
% else output the float value which is defined by SDK description.
% the input is an array (1*4)
% e.g. : NDS(Battery_OS:Battery_OS+data_length-1)
hex_value = dec2hex(input);
hex_value = [hex_value(4,:),hex_value(3,:),hex_value(2,:),hex_value(1,:)];
if mark == 1
    res = hex2dec(hex_value);
else
    res = typecast(uint32(hex2dec(hex_value)),'single');
end
end

function bin = detectDroneState(input)
hex = dec2hex(input);
% put the hex_value in the order [31, 30......0]
hex = [hex(4,:),hex(3,:),hex(2,:),hex(1,:)];
for i=1:length(hex)
    if hex(i)=='F'
        bin((i*4)-3:i*4)=[1 1 1 1];
    elseif hex(i)=='E'
        bin((i*4)-3:i*4)=[1 1 1 0];
    elseif hex(i)=='D'
        bin((i*4)-3:i*4)=[1 1 0 1];
    elseif hex(i)=='C'
        bin((i*4)-3:i*4)=[1 1 0 0];
    elseif hex(i)=='B'
        bin((i*4)-3:i*4)=[1 0 1 1];
    elseif hex(i)=='A'
        bin((i*4)-3:i*4)=[1 0 1 0];
    elseif hex(i)=='9'
        bin((i*4)-3:i*4)=[1 0 0 1];
    elseif hex(i)=='8'
        bin((i*4)-3:i*4)=[1 0 0 0];
    elseif hex(i)=='7'
        bin((i*4)-3:i*4)=[0 1 1 1];
    elseif hex(i)=='6'
        bin((i*4)-3:i*4)=[0 1 1 0];
    elseif hex(i)=='5'
        bin((i*4)-3:i*4)=[0 1 0 1];
    elseif hex(i)=='4'
        bin((i*4)-3:i*4)=[0 1 0 0];
    elseif hex(i)=='3'
        bin((i*4)-3:i*4)=[0 0 1 1];
    elseif hex(i)=='2'
        bin((i*4)-3:i*4)=[0 0 1 0];
    elseif hex(i)=='1'
        bin((i*4)-3:i*4)=[0 0 0 1];
    elseif hex(i)=='0'
        bin((i*4)-3:i*4)=[0 0 0 0];
    end
end
end


