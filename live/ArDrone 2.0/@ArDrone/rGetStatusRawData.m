function rGetStatusRawData(drone)
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

drone.pCom.cStatus  = zeros(1,32);
drone.pCom.cRawData = zeros(14,1);

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

fclose(drone.pCom.stateChannel);
% it seems the minimum UDP timeout that allowed in matlab is about 1 milisec
drone.pCom.stateChannel.Timeout = 0.001;
% tic
mark = 1;
while mark
    try
        fopen(drone.pCom.stateChannel);
        mark = 0;
    catch
        mark = 1;
    end
end
% toc

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
        sprintf('AT*CONFIG=%d,"general:navdata_demo","TRUE"\r',drone.pCom.SequenceNumber));
%  %   sprintf('AT*CONFIG=%d,"general:navdata_demo","FALSE"\r',drone.pCom.SequenceNumber)); % Comentado 23-09 T 
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;
    
    % All data configuration
    fprintf(drone.pCom.controlChannel, ...
        sprintf( 'AT*CONFIG=%d,"general:navdata_options",%s\r', drone.pCom.SequenceNumber, '290718208'));
catch
    disp('Falha de configuração do NavData.')
end
drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;

% Tentativa de configuração desabilitar controle de altitude
try
    fprintf(drone.pCom.controlChannel, ...
        sprintf( 'AT*CONFIG=%d,"general:navdata_options","FALSE"\r', drone.pCom.SequenceNumber));
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
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;
    fprintf(drone.pCom.controlChannel, ...
        sprintf( 'AT*CONFIG=%d,"general:com_watchdog","TRUE"\r', drone.pCom.SequenceNumber));
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;
    fprintf(drone.pCom.controlChannel, ...
        sprintf( 'AT*CONFIG=%d,"general:video_enable","TRUE"\r', drone.pCom.SequenceNumber));
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;
    fprintf(drone.pCom.controlChannel, ...
            sprintf( 'AT*CONFIG=%d,"general:vision_enable","TRUE"\r', drone.pCom.SequenceNumber));
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber+1;
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
    drone.pPar.Battery = drone.pCom.cRawData(1);
    if drone.pCom.cRawData(1) < 30 && drone.pFlag.BatteryLevel == 2
        %         drone.rLand;
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

end

function [status, output] = Interpret_NavData(NDS)

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
% a valid status is an arry consisting of 32 elements in the index order
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
%function [status, output] = Interpret_NavData(NDS)
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
% a valid status is an arry consisting of 32 elements in the index order
% [31,30,29,28,...1,0]
% meaning of each element can be found at the end of the comment.
%  e.g. : status(32) ---> indicates landing/flying state
%
% a valid output is an 8*1 array in which the structure looks like:
% [batteryLevel unit: %        1
%  pitch        unit: angle    2
%  roll         unit: angle    3
%  yaw          unit: angle    4
%  altitude     unit: meter    5
%  Vx           unit: m/s      6
%  Vy           unit: m/s      7
%  Vz];         unit: m/s      8
%

%%
% programming...
% several fixed offset

DroneStateOffSet = 5;
SequenceNoOffSet = 9;

% from byte #20 - #24: the meaning is unknow.
% then: the data part follows
data_length = 4;
Battery_OS  = 25; % data element has a length of 4 bytes
Pitch_OS    = 29;
Roll_OS     = 33;
Yaw_OS      = 37;
Altitude_OS = 41;
Vx_OS       = 45;
Vy_OS       = 49;
Vz_OS       = 53;


if length(NDS) == 24
    % navData.Mode.BOOTSTRAP
    status = detectDroneState(NDS(DroneStateOffSet:SequenceNoOffSet-1));
    output = 0;
elseif length(NDS) >= 299
    % navData.MOde.DEMO
    % ------------------- start -----------------------
    
    % process battery info:
    batteryLevel = reArrange_and_decodeValue(NDS(Battery_OS:Battery_OS+data_length-1),1); % unit: %
    pitch = reArrange_and_decodeValue(NDS(Pitch_OS:Pitch_OS+data_length-1),0)/1000; % unit: deg
    roll = reArrange_and_decodeValue(NDS(Roll_OS:Roll_OS+data_length-1),0)/1000; % unit: deg
    yaw = reArrange_and_decodeValue(NDS(Yaw_OS:Yaw_OS+data_length-1),0)/1000;    % unit: deg
    altitude = reArrange_and_decodeValue(NDS(Altitude_OS:Altitude_OS+data_length-1),1)/1000; % unit: meter
    Vx = reArrange_and_decodeValue(NDS(Vx_OS:Vx_OS+data_length-1),0)/1000;  % unit: meter/s
    Vy = reArrange_and_decodeValue(NDS(Vy_OS:Vy_OS+data_length-1),0)/1000;  % unit: meter/s
    Vz = reArrange_and_decodeValue(NDS(Vz_OS:Vz_OS+data_length-1),0)/1000;  % unit: meter/s
    output = [batteryLevel
        pitch
        roll
        yaw
        altitude
        Vx
        Vy
        Vz];
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


