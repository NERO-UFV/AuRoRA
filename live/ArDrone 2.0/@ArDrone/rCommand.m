function rCommand(drone)

% Keep sending this command can maintain the control
%  e.g. : send this function with a rotate command once, the command will
%         last only for a while (maybe 0.1,0.2sec). The drone will rotate
%         just a little bit.
%
% --------------------- input & requirements ---------------
% -----******** pls read carefully, breaking the requirements leads to
% failure execution of this command ******---------
%
% 1. SequenceNumber
% 2. controlChannel : UDP('192.168.1.1', 5556, 'LocalPort', 5556)
% 3. falg: flag enabling the use of progressive commands and/or the Combined Yaw mode (bitfield)
%     Always set the flag bit 0 to 1 to make the drone consider the other arguments.
%     Setting it to 0 makes the drone enter hovering mode
%     (staying on top of the same point on the ground).
%   Thus, a recommended vaule for flag is 1 if flight control is desired.
% 4. LR_tilt: drone left-right tilt - floating-point value in range [-1..1]
%             A negative value makes the drone tilt to its left
% 5. FB_tilt: drone front-back tilt - floating-point value in range [-1..1]
%             A negative value makes the drone lower its nose
% 6. VerticalVel: drone vertical speed - floating-point value in range [-1..1]
%             A positive value makes the drone rise in the air
% 7. AngularVel: drone angular speed - floating-point value in range [-1..1]
%             A negative value makes the Drone spin left
%
% --------------------- output ---------------------------
% The return value, SequenceNumber, is the next sequence user can use.
% --------------------- Note -----------------------------
% if you want to continuouly send this command, you need to mannually add pause or wait
% (e.g. wait(0.2)) in your loop. otherwise the UDP may cannot accept high
% frequence update.



if abs(drone.pSC.Ud(1))<=1 && abs(drone.pSC.Ud(2))<=1 && ...
        abs(drone.pSC.Ud(3))<=1 && abs(drone.pSC.Ud(4))<=1
    
    Sig = sprintf('AT*PCMD=%d,%d,%d,%d,%d,%d\r',...
        drone.pCom.SequenceNumber,...
        1,...
        ARDrone_FloatArg2Int(drone.pSC.Ud(1)),...
        ARDrone_FloatArg2Int(drone.pSC.Ud(2)),...
        ARDrone_FloatArg2Int(drone.pSC.Ud(3)),...
        ARDrone_FloatArg2Int(drone.pSC.Ud(4)));
    try
        fprintf(drone.pCom.controlChannel, Sig);
    catch
        disp('Exception in MotionCommand()...');
    end
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
end

end

% =========================================================================
function intValue = ARDrone_FloatArg2Int(floatValue)
hex = sprintf('%tx',floatValue);
for i=1:length(hex)
    if hex(i)=='f'
        bin((i*4)-3:i*4)=[1 1 1 1];
    elseif hex(i)=='e'
        bin((i*4)-3:i*4)=[1 1 1 0];
    elseif hex(i)=='d'
        bin((i*4)-3:i*4)=[1 1 0 1];
    elseif hex(i)=='c'
        bin((i*4)-3:i*4)=[1 1 0 0];
    elseif hex(i)=='b'
        bin((i*4)-3:i*4)=[1 0 1 1];
    elseif hex(i)=='a'
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
intValue = 0;
for i = 2: length(bin)
    intValue = intValue + bin(i)*(2^(length(bin)-i));
end
if bin(1) == 1
    intValue = intValue - 2^(length(bin)-1);
end
intValue = int32(intValue);
end