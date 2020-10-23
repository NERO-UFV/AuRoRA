function rTakeOff(drone)

disp('Try to take off...')
for i = 1:10 % ask for drone state for 10 times
    rGetStatusRawData(drone);
    if length(drone.pCom.cStatus) == 32
        break;
    end
end

if i >= 10 && length(drone.pCom.cStatus)<32
    FailTakeOff(drone);
    return
else
    
    % Set the reference for horizontal plane
    fprintf(drone.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'FTRIM', drone.pCom.SequenceNumber, ''));
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
    
    % Set maximum altitude [default 3000 mm]
    fprintf(drone.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'CONFIG', drone.pCom.SequenceNumber, '"control:altitude_max","2700"'));
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
    
%     % Set outdoor mode
%     fprintf(drone.pCom.controlChannel, ...
%         sprintf( 'AT*%s=%i,%s\r', 'CONFIG', drone.pCom.SequenceNumber,'"control:outdoor","FALSE"'));
%     drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
        
    % Send Takeoff command
    %290718208 = (00010001010101000000001000000000)
    fprintf(drone.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'REF', drone.pCom.SequenceNumber, '290718208'));
    drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
    
    mark = 0;
    count = 1;
    while count<10
        rGetStatusRawData(drone);
        %         DroneData
        if length(drone.pCom.cStatus) == 32
            if drone.pCom.cStatus(32)==1 % in flying state
                disp('in flight state');
                mark = 1;
                break;
            else
                if drone.pCom.cStatus(1) == 1 % in ground and in emergency state
                    disp('In emergency state...')
                    disp('Set to normal state...')
                    disp('If the drone does not take off, pls clear out emergency cause, and then reconnect the drone battery.')
                    Sig = sprintf('AT*REF=%d,290718464\r',drone.pCom.SequenceNumber);%(00010001010101000000001100000000)
                    try
                        fprintf(drone.pCom.controlChannel, Sig);
                    catch
                    end
                    pause(1);
                    %                     mark = 1;
                    %                     break;
                else % in ground but in normal state
                    disp('try to take off again...')
                    Sig = sprintf('AT*REF=%d,290718208\r',drone.pCom.SequenceNumber);%(00010001010101000000001000000000)
                    try
                        fprintf(drone.pCom.controlChannel, Sig);
                    catch
                    end
                end
                drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
            end
        end
        count = count + 1;
    end
    
    if mark == 1
        disp('Take off successfully...');
    else
        FailTakeOff(drone);
    end
    
end

end


function FailTakeOff(drone)
try
    fprintf(drone.pCom.controlChannel, sprintf('AT*REF=%d,290717696\r',drone.pCom.SequenceNumber)); % take a normal land
catch
end
drone.pCom.SequenceNumber = -1;
disp('Wireless connection problem...');
disp('Cannot take off...(re-connect the drone power probably can solve the problem).');
end