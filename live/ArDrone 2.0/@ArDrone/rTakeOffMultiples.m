function rTakeOffMultiples(drone1,drone2)

disp('Try to take off...')
for i = 1:10 % ask for drone state for 10 times
    rGetStatusRawData(drone1);
    rGetStatusRawData(drone2);
    if (length(drone1.pCom.cStatus) == 32) && (length(drone2.pCom.cStatus) == 32)
        break;
    end
end

if i >= 10 && length(drone1.pCom.cStatus)<32 || length(drone2.pCom.cStatus)<32
    FailTakeOff(drone1);
    FailTakeOff(drone2);
    return
else
    
    % Set the reference for horizontal plane
    fprintf(drone1.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'FTRIM', drone1.pCom.SequenceNumber, ''));
    drone1.pCom.SequenceNumber = drone1.pCom.SequenceNumber + 1;
    fprintf(drone2.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'FTRIM', drone2.pCom.SequenceNumber, ''));
    drone2.pCom.SequenceNumber = drone2.pCom.SequenceNumber + 1;
    
    % Set maximum altitude [default 3000 mm]
    fprintf(drone1.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'CONFIG', drone1.pCom.SequenceNumber, '"control:altitude_max","2700"'));
    drone1.pCom.SequenceNumber = drone1.pCom.SequenceNumber + 1;
    fprintf(drone2.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'CONFIG', drone2.pCom.SequenceNumber, '"control:altitude_max","2700"'));
    drone2.pCom.SequenceNumber = drone2.pCom.SequenceNumber + 1;
      
    % Send Takeoff command
    %290718208 = (00010001010101000000001000000000)
    fprintf(drone1.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'REF', drone1.pCom.SequenceNumber, '290718208'));
    drone1.pCom.SequenceNumber = drone1.pCom.SequenceNumber + 1;
    fprintf(drone2.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'REF', drone2.pCom.SequenceNumber, '290718208'));
    drone2.pCom.SequenceNumber = drone2.pCom.SequenceNumber + 1;
    mark = 0;
    count = 1;
    while count<10
        rGetStatusRawData(drone1);
        rGetStatusRawData(drone2);
        %         DroneData
        if length(drone1.pCom.cStatus)==32 && length(drone2.pCom.cStatus)==32 
            if drone1.pCom.cStatus(32)==1 && drone2.pCom.cStatus(32)==1% in flying state
                disp('in flight state');
                mark = 1;
                break;
            else
                if drone1.pCom.cStatus(1)==1 && drone2.pCom.cStatus(1)==1% % in ground and in emergency state
                    disp('In emergency state...')
                    disp('Set to normal state...')
                    disp('If the drone does not take off, pls clear out emergency cause, and then reconnect the drone battery.')
                    Sig1 = sprintf('AT*REF=%d,290718464\r',drone1.pCom.SequenceNumber);%(00010001010101000000001100000000)
                    Sig2 = sprintf('AT*REF=%d,290718464\r',drone2.pCom.SequenceNumber);%(00010001010101000000001100000000)
                    try
                        fprintf(drone1.pCom.controlChannel, Sig1);
                        fprintf(drone2.pCom.controlChannel, Sig2);
                    catch
                    end
                    pause(1);
                    %                     mark = 1;
                    %                     break;
                else % in ground but in normal state
                    disp('try to take off again...')
                    Sig1 = sprintf('AT*REF=%d,290718208\r',drone1.pCom.SequenceNumber);%(00010001010101000000001000000000)
                    Sig2 = sprintf('AT*REF=%d,290718208\r',drone2.pCom.SequenceNumber);%(00010001010101000000001000000000)
                    try
                        fprintf(drone1.pCom.controlChannel, Sig1);
                        fprintf(drone2.pCom.controlChannel, Sig2);
                    catch
                    end
                end
                drone1.pCom.SequenceNumber = drone1.pCom.SequenceNumber + 1;
                drone2.pCom.SequenceNumber = drone2.pCom.SequenceNumber + 1;
            end
        end
        count = count + 1;
    end
    
    if mark == 1
        disp('Take off successfully...');
    else
        FailTakeOff(drone1);
        FailTakeOff(drone2);
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