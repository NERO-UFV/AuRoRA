function rLand(drone)

disp('Try to land ...');

% Decimal to Binary: Command
% (290717696: 00010001010101000000000000000000)
Sig = sprintf('AT*REF=%d,290717696\r',drone.pCom.SequenceNumber);
try
    fprintf(drone.pCom.controlChannel, Sig);
catch
end

drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
count = 0;
while count <= 5
%      rGetStatusRawData(drone)
    rGetStatusRawData(drone)
    if length(drone.pCom.cStatus) == 32
        if drone.pCom.cStatus(32) == 0
            disp('Land successfully ... ');
            break;
        else
            % Decimal to Binary: Command
            % (290717696: 00010001010101000000000000000000)
            Sig = sprintf('AT*REF=%d,290717696\r',drone.pCom.SequenceNumber);
            try
                fprintf(drone.pCom.controlChannel, Sig);
            catch
            end
            drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;
            disp('Try to land ...');
            pause(2);
        end
    end
    count = count + 1;
end
rGetStatusRawData(drone)
disp(['Battery Level: ' num2str(drone.pPar.Battery) '%'])
end