function rEmergency(drone)

% Emergency only when altitude is lower than 0.40 meters
if drone.pPos.X(3) < 0.40    
    Sig = sprintf('AT*REF=%d,290717952\r',drone.pCom.SequenceNumber);
    try
        fprintf(drone.pCom.controlChannel, Sig);
    catch
    end
end

drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;

Sig = sprintf('AT*REF=%d,290717696\r',drone.pCom.SequenceNumber);
try
    fprintf(drone.pCom.controlChannel, Sig);
catch
end

drone.pCom.SequenceNumber = drone.pCom.SequenceNumber + 1;

disp(['Battery Level: ' num2str(drone.pPar.Battery) '%'])

end