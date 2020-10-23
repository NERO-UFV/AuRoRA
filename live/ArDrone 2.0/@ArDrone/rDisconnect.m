function rDisconnect(drone)
% Leds Blink Red
drone.rSetLed(2,10,1);

disp(['Battery Level: ' num2str(drone.pPar.Battery) '%'])

% Close communication objects
fclose(drone.pCom.stateChannel);
fclose(drone.pCom.controlChannel);