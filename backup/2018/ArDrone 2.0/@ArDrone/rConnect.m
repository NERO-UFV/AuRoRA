function rConnect(drone)
% Function to create and open the connections with the ArDrone.


drone.pCom.controlChannel = udp(drone.pPar.ip, 5556, 'LocalPort', drone.pPar.LocalPortControl);
drone.pCom.stateChannel = udp(drone.pPar.ip, 5554, 'LocalPort', drone.pPar.LocalPortState);
drone.pCom.stateChannel.InputBufferSize = 910; % Define the buffer size
drone.pCom.SequenceNumber = tic;

drone.pFlag.Connected = 1;
drone.pFlag.BatteryLevel = 0;
try
    fopen(drone.pCom.controlChannel);
    fopen(drone.pCom.stateChannel);
    disp(['Connection established to ' drone.pPar.ip]);
    drone.rSetLed(1,10,1); % Blink Leds to show a successful connection
catch excp
    disp('failed to open udp channels.');
    disp(excp.message)
    fclose(drone.pCom.stateChannel);
    fclose(drone.pCom.controlChannel);
    return
end
