function rSendControlSignals(drone)

if drone.pFlag.Connected == 1
    % Experiment Mode: Ardrone 2.0
    drone.rCommand;
else
    % Simulation Mode
    drone.pSC.U = drone.pSC.Ud;
    drone.sDynamicModel;
end

% Stand-by mode
drone.pSC.Ud = [0; 0; 0; 0];
