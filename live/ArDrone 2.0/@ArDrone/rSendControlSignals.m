function rSendControlSignals(drone)

if drone.pFlag.Connected == 1
    if drone.pCom.cStatus(32)==1    % flying state flag (1=flying/0=landed)
        % Experiment Mode: Ardrone 2.0
        drone.rCommand;
    end
else
    % Simulation Mode
    drone.pSC.U = drone.pSC.Ud;
    drone.sDynamicModel;
end

% Stand-by mode
drone.pSC.Ud = [0; 0; 0; 0];
