%% Desconectando o Matlab ao V-Rep via servidor cliente

% Now send some data to V-REP in a non-blocking fashion:
function vDisconnect(drone)
    drone.vrep.simxAddStatusbarMessage(drone.clientID,'Disconnected to Matlab',drone.vrep.simx_opmode_oneshot);

    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    drone.vrep.simxGetPingTime(drone.clientID);

    % Now close the connection to V-REP:    
    drone.vrep.simxFinish(drone.clientID);
    drone.vrep.delete(); % call the destructor!

    disp('Communication ended');
end