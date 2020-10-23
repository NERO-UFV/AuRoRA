%% Função para pegar as informações de Velocidade, Posição e Postura

function vGetSensorData(drone)

%% Posição do GPS acoplado ao Pioneer
[erro,drone.pPos.X(1:3,1)]= drone.vrep.simxGetObjectPosition(drone.clientID,drone.DroneHandle,-1,drone.vrep.simx_opmode_buffer);


%% Orientação do Pioneer
[erro,drone.pPos.X(4:6,1)] = drone.vrep.simxGetObjectOrientation(drone.clientID,drone.DroneHandle,-1,drone.vrep.simx_opmode_buffer);


% alpha, beta e gama

%% Velocidade do Pioneer
[erro,drone.pPos.X(7:9,1), drone.pPos.X(10:12,1)]= drone.vrep.simxGetObjectVelocity(drone.clientID,drone.DroneHandle,drone.vrep.simx_opmode_buffer);




end