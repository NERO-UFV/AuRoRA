%% Enviar o comando de postura para o robô
function vSetPose(drone,vector)

vector(3) = 0.1586;

drone.pPos.Xc([1 2 3 6]) = vector;

drone.pPos.X = drone.pPos.Xc;

% drone.pPos.X([1 2]) = drone.pPos.Xc([1 2]) + [drone.pPar.a*cos(drone.pPos.Xc(6)); drone.pPar.a*sin(drone.pPos.Xc(6))]; 

% drone.pPos.Xd = drone.pPos.X;
    
[erro]= drone.vrep.simxSetObjectPosition(drone.clientID,drone.DroneHandle,...
    -1,drone.pPos.Xc([1 2 3]),drone.vrep.simx_opmode_oneshot_wait);
erro
[erro]= drone.vrep.simxSetObjectOrientation(drone.clientID,drone.DroneHandle,...
    -1,drone.pPos.Xc([4 5 6]),drone.vrep.simx_opmode_oneshot_wait);
erro    
end