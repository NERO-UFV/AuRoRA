%% Enviar o comando de velocidade para as juntas do Pioneer
function vSendControlSignals(drone)
  
   
    for i=1:4
        erro = drone.vrep.simxSetStringSignal(drone.clientID,'particleVelocity',drone.pPar.W(i), drone.vrep.simx_opmode_oneshot);
%         erro = drone.vrep.simxSetJointTargetVelocity(drone.clientID, drone.MotorHandle(i), drone.pPar.W(i), drone.vrep.simx_opmode_streaming);
    end
end