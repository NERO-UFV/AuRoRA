if robot_type == 'A'
        switch ron
            case 'Quadricopter'  
                %% Carregando os objetos

                [erro,vp3dx.DroneHandle] = vp3dx.vrep.simxGetObjectHandle(vp3dx.clientID, answer ,vp3dx.vrep.simx_opmode_oneshot_wait);
                if (erro > 0)
                    disp('Object Drone not found!')
                else
    %                 disp('Pioneer 3DX found!')
                end

                [erro,vp3dx.pSC.U(1:3,1),vp3dx.pSC.U(4:6,1)] = vp3dx.vrep.simxGetObjectVelocity(vp3dx.clientID,vp3dx.DroneHandle,vp3dx.vrep.simx_opmode_streaming);
                [erro,vp3dx.pPos.Xc(1:3,1)] = vp3dx.vrep.simxGetObjectPosition(vp3dx.clientID,vp3dx.DroneHandle,-1,vp3dx.vrep.simx_opmode_streaming);
                [erro,vp3dx.pPos.Xc(4:6,1)] = vp3dx.vrep.simxGetObjectOrientation(vp3dx.clientID,vp3dx.DroneHandle,-1,vp3dx.vrep.simx_opmode_streaming);

                %% inicializacao dos motores
                for i=1:4
                    motor = [answer '_propeller_joint' num2str(i)];
                    [erro,vp3dx.MotorHandle(i)] = vp3dx.vrep.simxGetObjectHandle(vp3dx.clientID, motor ,vp3dx.vrep.simx_opmode_oneshot_wait);
                    if (erro > 0)
                        msg = ['Object Propeller ' num2str(i) ' not found!'];
                        disp(msg)
                    else
        %                 disp('left Motor found!!')
                    end
                end

                disp('Done!')