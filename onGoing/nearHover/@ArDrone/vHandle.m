%% Responsável por carregar todos Object Handles do V-Rep
%[erro,handles, intData, floatData,stringData]= p3dx.vrep.simxGetObjectGroupData(p3dx.clientID,p3dx.vrep.sim_appobj_object_type,0,p3dx.vrep.simx_opmode_blocking )
function vHandle(drone,robot_name,index)
    
    switch nargin
        case 1
            prompt = {'Necessary to enter the robot name. Example:'};
            title = 'Warning';
            definput = {'Quadricopter'};
            opts.Interpreter = 'tex';
            answer = inputdlg(prompt,title,[1 40],definput,opts);

            drone.vHandle(answer{1});
        case 2 
            %% inicializacao do Pioneer

            [erro,drone.DroneHandle] = drone.vrep.simxGetObjectHandle(drone.clientID,robot_name,drone.vrep.simx_opmode_oneshot_wait);
            if (erro > 0)
                disp('Object Quadricopter not found!')
            else
        %                 disp('Pioneer 3DX found!')
            end

            [erro,LinearVelocity, AngularVelocity] = drone.vrep.simxGetObjectVelocity(drone.clientID,drone.DroneHandle,drone.vrep.simx_opmode_streaming);
            [erro,drone.pPos.Xc(1:3,1)] = drone.vrep.simxGetObjectPosition(drone.clientID,drone.DroneHandle,-1,drone.vrep.simx_opmode_streaming);
            [erro,drone.pPos.Xc(4:6,1)] = drone.vrep.simxGetObjectOrientation(drone.clientID,drone.DroneHandle,-1,drone.vrep.simx_opmode_streaming);

            %% inicializacao dos motores
            for i=1:4
                motor = [robot_name '_propeller_joint' num2str(i)];
                [erro,drone.MotorHandle(i)] = drone.vrep.simxGetObjectHandle(drone.clientID, motor ,drone.vrep.simx_opmode_oneshot_wait);
                if (erro > 0)
                    disp('Object left Motor not found!')
                else
            %                 disp('left Motor found!!')
                end
                [erro, force]= drone.vrep.simxGetJointForce(drone.clientID, drone.MotorHandle(i), drone.vrep.simx_opmode_streaming);
            end
           
    case 3
        %% inicializacao do Pioneer
        name = [robot_name '#' index];
        
            [erro,drone.DroneHandle] = drone.vrep.simxGetObjectHandle(drone.clientID,name,drone.vrep.simx_opmode_oneshot_wait);
            if (erro > 0)
                disp('Object Pioneer 3DX not found!')
            else
        %                 disp('Pioneer 3DX found!')
            end

            [erro,LinearVelocity, AngularVelocity] = drone.vrep.simxGetObjectVelocity(drone.clientID,drone.DroneHandle,drone.vrep.simx_opmode_streaming);
            [erro,drone.pPos.Xc(1:3,1)] = drone.vrep.simxGetObjectPosition(drone.clientID,drone.DroneHandle,-1,drone.vrep.simx_opmode_streaming);
            [erro,drone.pPos.Xc(4:6,1)] = drone.vrep.simxGetObjectOrientation(drone.clientID,drone.DroneHandle,-1,drone.vrep.simx_opmode_streaming);

            %% inicializacao dos motores
            % Left Motor
            motor = [robot_name '_leftMotor#' index];
            %% inicializacao dos motores
            for i=1:4
                motor = [robot_name '_propeller_joint' num2str(i) '#' index];
                [erro,drone.MotorHandle(i)] = drone.vrep.simxGetObjectHandle(drone.clientID, motor ,drone.vrep.simx_opmode_oneshot_wait);
                if (erro > 0)
                    disp('Object left Motor not found!')
                else
            %                 disp('left Motor found!!')
                end
                [erro, force]= drone.vrep.simxGetJointForce(drone.clientID, drone.MotorHandle(i), drone.vrep.simx_opmode_streaming);
            end
        
    end
    disp('Done!');
end