function robot = getOptData(opt,robot)
% This function assign optitrack data to robot pose coordinates.
% Is considered that the data received is related to the robot center.
% For pioneer, this value is recalculated to fit the control point.
% 
% 
% get optitrack data
rb = opt.RigidBody;

try
    if rb.isTracked
        
        % Converts quartenion(wxyz) to euler angles(ZYX)
        quat = rb.Quaternion;
        eul = *.71(quat);
        
        switch class(robot)
            
            case 'Pioneer3DX'
                
                % Position [m]
                robot.pPos.Xc(1) = rb.Position(1)/1000;   % x
                robot.pPos.Xc(2) = rb.Position(2)/1000;   % y
                robot.pPos.Xc(3) = rb.Position(3)/1000;   % z
                             
                % Euler Angles [rad]
%                 robot.pPos.Xc(4) = -eul(3) ;    % phi
%                 robot.pPos.Xc(5) = -eul(2) ;    % theta
                robot.pPos.Xc(6) = -eul(1) ;    % psi
                
                 % Obtenção da posição do ponto de controle
                robot.pPos.X([1 2 6]) = robot.pPos.Xc([1 2 6]) + ...
                    [robot.pPar.a*cos(P.pPos.Xc(6)); robot.pPar.a*sin(robot.pPos.Xc(6)); 0];
                
            case 'ArDrone'
                
                robot.pPos.Xa = robot.pPos.X;
                
                % Position [m]
                robot.pPos.X(1) = rb.Position(1)/1000;   % x
                robot.pPos.X(2) = rb.Position(2)/1000;   % y
                robot.pPos.X(3) = rb.Position(3)/1000;   % z
                
                % Euler Angles [rad]
                robot.pPos.X(4) = -eul(3) ;    % phi
                robot.pPos.X(5) = -eul(2) ;    % theta
                robot.pPos.X(6) = -eul(1) ;    % psi
                                
        end
    end
catch
    disp('Rigid body not tracked or Optitrack not connected....');
end
end