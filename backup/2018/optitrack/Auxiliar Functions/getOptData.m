function rigidBody = getOptData(optData,rigidBody)
% This function assign optitrack data to rigid body posture coordinates.
% Is considered that the data received is related to the body center.
% *For pioneer, this value is recalculated to find the control point
% posture (X)
%
% optData   = Rigid Body structure of the robot
% rigidBody = Object class variable (Pioneer, Ardrone,Load,Obstacle)

type = class(rigidBody);      % rigid body class name

if optData.isTracked
    
    rigidBody.pPos.Xa = rigidBody.pPos.X;  % Save previous data for derivative
    
    % Converts quartenion(xyzw) to euler angles(ZYX)
    q      = quaternion(optData.Quaternion(2),optData.Quaternion(3),optData.Quaternion(4),optData.Quaternion(1)); % reorder quaternion
    qRot   = quaternion( 0, 0, 0, 1);    % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
    q      = mtimes(q, qRot);
    angles = EulerAngles(q,'ZYX');
    
    roll  = angles(1);
    pitch = -angles(2);   % must invert due to 180 flip above
    yaw   = angles(3);
    
    % In case the rigid body is Pioneer
    if type(1) == 'P' || type(1) == 'p'
        % Position [m]
        rigidBody.pPos.Xc([1 2 3]) = optData.Position([1 2 3])/1000;   % x,y,z
        
        % Euler Angles [rad]
        rigidBody.pPos.Xc(6) = yaw ;    % psi
        
        % Calculate control point position
        rigidBody.pPos.X([1 2 6]) = rigidBody.pPos.Xc([1 2 6]) + ...
            [rigidBody.pPar.a*cos(rigidBody.pPos.Xc(6)); rigidBody.pPar.a*sin(rigidBody.pPos.Xc(6)); 0];
    else
        
        % For Ardrone, Load and Obstacle
        % Position [m]
        rigidBody.pPos.X([1 2 3]) = optData.Position([1 2 3])/1000;   % x,y,z
        
        % Euler Angles [rad]
        rigidBody.pPos.X([4 5 6]) = [roll pitch yaw];
       
    end
    
    % Derivative to obtain velocities - for all rigid body
    dt = rigidBody.pPar.Ts;    % sample time
    
    rigidBody.pPos.X(7:9)   = (rigidBody.pPos.X(1:3)-rigidBody.pPos.Xa(1:3))/dt;    % dX,dY,dZ
    rigidBody.pPos.X(10:11) = (rigidBody.pPos.X(4:5)-rigidBody.pPos.Xa(4:5))/dt;  % phi,theta
    
    if abs(rigidBody.pPos.X(6) - rigidBody.pPos.Xa(6)) > pi                       % Yaw Test
        if rigidBody.pPos.Xa(6) < 0
            rigidBody.pPos.Xa(6) =  2*pi + rigidBody.pPos.Xa(6);
        else
            rigidBody.pPos.Xa(6) = -2*pi + rigidBody.pPos.Xa(6);
        end
    end
    rigidBody.pPos.X(12) = (rigidBody.pPos.X(6)-rigidBody.pPos.Xa(6))/dt;          % dPsi
   
else
    disp('No rigid body tracked!');
end
