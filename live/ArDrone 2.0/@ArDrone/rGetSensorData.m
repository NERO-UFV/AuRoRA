function rGetSensorData(drone)
% Assing data from rGetStatusRawData
% The output vector is an 8*1 array in which the structure looks like:
% [batteryLevel unit: %                     1
%  pitch        unit: angle    world ref    2
%  roll         unit: angle    world ref    3
%  yaw          unit: angle    world ref    4
%  altitude     unit: meter    surface ref  5
%  Vx           unit: m/s      body ref     6
%  Vy           unit: m/s      body ref     7
%  Vz           unit: m/s      body ref     8
%  Accx         unit: m/s^2    body ref     9
%  Accy         unit: m/s^2    body ref     10
%  Accz         unit: m/s^2    body ref     11
%  gyrox        unit: angle/s  world ref    12
%  gyroy        unit: angle/s  world ref    13
%  gyroz        unit: angle/s  world ref    14

if drone.pFlag.Connected
    % --------------------------------------------------------------------
    % Real ArDrone2.0
    drone.rGetStatusRawData;
    % drone.rGetStatusRawDataFull;
    
    if drone.pPar.ti < 10
        drone.pPar.ti = drone.pPar.ti + 1;
    else
        drone.pPar.Ts = toc(drone.pPar.t);
    end
    drone.pPar.t  = tic;
    
    drone.pPos.Xa = drone.pPos.X;
    % --------------------------------------------------------------------
    % X variables
    % Atitude velocities: d(phi)/dt, d(theta)/dt, d(psi)/dt [rad/s]
    % Comentado na ComicCon
    angles(1)  =  drone.pCom.cRawData(3)*pi/180; % Roll   (Phi)   [rad]
    angles(2)  = -drone.pCom.cRawData(2)*pi/180; % Pitch  (Theta) [rad]
    angles(3)  = -drone.pCom.cRawData(4)*pi/180; % Yaw    (Psi)   [rad]
    
    % angles = [1 0 0; 0 -1 0; 0 0 -1]*drone.pCom.cRawData([3 2 4])*pi/180;
    % Bias yaw angle: PSI
    angles(3) = angles(3) - drone.pPos.Xo(6);
    if abs(angles(3)) > pi
        if angles(3) > 0
            angles(3) = angles(3) - 2*pi;
        else
            angles(3) = angles(3) + 2*pi;
        end
    end
    
    % Angular time derivative makes the ArDrone unstable
    %     drone.pPos.X(10:12) = (angles - drone.pPos.X(4:6))/drone.pPar.Ts;
    
    % Atitude - Angles
    drone.pPos.X(4)  = angles(1); % Roll   (Phi)   [rad]
    drone.pPos.X(5)  = angles(2); % Pitch  (Theta) [rad]
    drone.pPos.X(6)  = angles(3); % Yaw    (Psi)   [rad]
    
    % Rotational matrix
    Rx = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
    Ry = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
    Rz = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];
    
    R = Rz*Ry*Rx;
    
    %     Wn = [1 0 -sin(drone.pPos.X(5)); 0 cos(drone.pPos.X(4)) sin(drone.pPos.X(4))*cos(drone.pPos.X(5)); 0 -sin(drone.pPos.X(4)) cos(drone.pPos.X(4))*cos(drone.pPos.X(5))];
    
    % Velocity: Vx, Vy, Vz [m/s]
    drone.pPos.X(7:9) = R*[1 0 0; 0 -1 0; 0 0 -1]*drone.pCom.cRawData(6:8,1); % Conversão NED para XYZ
    
    % Valid altitude data
    if drone.pCom.cRawData(5) > 0.20
        drone.pPos.X(3) = drone.pCom.cRawData(5); % z-Altitude [m]
    end
    
    % Linear Kalman filter
    GainK = (drone.pPar.LKF.mse + drone.pPar.LKF.varn)\drone.pPar.LKF.mse;
    drone.pPar.LKF.xp  = drone.pPar.LKF.xp + GainK*(drone.pPos.X(3:8) - drone.pPar.LKF.xp);
    drone.pPar.LKF.mse = (eye(6)-GainK)*drone.pPar.LKF.mse + drone.pPar.LKF.varw;
    drone.pPos.X(3:8)  = drone.pPar.LKF.xp;
    
    % Position: Numerical integration [m]
    drone.pPos.X(1:2) = drone.pPos.X(7:8)*drone.pPar.Ts + drone.pPos.X(1:2);
    
    % Valid altitude data
    fe = 0.7;
    if drone.pCom.cRawData(5) > 0.25 %[m]
        % Digital low-pass filter for z-velocity
        drone.pPos.X(9) = fe*drone.pPos.X(9) + (1-fe)*(drone.pPos.Xa(3)-drone.pPos.X(3))/drone.pPar.Ts;
    end
    drone.pPos.X(10:12) = fe*drone.pPos.X(10:12) + (1-fe)*(drone.pPos.Xa(10:12)-drone.pPos.X(10:12))/drone.pPar.Ts;
    
    % --------------------------------------------------------------------
    % dX variables
    % Velocity
    drone.pPos.dX(1)  = drone.pPos.X(7); % Vx [m/s]
    drone.pPos.dX(2)  = drone.pPos.X(8); % Vy [m/s]
    drone.pPos.dX(3)  = drone.pPos.X(9); % Vz [m/s]
    
    drone.pPos.dX(4)  = drone.pPos.X(10); % d(phi)/dt   [rad/s]
    drone.pPos.dX(5)  = drone.pPos.X(11); % d(theta)/dt [rad/s]
    drone.pPos.dX(6)  = drone.pPos.X(12); % d(psi)/dt   [rad/s]
    %
    %     % Acceleration
    %     drone.pPos.dX(7)  = 0; %  drone.pCom.cRawData(9);  % Accx [m/s]
    %     drone.pPos.dX(8)  = 0; % -drone.pCom.cRawData(10); % Accy [m/s]
    %     drone.pPos.dX(9)  = 0; % -drone.pCom.cRawData(11); % Accz [m/s]
    %
    %     drone.pPos.dX(10) = 0;
    %     drone.pPos.dX(11) = 0;
    %     drone.pPos.dX(12) = 0;
    
else
    % Simulation    -----------------------------------------------------------------
    drone.pPos.X = drone.pPos.X;
end
