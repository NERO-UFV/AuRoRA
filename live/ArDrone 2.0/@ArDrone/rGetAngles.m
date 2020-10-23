function rGetAngles(drone)
% Assing phi and roll angles data from rGetStatusRawData
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
    
%     drone.pPos.Xa = drone.pPos.X;
    % --------------------------------------------------------------------
    % X variables
    % Atitude velocities: d(phi)/dt, d(theta)/dt, d(psi)/dt [rad/s]
    angles = [1 0 0; 0 -1 0; 0 0 -1]*drone.pCom.cRawData([3 2 4])*pi/180;
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
    % drone.pPos.X(10:12) = (angles - drone.pPos.X(4:6))/drone.pPar.Ts;
    
    % Atitude - Angles
    drone.pPos.X(4)  = angles(1); % Roll   (Phi)   [rad]
    drone.pPos.X(5)  = angles(2); % Pitch  (Theta) [rad]
    % drone.pPos.X(6)  = angles(3); % Yaw    (Psi)   [rad]     
    
     
else
    % Simulation    -----------------------------------------------------------------
    drone.pPos.X(4:5) = drone.pPos.X(4:5);   
end
