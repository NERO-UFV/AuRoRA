function rGetSensorCalibration(drone)
% This function ... (?)

%  pitch        unit: angle    world ref    2
%  roll         unit: angle    world ref    3
%  yaw          unit: angle    world ref    4

if drone.pFlag.Connected
    disp('Calibration: Start ..........')
    
    t = tic;
    tc = tic;
    
    Xo = [];
    
    while toc(t) < 3
    end
    
    while toc(t) < 5
        if toc(tc) > 1/30
            % --------------------------------------------------------------------
            % Real ArDrone2.0
            drone.rGetStatusRawData;
            % drone.rGetStatusRawDataFull;
            
            % --------------------------------------------------------------------
            % X variables
            % Atitude - Angles
            drone.pPos.X(4)  =  drone.pCom.cRawData(3)*pi/180; % Roll   (Phi)   [rad]
            drone.pPos.X(5)  = -drone.pCom.cRawData(2)*pi/180; % Pitch  (Theta) [rad]
            drone.pPos.X(6)  = -drone.pCom.cRawData(4)*pi/180; % Yaw    (Psi)   [rad]
            
            % Rotational matrix
            Rx = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
            Ry = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
            Rz = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];
            
            R = Rz*Ry*Rx;
            
            % Wn = [1 0 -sin(drone.pPos.X(5)); 0 cos(drone.pPos.X(4)) sin(drone.pPos.X(4))*cos(drone.pPos.X(5)); 0 -sin(drone.pPos.X(4)) cos(drone.pPos.X(4))*cos(drone.pPos.X(5))];
            
            % Velocity: Vx, Vy, Vz [m/s]
            drone.pPos.X(7:9) = R*[1 0 0; 0 -1 0; 0 0 -1]*drone.pCom.cRawData(6:8); % Conversão NED para XYZ
                        
            Xo = [Xo drone.pPos.X];
        end
    end
    drone.pPos.Xo = mean(Xo,2);
    disp('Calibration: Finished .........')
end