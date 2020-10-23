function iParameters(drone)

drone.pPar.Model = 'ArDrone2'; % robot type
drone.pPar.ip = ['192.168.1.' num2str(drone.pID)];
drone.pPar.LocalPortControl = 5556;
drone.pPar.LocalPortState = 5554;

% Sample time
drone.pPar.t   = tic; % Current Time
drone.pPar.Ts  = 1/30; % ArDrone
drone.pPar.ti  = tic; % Flag time 

drone.pPar.Tsm = drone.pPar.Ts/4; % Motor

% Dynamic Model Parameters 
drone.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration
drone.pPar.Altmax = 5000; 

% [kg] ArDrone mass
drone.pPar.m = 0.429; %0.442;  

% [kg.m^2] Moments of Inertia
drone.pPar.Ixx = 2.237568e-3; % 9.57*1e-3;
drone.pPar.Iyy = 2.985236e-3; % 9.57*1e-3;
drone.pPar.Izz = 4.80374e-3;  % 25.55*1e-3;

drone.pPar.Ixy = 0;
drone.pPar.Ixz = 0;
drone.pPar.Iyz = 0;

% Rotor Parameters
drone.pPar.r = 8.625; % Reduction Gear
drone.pPar.R = 0.6029; % Motor resistance
drone.pPar.Jm = 0.1215; %2.029585e-5; 
drone.pPar.Bm = 3.7400; %1.06e-3;
drone.pPar.Km = 1.3014e2; %0.39;
drone.pPar.Kb = 1.3014e-3; %8e-5;

drone.pPar.Cf = 8.048e-6; 
drone.pPar.Ct = 2.423e-7; 

% Low-level PD controller gains
drone.pPar.kdp = 0.1; 
drone.pPar.kpp = 0.1;
drone.pPar.kdt = 0.1;
drone.pPar.kpt = 0.1;
drone.pPar.kds = 0.05; 
drone.pPar.kps = 0.1; 
drone.pPar.kdz = 0.05; 
drone.pPar.kpz = 5;

% Propeller coeficients
drone.pPar.k1 = 0.1785; 
drone.pPar.k2 = drone.pPar.Ct/drone.pPar.Cf; 

% Saturation values
%     pitch          | [-1,1] <==> [-15,15] degrees
%     roll           | [-1,1] <==> [-15,15] degrees
%     altitude rate  | [-1,1] <==> [-1,1] m/s
%     yaw rate       | [-1,1] <==> [-100,100] degrees/s
drone.pPar.uSat    = zeros(4,1);
drone.pPar.uSat(1) = 15*pi/180;  % Max roll  angle reference 
drone.pPar.uSat(2) = 15*pi/180;  % Max pitch angle reference 
drone.pPar.uSat(3) = 1;          % Max altitude rate reference 
drone.pPar.uSat(4) = 100*pi/180; % Max yaw rate reference 
    
% Pose reference
drone.pPar.Xr  = zeros(12,1); 
drone.pPar.Xra = zeros(12,1); 

% Motor voltage in hovering stage
drone.pPar.Wo = sqrt(drone.pPar.m*drone.pPar.g/4/drone.pPar.Cf);

drone.pPar.Vo = (drone.pPar.R*drone.pPar.Bm/drone.pPar.Km + drone.pPar.Kb)*drone.pPar.Wo + ...
    drone.pPar.R/drone.pPar.r/drone.pPar.Km*drone.pPar.Ct*drone.pPar.Wo^2;

% Rotor velocities
drone.pPar.W = zeros(4,1); 

% Model disturbances
drone.pPar.D = zeros(6,1); 
drone.pPar.Q = zeros(3,1); 

% Linear Kalman Filter: rGetSensorData variables
% xp = [z phi theta psi dx dy]
drone.pPar.LKF.xp   = zeros(6,1); % Linear predictor
drone.pPar.LKF.mse  = diag(randn(1,6)*0.01); % Mean square error
drone.pPar.LKF.varw = diag([3e-3 3e-2 3e-2 3e-4 3e-3 3e-3]); % State/Observation variance
drone.pPar.LKF.varn = diag([4e-6 8e-6 8e-6 8e-7 4e-6 4e-6]); % System variance

% xd = [z phi theta psi dx dy]
drone.pPar.LKF.xd   = zeros(2,1); % Linear predictor
drone.pPar.LKF.msed  = diag(randn(1,2)*0.01); % Mean square error
drone.pPar.LKF.varwd = diag([3e-8 3e-8]); % State/Observation variance
drone.pPar.LKF.varnd = diag([8e-8 8e-8]); % System variance
