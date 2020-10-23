function iParameters(pendulum)
% Sample time
pendulum.pPar.t   = tic; % Current Time
pendulum.pPar.Ts  = 1/30; % ArDrone
pendulum.pPar.ti  = 0; % Flag time 
pendulum.pPar.Tsm = pendulum.pPar.Ts/4; % Motor

% Dynamic Model Parameters 
pendulum.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration
pendulum.pPar.r = 0.70;   % [m] Pivot rod length
pendulum.pPar.m = 0.429;  % [kg] Drone's mass    
pendulum.pPar.mc = 0.100; % [kg] Pendulum's mass => MEDIR PESO!
% [kg.m^2] Pendulum's moment of inertia
pendulum.pPar.I = (pendulum.pPar.g)*((pendulum.pPar.r)^2);
% [kg.m^2] pendulum's moments of Inertia
pendulum.pPar.Ix = 2.237568e-3; % 9.57*1e-3;
pendulum.pPar.Iy = 2.985236e-3; % 9.57*1e-3;
pendulum.pPar.Iz = 4.80374e-3;  % 25.55*1e-3;
pendulum.pPar.Ixx = 2.237568e-3; % 9.57*1e-3;
pendulum.pPar.Iyy = 2.985236e-3; % 9.57*1e-3;
pendulum.pPar.Izz = 4.80374e-3;  % 25.55*1e-3;

pendulum.pPar.Ixy = 0;
pendulum.pPar.Ixz = 0;
pendulum.pPar.Iyz = 0;

pendulum.pPar.Ixy = 0;
pendulum.pPar.Ixz = 0;
pendulum.pPar.Iyz = 0;

pendulum.pPar.F = zeros(8,1);


% Model disturbances (FOR FUTURE USE)
pendulum.pPar.D = zeros(6,1); 
pendulum.pPar.Q = zeros(3,1); 

% q_desired = [alpha beta phi theta]

% Coupling parameters
pendulum.pPar.k1 = 0.1785; 
pendulum.pPar.k2 = 0.0301; 

% Rotor Parameters
pendulum.pPar.r1 = 8.625; % Reduction Gear
pendulum.pPar.R = 0.6029; % Motor resistance
pendulum.pPar.Jm = 0.1215; %2.029585e-5; 
pendulum.pPar.Bm = 3.7400; %1.06e-3;
pendulum.pPar.Km = 1.3014e2; %0.39;
pendulum.pPar.Kb = 1.3014e-3; %8e-5;

pendulum.pPar.Cf = 8.048e-6; 
pendulum.pPar.Ct = 2.423e-7; 

% Low-level PD controller gains
pendulum.pPar.kdp = 0.1; 
pendulum.pPar.kpp = 0.1;
pendulum.pPar.kdt = 0.1;
pendulum.pPar.kpt = 0.1;
pendulum.pPar.kds = 0.05; 
pendulum.pPar.kps = 0.1; 
pendulum.pPar.kdz = 0.05; 
pendulum.pPar.kpz = 5;

% Motor voltage in hovering stage
pendulum.pPar.Wo = sqrt(pendulum.pPar.m*pendulum.pPar.g/4/pendulum.pPar.Cf);
% Rotor velocities
pendulum.pPar.W = zeros(4,1); 

pendulum.pPar.Vo = (pendulum.pPar.R*pendulum.pPar.Bm/pendulum.pPar.Km + pendulum.pPar.Kb)*pendulum.pPar.Wo + ...
    pendulum.pPar.R/pendulum.pPar.r/pendulum.pPar.Km*pendulum.pPar.Ct*pendulum.pPar.Wo^2;
% Saturation values
%     pitch          | [-1,1] <==> [-15,15] degrees
%     roll           | [-1,1] <==> [-15,15] degrees
%     altitude rate  | [-1,1] <==> [-1,1] m/s
%     yaw rate       | [-1,1] <==> [-100,100] degrees/s
pendulum.pPar.uSat    = zeros(4,1);
pendulum.pPar.uSat(1) = 15*pi/180;  % Max roll  angle reference 
pendulum.pPar.uSat(2) = 15*pi/180;  % Max pitch angle reference 
pendulum.pPar.uSat(3) = 1;          % Max altitude rate reference 
pendulum.pPar.uSat(4) = 100*pi/180; % Max yaw rate reference 

%For gain fine-tuning:
pendulum.pPar.Gains  = zeros(8,2);
pendulum.pPar.Gainsa = zeros(8,2);
pendulum.pPar.Erro   = zeros(8,2);

