function iParameters(load)

% Dynamic Model Parameters 
load.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration

% [kg] Load mass
load.pPar.m = 0.100; 
load.pPar.l = 1.000;
load.pPar.Ts = 1/30;

% [kg.m^2] Moments of Inertia
load.pPar.Ixx = 8.00-6; 
load.pPar.Iyy = 0.00-4;
load.pPar.Izz = 0.00-4; 

load.pPar.Ixy = 0;
load.pPar.Ixz = 0;
load.pPar.Iyz = 0;
    
% Pose reference
load.pPar.Xr  = zeros(12,1); 
load.pPar.Xra = zeros(12,1); 

% Model disturbances
load.pPar.D = zeros(3,1); 
load.pPar.Q = zeros(3,1); 