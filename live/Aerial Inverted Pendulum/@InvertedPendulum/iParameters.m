function iParameters(pendulum)
% Dynamic Model Parameters 
pendulum.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration
pendulum.pPar.r = 0.70;   % [m] Pivot rod length
pendulum.pPar.R = 0.025;  % [m] Ball Radius
pendulum.pPar.m = 0.100;  % [kg] Pendulum's mass => MEDIR PESO!

% [kg.m^2] Pendulum's moment of inertia
pendulum.pPar.I = (pendulum.pPar.g)*((pendulum.pPar.r)^2);
