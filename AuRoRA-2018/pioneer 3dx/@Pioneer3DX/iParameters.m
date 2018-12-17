function iParameters(p3dx)

p3dx.pPar.Model = 'P3DX'; % robot model

% Sample time
p3dx.pPar.Ts = 1/30;

% Dynamic Model Parameters 
p3dx.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration

% [kg] 
p3dx.pPar.m = 0.429; %0.442;  

% [kg] 
p3dx.pPar.a = 0.15; % point of control

p3dx.pPar.theta = [0.5338; 0.2168; -0.0134; 0.9560; -0.0843; 1.0590];  % Pioneer 3dx 
% p3dx.pPar.theta = [0.2604; 0.2509; -0.0005; 0.9965; 0.0026; 1.0768]; % Pioneer 3dx com laser

