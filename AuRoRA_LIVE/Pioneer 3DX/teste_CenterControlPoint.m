clear
close all
clc

%% Look for root path
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Create robots
P = Pioneer3DX;
P.pPar.a = 0;
P.pPar.alpha = 0;
% Define robot parameters
P.rConnect;             % robot or mobilesim
Xo = [-1 4 0 0];
P.rSetPose(Xo);         % set initial pose

% Desired positions
P.pPos.Xd([1 2 6]) = [4 2 0];

%% Initial errors
P.pPos.Xtil = P.pPos.Xd - P.pPos.X;

%% Parameters
a   = P.pPar.a;      % control point distance from robot center
dXd = [0;0;0];       % desired velocity (for trajectory tracking)
Kp1 = diag([0.75 0.75 1]); % maximum value gain
Kp2 = diag([1 1 1]); % saturation gain

kx   = 0.5;
ky   = 0.5;
kpsi = 0.5;

% Window
fig = figure(1);
plot(P.pPos.Xd(1),P.pPos.Xd(2),'kx','MarkerSize',15);
hold on;
%% Simulation
XX  = [];          % to save data
tap = 0.1;         % robot sample rate
tp  = tic;         % graphic update time
tc  = tic;         % robot send/receive time
t   = tic;         % simulation time

while toc(t) < 20
    if toc(tc) > tap
        
        tc = tic;    % reset clock
        
        % ---------------------------------------------------------------
        % Data aquisition
        P.rGetSensorData;
        
        % Error
        
        ux = P.pPos.Xd(7) + kx*(P.pPos.Xd(1)-P.pPos.X(1));
        uy = P.pPos.Xd(8) + ky*(P.pPos.Xd(2)-P.pPos.X(2));
        
        
        % Variables definition
        psi  = P.pPos.X(6);            % yaw angle
        
        % Inverse Kinematic matrix               
        if abs(P.pPar.alpha) ~= pi/2 && P.pPar.alpha ~= 0
            % Controller
            Ud(2) = (-sin(psi)*ux + cos(psi)*uy)/(-P.pPar.a*cos(P.pPar.alpha));
            Ud(1) = ux*cos(psi) + uy*sin(psi) + P.pPar.a*sin(P.pPar.alpha)*Ud(2);
        else            
            upsi = atan2(uy,ux) - P.pPos.X(6);
            Ud(2) = kpsi*upsi;
            Ud(1) = cos(psi)*ux + sin(psi)*uy + P.pPar.a*Ud(2);
        end
        
        P.pSC.Ud = Ud';
        
        % Send control to robot
        P.rSendControlSignals;
        
        % Save data
        XX = [XX; P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)];
        
    end
    
    %% Desenha os robôs
    
    if toc(tp) > tap
        tp = tic;
        P.mCADdel
        P.mCADplot(1,'k')
        axis([-1 5 -1 5])
        grid on
        drawnow
    end
    
end

