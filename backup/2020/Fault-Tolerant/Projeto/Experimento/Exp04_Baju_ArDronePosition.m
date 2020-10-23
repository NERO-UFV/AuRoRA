clearvars
close all
clc

try
    fclose(instrfindall);
catch
end

%% Load Class
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Numero de drones
nDrones = 2;

% Load ArDrone
for ii = 1:nDrones
    A{ii} = ArDrone(ii);
end
for ii = 1:nDrones
    % detect rigid body ID from optitrack
    idA{ii} = getID(OPT,ArDrone,ii);     % ID do ardrone no optitrack
    
    %
    rb = OPT.RigidBody;          % read optitrack data
    A{ii} = getOptData(rb(idA{ii}),A{ii});   % get pioneer data
end

% Joystick
J = JoyControl;

for ii = 1:nDrones    
    if ii == 2
        A{ii}.pPar.LocalPortControl = 5558;
        A{ii}.pPar.LocalPortState = 5552;
    end
    % Conectando no Drone
    A{ii}.rConnect;
    
    % ArDrone Takeoff
    disp('Start Take Off Timming....');
    A{ii}.rTakeOff;
%     pause(10);
    disp('Taking Off End Time....');
end
%% POSIÇÕES DESEJADAS

Xd = [0 -1 1;
      0 1  1];

%% TEMPORIZADORES

T_MAX = 30;
T_AMOSTRAGEM = A{1}.pPar.Ts;
W = 2*pi/T_MAX;
rx = 1;
ry = 1;

T = tic;
Ta = tic;

while toc(T) < T_MAX
if toc(Ta) > T_AMOSTRAGEM 
%     Xd = [rx*sin(2*W*toc(T)) ry*sin(W*toc(T)) 1.5];
% %     
%     dXd = [2*W*rx*cos(2*W*toc(T)) W*ry*cos(W*toc(T)) 0];
    
    for ii = 1:nDrones
    A{ii}.pPos.Xd(1:3) = Xd(ii,:)';
%     A{ii}.pPos.Xd(7:9) = dXd(ii,:)';
    end
    %% Acquire sensors data        
    % Get optitrack data        
    rb = OPT.RigidBody;             % read optitrack
    
    % Pioneer
    for ii = 1:nDrones
    A{ii} = getOptData(rb(idA{ii}),A{ii});
%     disp(ii)
    A{ii}.pPos.X
    end
    
    for ii = 1:nDrones
    A{ii} = cInverseDynamicController_Compensador_ArDrone(A{ii});        % new controller (by timotiu 2020)
    end
    
    for ii = 1:nDrones
    A{ii} = J.mControl(A{ii});
    end
    
    for ii = 1:nDrones
    A{ii}.rSendControlSignals;
    end
    
    if J.pFlag == 1
       break 
    end
    
end
end

for jj = 1:3
for ii = 1:nDrones
disp(ii)
A{ii}.rLand;
end
end

%%
% P.pSC.Ud = [0; 0];
% 
% P.rCommand;




















