%% 3D Drone Control

% Initial Comands

clear; close all; clc;

try
    fclose(instrfindall);
catch
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %
% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

% Robot
A = ArDrone(40);
A.pPar.Ts = 1/30;

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

%% Robots initial pose
% detect rigid body ID from optitrack
idA = getID(OPT,A);            % drone ID on optitrack
rb = OPT.RigidBody;            % read optitrack data
A = getOptData(rb(idA),A);     % get ardrone data

A.rConnect;

% Send Comand do Take off
A.rTakeOff;

tp = tic;
tc = tic;
disp('Start Take Off Timming....');

while toc(tp) < 10
    if toc(tc) > 1/30
        A = J.mControl(A);
        A.rSendControlSignals;
    end
end
disp('Taking Off End Time....');

%% Variable initialization
data = [];

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization

T_CONTROL = 1/30;
T_ARDRONE = 1/30;

rX = 1.75;       % [m]
rY = 1.75;       % [m]
rho = 1.5;      % [m]
T = 60;         % [s]
Tf = 120;       % [s]
w = 2*pi/T;     % [rad/s]

T1 = 120.0;             % Lemniscata
% % T2 =  15.0 + T1;        % Aproximação + Emergency
% % T3 =  0.1 + T2;         % Andando com o Drone pousado
% % T4 =  5.0 + T3;         % Parando o Pioneer

t_control = tic;
t_integ_1 = tic;
t  = tic;

while toc(t) < T1
    
    if toc(t_control) > T_CONTROL    
        t_control = tic;

        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        A = getOptData(rb(idA),A);
        
        A.pPos.X(7:12) = (A.pPos.X(1:6)-A.pPos.Xa(1:6))/toc(t_integ_1);
        A.pSC.U = [1 ; -1 ; 1; -1].*[A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]./A.pPar.uSat; % populates actual control signal to save data
        t_integ_1 = tic;
        
        %% Trajectory
        
        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf;

        % Drone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        
        A.pPos.Xd(1:3) = [ rX*sin(w*tp); rY*sin(2*w*tp); rho ];
        A.pPos.Xd(7:9) = [ w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp); ...
            2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);...
            0];
        
        % % % % %         A.pPos.Xd(6) = P.pPos.X(6);
        % % % % %         A.pPos.Xd(12) = P.pPos.X(12);  % dPsi
        
        
        % The Gains must be given in the folowing order
        % Rolagem Arfagem e Guinada (cabeceo)
        % kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2

%         Agains =   [   0.3    3.00    0.3   3.00   10.00    3.00 ;  1   20  1   20   1   3.5]; % GANHOS QUENTE PELANDO
  
        Agains =   [   0.3    3.00    0.3   3.00   10.00    3.00 ;  1   20  1   20   1   3.0]; % GANHOS QUENTE PELANDO
        
        A = cUnderActuatedController(A,Agains);  % ArDrone
        A = J.mControl(A);                       % joystick command (priority)
        
        % Drone
        
        if toc(t) > T1 - 5
            
            A.rLand
            fprintf('\n\nParando o Drone\n');
            
        end
        
        %% Save data (.txt file)
        
        % Variable to feed plotResults function
        data = [  data  ; A.pPos.Xd'  A.pPos.X'    A.pSC.Ud'   A.pSC.U'   toc(t)];
        
        % %         %   1 -- 12      13 -- 24     25 -- 28          29 -- 32
        % %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:4)'    P.pSC.U(1:4)'
        
        %% Send control signals to robots
        A.rSendControlSignals;
        
    end
    
end

%% Land drone
if A.pFlag.Connected == 1
    A.rLand;                % Commando to Land Drone
end

%% Plot results

% Drone data
AXd   = data(:,(1:12))';
AX    = data(:,12+(1:12))';
AUd   = data(:,24+(1:4))';
AU    = data(:,28+(1:4))';
AXtil = AXd - AX;

sl= 1; %'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

% Posição dos robôs
% Parameters
scale  = 1;     % robot model plot scale
Pcolor = 'k';   % robot A color
step   = 200;   % model plot step
h      = 0.1;   % line height

figure;
axis equal
axis ([-2 2 -2 2 0 2])
set(gca,'Box','on')

hold on, grid on;

% Initial positions
ps1 = plot3(AXd(1,1),AXd(2,1),AXd(3,1),'r^','MarkerSize',5,'LineWidth',3);
ps2 = plot3(AX(1,1),AX(2,1),AX(3,1),'c^','MarkerSize',5,'LineWidth',3);


% % Percourse made
p1 = plot3(AXd(1,:),AXd(2,:),AXd(3,:),'r-','LineWidth',0.5);
p2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',0.5);

xlabel('$x$ [m]','interpreter','Latex');
ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','interpreter','Latex');

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
