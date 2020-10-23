%% 3D Line Formation Pioneer-Drone

% Pioneer is the reference of the formation
% The formation variables are:
% Q = [xf yf zf rhof alfaf betaf]

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  %

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

% Robots
P{1} = Pioneer3DX(1);
A{1} = ArDrone(1);

% Formation 3D
ALF{1} = Angular_LineFormationControl;

% Robot initial pose
% Pioneer P3DX
P{1}.rSetPose([0 0 0 pi/4]');
P{1}.pSC.Ud = [0; 0];
P{1}.rSendControlSignals;    % Pioneer

% ArDrone
A{1}.pPos.X(1:3) = [0 0 0.25];
A{1}.pPos.X(6) = pi/4;

% Atribuindo posições desejadas   

Xd = zeros(3,1);
Xda = zeros(3,1);
dXd = zeros(3,1);
XTs = ALF{1}.pPar.Ts;

% P1
P{1}.pPos.Xd(1:3) = Xd;         % Posição desejada
P{1}.pPos.Xd(7:9) = dXd;        % Velocidade desejada
% A1
A{1}.pPos.Xda = A{1}.pPos.Xd;   % save previous posture
A{1}.pPos.Xd(1:3) = Xd + [ 0.0 ; 0.0 ; 1.5 ]; % Posição desejada
A{1}.pPos.Xd(7:9) = dXd;

%% Configure simulation window

fig = figure(1);
axis([-3 3 -3 3 0 3]);
view(-21,30);
hold on;
grid on;

P{1}.mCADdel;
try
    delete(fig1);
    delete(fig2);
    delete(fig3);
catch
end

P{1}.mCADplot(0.75,'k');
A{1}.mCADcolor([0 0 1]);
A{1}.mCADplot;
drawnow;

%% Simulation
pause(3);

fprintf('\nStart..............\n\n');

% Time variables initialization

T_PLOT = 3;       % Período de plotagem em tempo real

T_FORMATION = ALF{1}.pPar.Ts; % 200ms
T_PIONEER = P{1}.pPar.Ts; %0.100;
T_ARDRONE = A{1}.pPar.Ts; %1/30

rX = 1.0; % [m]
rY = 1.0; % [m]
rho = 1.5;
T = 75;   % [s]
Tf = 150;
w = 2*pi/T; % [rad/s]

T1 = 115.0;             % Lemniscata
T2 =  15.0 + T1;        % Aproximação + Emergency
T3 =  15.1 + T2;        % Andando com o Drone pousado
T4 =  5.0 + T3;         % Parando o Pioneer

caso = 1;

% Data variables
kk = 1;
data = zeros(round(T4/T_FORMATION),79); % Data matrix

t  = tic;
t_plot = tic;

t_Formation = tic;      % Formation control

t_Pioneer_1 = tic;

t_ArDrone_1 = tic;

while toc(t)< T4

    % Pioneer
    
    if toc(t_Pioneer_1) > T_PIONEER
        t_Pioneer_1 = tic;
%         disp('P1');
        P{1}.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
%         P{1} = fDynamicController(P{1});    % Controlador        
        P{1} = fKinematicControllerExtended(P{1});    % Controlador        
        P{1}.rSendControlSignals;           % Enviar sinal de controle para o robô
    end
    
    % ArDrone
    
    if toc(t_ArDrone_1) > T_ARDRONE
        t_ArDrone_1 = tic;
%         disp('D1');
        A{1}.rGetSensorData;                    % Adquirir dados dos sensores - ArDrone
        A{1} = cUnderActuatedController(A{1});  % Controlador 
        A{1}.rSendControlSignals;               % Enviar sinal de controle para o robô
    end

    % =====================================================================
    % Laço de controle de formação
    if toc(t_Formation) > T_FORMATION
        t_Formation = tic;

%         P{1}.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
%         A{1}.rGetSensorData;    % Adquirir dados dos sensores - ArDrone   
% 
%         ALF.pPos.X = [P{1}.pPos.X(1:3); A{1}.pPos.X(1:3)];   % Posição dos membros da formação
%         ALF.mAngularDirTrans;                                % Transformada Direta X --> Q
        
        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf;

        Xda = Xd;
        Xd(1) = rX*sin(w*tp);                   % xF
        Xd(2) = rY*sin(2*w*tp);                 % yF
        Xd(3) = 0.00;                           % zF
        dXd = (Xd - Xda)/XTs;
            
        % Atribuindo posições desejadas       
        % P1
        P{1}.pPos.Xd(1:3) = Xd;         % Posição desejada
        P{1}.pPos.Xd(7:9) = dXd;        % Velocidade desejada
        % A1
        A{1}.pPos.Xda = A{1}.pPos.Xd;   % save previous posture
        A{1}.pPos.Xd(1:3) = Xd + [ 0.0 ; 0.0 ; 1.5 ]; % Posição desejada
        A{1}.pPos.Xd(7:9) = dXd;
        
              
        % Variable to feed plotResults function    
        data(kk,:) = [  P{1}.pPos.Xd'     P{1}.pPos.X'        P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)'...
                        A{1}.pPos.Xd'     A{1}.pPos.X'        A{1}.pSC.Ud'         A{1}.pSC.U' ...
                        ALF{1}.pPos.Qd'    ALF{1}.pPos.Qtil'    ALF{1}.pPos.Xr'...
                        toc(t)];
                    
                        kk = kk + 1;
        
        % %         %   1 -- 12             13 -- 24             25 -- 26             27 -- 28
        % %             P{1}.pPos.Xd'       P{1}.pPos.X'         P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)'
        % %         
        % %         %   29 -- 40            41 -- 52             53 -- 56             57 -- 60
        % %             A{1}.pPos.Xd'       A{1}.pPos.X'         A{1}.pSC.Ud'         A{1}.pSC.U'
        % %        
        % %         %   62 -- 67            68 -- 73             74 -- 79
        % %             LF{1}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'
        % %
        % %         %   80 
        % %         %   toc(t)  ]
        
    end
    
    %% Draw robots
    if toc(t_plot) > T_PLOT
        t_plot = tic;
        % Pioneer
        P{1}.mCADdel;
   
        try
            delete(fig1);
            delete(fig2);
            delete(fig3);
        catch
        end
        
        P{1}.mCADplot(0.75,'k');
        A{1}.mCADplot;
      
        % Percourse made
        fig1 = plot3(data(2:kk-1,1),data(2:kk-1,2),data(2:kk-1,3),'k--','LineWidth',1.0);
        fig2 = plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
        fig3 = plot3(data(2:kk-1,29),data(2:kk-1,30),data(2:kk-1,31),'b-','LineWidth',0.5);
        drawnow;
        
    end 
end

axis equal;
set(gca,'Box','on');
