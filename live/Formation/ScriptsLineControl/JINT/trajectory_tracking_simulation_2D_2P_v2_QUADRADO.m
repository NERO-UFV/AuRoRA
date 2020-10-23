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
P{2} = Pioneer3DX(2);

A{1} = ArDrone(1);
A{2} = ArDrone(2);

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Formation 3D
LF{1} = LineFormationControl;
LF{2} = LineFormationControl;
LF{3} = LineFormationControl;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Robot initial pose
% Pioneer P3DX
P{1}.rSetPose([0 0 0 pi/4]');
P{1}.pSC.Ud = [0; 0];
P{1}.rSendControlSignals;    % Pioneer

P{2}.rSetPose([1.5 1.5 0 pi/4]');
P{2}.pSC.Ud = [0; 0];
P{2}.rSendControlSignals;    % Pioneer

% ArDrone
A{1}.pPos.X(1:3) = [0 0 0.25];
A{1}.pPos.X(6) = pi/4;

A{2}.pPos.X(1:3) = [1.5 1.5 0.25];
A{2}.pPos.X(6) = pi/4;

% Atribuindo posições desejadas   

Xd = zeros(3,1);
Xda = zeros(3,1);
dXd = zeros(3,1);
XTs = LF{1}.pPar.Ts;

% P1
P{1}.pPos.Xd(1:3) = Xd;         % Posição desejada
P{1}.pPos.Xd(7:9) = dXd;        % Velocidade desejada
% A1
A{1}.pPos.Xda = A{1}.pPos.Xd;   % save previous posture
A{1}.pPos.Xd(1:3) = Xd + [ 0.0 ; 0.0 ; 1.5 ]; % Posição desejada
A{1}.pPos.Xd(7:9) = dXd;

% P2
P{2}.pPos.Xd(1:3) = Xd + [ 1.5 ; 1.5 ; 0.0 ]; % Posição desejada
P{2}.pPos.Xd(7:9) = dXd;        % Velocidade desejada
% A2
A{2}.pPos.Xda = A{2}.pPos.Xd;   % save previous posture
A{2}.pPos.Xd(1:3) = Xd + [ 1.5 ; 1.5 ; 1.5 ];
A{2}.pPos.Xd(7:9) = dXd;        % Velocidade desejada

%% Configure simulation window

fig = figure(1);
axis([-3 3 -3 3 0 3]);
view(-21,30);
hold on;
grid on;

P{1}.mCADdel;
P{2}.mCADdel;
try
    delete(fig1);
    delete(fig2);
    delete(fig3);
    delete(fig4);
    delete(fig5);
    delete(square);
catch
end

P{1}.mCADplot(0.75,'k');
P{2}.mCADplot(0.75,'g');
A{1}.mCADcolor([0 0 1]);
A{1}.mCADplot;
A{2}.mCADcolor([0 0 0]);
A{2}.mCADplot;
drawnow;

%% Simulation
pause(3);

fprintf('\nStart..............\n\n');

% Time variables initialization

T_PLOT = 3;       % Período de plotagem em tempo real

T_FORMATION = LF{1}.pPar.Ts; % 200ms
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
data = zeros(round(T4/T_FORMATION),175); % Data matrix

t  = tic;
t_plot = tic;

t_Formation = tic;      % Formation control

t_Pioneer_1 = tic;
t_Pioneer_2 = tic;

t_ArDrone_1 = tic;
t_ArDrone_2 = tic;

while toc(t)< T4
    % =====================================================================
    % Pioneer
    
    if toc(t_Pioneer_1) > T_PIONEER
        t_Pioneer_1 = tic;
        disp('P1');
        P{1}.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
        P{1} = fDynamicController(P{1});    % Controlador        
        P{1}.rSendControlSignals;           % Enviar sinal de controle para o robô
    end
    
    if toc(t_Pioneer_2) > T_PIONEER
        t_Pioneer_2 = tic;
        disp('P2');
        P{2}.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
        P{2} = fDynamicController(P{2});    % Pioneer Dynamic Compensator        
        P{2}.rSendControlSignals;           % Enviar sinal de controle para o robô
    end
    
    % =====================================================================
    % ArDrone
    
    if toc(t_ArDrone_1) > T_ARDRONE
        t_ArDrone_1 = tic;
        disp('D1');
        A{1}.rGetSensorData;                    % Adquirir dados dos sensores - ArDrone
        A{1} = cUnderActuatedController(A{1});  % Controlador 
        A{1}.rSendControlSignals;               % Enviar sinal de controle para o robô
    end
    
    if toc(t_ArDrone_2) > T_ARDRONE
        t_ArDrone_2 = tic;
        disp('D2');
        A{2}.rGetSensorData;                    % Adquirir dados dos sensores - ArDrone
        A{2} = cUnderActuatedController(A{2});  % Controlador
        A{2}.rSendControlSignals;               % Enviar sinal de controle para o robô
    end
        
    % =====================================================================
    % Laço de controle de formação
    if toc(t_Formation) > T_FORMATION
        t_Formation = tic;

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
        
        % P2
        P{2}.pPos.Xd(1:3) = Xd + [ 1.5 ; 1.5 ; 0.0 ]; % Posição desejada
        P{2}.pPos.Xd(7:9) = dXd;        % Velocidade desejada
        % A2
        A{2}.pPos.Xda = A{2}.pPos.Xd;   % save previous posture
        A{2}.pPos.Xd(1:3) = Xd + [ 1.5 ; 1.5 ; 1.5 ];
        A{2}.pPos.Xd(7:9) = dXd;        % Velocidade desejada
               
        % Variable to feed plotResults function    
        data(kk,:) = [  P{1}.pPos.Xd'     P{1}.pPos.X'        P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)'...
                        P{2}.pPos.Xd'     P{2}.pPos.X'        P{2}.pSC.Ud(1:2)'    P{2}.pSC.U(1:2)'...
                        A{1}.pPos.Xd'     A{1}.pPos.X'        A{1}.pSC.Ud'         A{1}.pSC.U' ...
                        A{2}.pPos.Xd'     A{2}.pPos.X'        A{2}.pSC.Ud'         A{2}.pSC.U' ...
                        LF{1}.pPos.Qd'    LF{1}.pPos.Qtil'    LF{1}.pPos.Xr'...
                        LF{2}.pPos.Qd'    LF{2}.pPos.Qtil'    LF{2}.pPos.Xr'...
                        LF{3}.pPos.Qd'    LF{3}.pPos.Qtil'    LF{3}.pPos.Xr'...
                        toc(t)];
                    
                        kk = kk + 1;
        
        % %         %   1 -- 12             13 -- 24             25 -- 26              27 -- 28
        % %             P{1}.pPos.Xd'       P{1}.pPos.X'         P{1}.pSC.Ud(1:2)'     P{1}.pSC.U(1:2)'
        % %         
        % %         %   29 -- 40            41 -- 52             53 -- 54              55 -- 56 
        % %             P{2}.pPos.Xd'       P{2}.pPos.X'         P{2}.pSC.Ud(1:2)'     P{2}.pSC.U(1:2)'
        % %         
        % %         %   57 -- 68            69 -- 80             81 -- 84              85 -- 88
        % %             A{1}.pPos.Xd'       A{1}.pPos.X'         A{1}.pSC.Ud'          A{1}.pSC.U'
        % %
        % %         %   89 -- 100           101 -- 112           113 -- 116            117 -- 120
        % %             A{2}.pPos.Xd'       A{2}.pPos.X'         A{2}.pSC.Ud'          A{2}.pSC.U'
        % %        
        % %         %   121 -- 126          127 -- 132           133 -- 138
        % %             LF{1}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'
        % %         
        % %         %   139 -- 144          145 -- 150           151 -- 156
        % %             LF{1}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'
        % %        
        % %         %   157 -- 162          163 -- 168           169 -- 174
        % %             LF{1}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'
        % %
        % %         %   175 
        % %         %   toc(t)  ]
        
    end
    
    %% Draw robots
    if toc(t_plot) > T_PLOT
        t_plot = tic;
        % Pioneer
        P{1}.mCADdel;
        P{2}.mCADdel;
   
        try
            delete(fig1);
            delete(fig2);
            delete(fig3);
            delete(fig4);
            delete(fig5);
            delete(square);
        catch
        end
        
        P{1}.mCADplot(0.75,'k');
        P{2}.mCADplot(0.75,'g');
        A{1}.mCADplot;
        A{2}.mCADplot([0;0;0]);
      
        % Percourse made
        fig1 = plot3(data(2:kk-1,1),data(2:kk-1,2),data(2:kk-1,3),'k--','LineWidth',1.0);
        fig2 = plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
        fig3 = plot3(data(2:kk-1,41),data(2:kk-1,42),data(2:kk-1,43),'g-','LineWidth',0.5);
        fig4 = plot3(data(2:kk-1,69),data(2:kk-1,70),data(2:kk-1,71),'b-','LineWidth',0.5);
        fig5 = plot3(data(2:kk-1,101),data(2:kk-1,102),data(2:kk-1,103),'k-','LineWidth',0.5);
        square = plot3( data(kk-2,[13 41 101 69 13]),...
                        data(kk-2,[14 42 102 70 14]),...
                        data(kk-2,[15 43 103 71 15]), 'Color',[1 0.5 0],'LineWidth',3);
        drawnow;
        
    end 
end

axis equal;
set(gca,'Box','on');
