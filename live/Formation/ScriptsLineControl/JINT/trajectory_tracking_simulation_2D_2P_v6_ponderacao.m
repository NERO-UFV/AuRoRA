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
LF{1} = LineFormationControl;   % P1 + D1 
LF{2} = LineFormationControl;   % P1 + P2 
LF{3} = LineFormationControl;   % P2 + D2
LF{1}.pPar.Ts = 0.200;

LF{2}.pPar.K1 = diag([  1.0   1.0   0.0   1.0    2.0   2.0  ]);     % kinematic control gain  - controls amplitude
LF{2}.pPar.K2 = diag([  0.5   0.5   0.0   0.5    0.3   0.3    ]);     % kinematic control gain - control saturation

for ii = [1 3]
    LF{ii}.pPar.K1 = diag([  1.0   1.0   0.0   1.0    2.0   2.0  ]);     % kinematic control gain  - controls amplitude
    LF{ii}.pPar.K2 = diag([  0.5   0.5   0.0   0.5    0.15   0.15  ]);     % kinematic control gain - control saturation
end

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

%% Robots initial pose
% Pioneer P3DX
P{1}.rSetPose([ 0.0 0.0 0.0 pi/4]');
P{1}.pSC.Ud = [0 ; 0];
P{1}.rSendControlSignals;    % Pioneer

P{2}.rSetPose([ 1.0 0.0 0.000001 pi/4]');
P{2}.pSC.Ud = [0 ; 0];
P{2}.rSendControlSignals;    % Pioneer

% ArDrone
A{1}.pPos.X(1:3) = [ 0.0-0.1 0.0 0.25 ];
A{1}.pPos.X(6) = pi/4;

A{2}.pPos.X(1:3) = [ 1.0-0.1 0.0 0.25 ];
A{2}.pPos.X(6) = pi/4;

% Atribuindo posições desejadas   
Xd = zeros(6,1);
Xda = zeros(6,1);
dXd = zeros(6,1);
XTs = LF{1}.pPar.Ts;

% P1
P{1}.pPos.Xd(1:3) = Xd(1:3);         % Posição desejada
P{1}.pPos.Xd(7:9) = dXd(1:3);        % Velocidade desejada
% A1
A{1}.pPos.Xda = A{1}.pPos.Xd;   % save previous posture
A{1}.pPos.Xd(1:3) = Xd(1:3) + [ 0.0 ; 0.0 ; 1.0 ]; % Posição desejada
A{1}.pPos.Xd(7:9) = dXd(1:3);

% P2
P{2}.pPos.Xd(1:3) = Xd(1:3) + [ 1.0 ; 0.0 ; 0.000001 ]; % Posição desejada
P{2}.pPos.Xd(7:9) = dXd(1:3);        % Velocidade desejada
% A2
A{2}.pPos.Xda = A{2}.pPos.Xd;   % save previous posture
A{2}.pPos.Xd(1:3) = Xd(1:3) + [ 1.0 ; 0.0 ; 1.0 ];
A{2}.pPos.Xd(7:9) = dXd(1:3);        % Velocidade desejada

%% Formation initial error
% Formation initial pose
LF{1}.pPos.X = [P{1}.pPos.X(1:3); A{1}.pPos.X(1:3)];
LF{1}.pPos.Xr = LF{1}.pPos.X;
LF{1}.mDirTrans;
LF{1}.pPos.Qda = LF{1}.pPos.Qd;

LF{2}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];
LF{2}.pPos.Xr = LF{2}.pPos.X;
LF{2}.mDirTrans;
LF{2}.pPos.Qda = LF{2}.pPos.Qd;

LF{3}.pPos.X = [P{2}.pPos.X(1:3); A{2}.pPos.X(1:3)];
LF{3}.pPos.Xr = LF{3}.pPos.X;
LF{3}.mDirTrans;
LF{3}.pPos.Qda = LF{3}.pPos.Qd;

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

T_PLOT = inf;       % Período de plotagem em tempo real

T_FORMATION = LF{1}.pPar.Ts; % 200ms
T_PIONEER = P{1}.pPar.Ts; %0.100;
T_ARDRONE = A{1}.pPar.Ts; %1/30

rX = 1.0; % [m]
rY = 1.0; % [m]
T = 60;   % [s]
Tf = 120; % [s]
w = 2*pi/T; % [rad/s]

T1 = 85.0;             % Lemniscata
T2 =  15.0 + T1;        % Aproximação + Emergency
T3 =  15.1 + T2;        % Andando com o Drone pousado
T4 =  5.0 + T3;         % Parando o Pioneer

caso = 1;

% Data variables
kk = 1;
data = zeros(round(T4/T_FORMATION),193); % Data matrix

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
%         disp('P1');
        P{1}.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
        P{1} = fDynamicController(P{1});    % Controlador        
        P{1}.rSendControlSignals;           % Enviar sinal de controle para o robô
    end
    
    if toc(t_Pioneer_2) > T_PIONEER
        t_Pioneer_2 = tic;
%         disp('P2');
        P{2}.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
        P{2} = fDynamicController(P{2});    % Pioneer Dynamic Compensator        
        P{2}.rSendControlSignals;           % Enviar sinal de controle para o robô
    end
    
    % =====================================================================
    % ArDrone
    
    if toc(t_ArDrone_1) > T_ARDRONE
        t_ArDrone_1 = tic;
%         disp('D1');
        A{1}.rGetSensorData;                    % Adquirir dados dos sensores - ArDrone
        
        Agains =   [   0.1    2.00    0.1   2.00   5.00    2.00 ;  1   20   1   15   1   2.5]; % GANHOS QUENTE PELANDO
        A{1} = cUnderActuatedController(A{1},Agains);  % Controlador 
        A{1}.rSendControlSignals;               % Enviar sinal de controle para o robô
    end
    
    if toc(t_ArDrone_2) > T_ARDRONE
        t_ArDrone_2 = tic;
%         disp('D2');
        A{2}.rGetSensorData;                    % Adquirir dados dos sensores - ArDrone
        
        Agains =   [   0.1    2.00    0.1   2.00   5.00    2.00 ;  1   20   1   15   1   2.5]; % GANHOS QUENTE PELANDO
        A{2} = cUnderActuatedController(A{2},Agains);  % Controlador
        A{2}.rSendControlSignals;               % Enviar sinal de controle para o robô
    end
        
    % =====================================================================
    % Laço de controle de formação
    if toc(t_Formation) > T_FORMATION
        t_Formation = tic;

        % Trajectory Planner
        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf;
        Xda = Xd;
        Xd(1) = rX*sin(w*tp);                   % xF
        Xd(2) = rY*sin(2*w*tp);                 % yF
        Xd(3) = 0.00;                           % zF
        Xd(4) = 0.00;                           % rho
        Xd(5) = 0.00;                           % alpha (frente/trás)
        Xd(6) = 0.00;                           % beta  (lateral)
        dXd = (Xd - Xda)/XTs;
        
        % Formation instant variables and references
        LF{1}.pPos.X = [P{1}.pPos.X(1:3); A{1}.pPos.X(1:3)];    % P1 + A1 | POSIÇÃO
        LF{2}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];    % P1 + P2 | POSIÇÃO
        LF{3}.pPos.X = [P{2}.pPos.X(1:3); A{2}.pPos.X(1:3)];    % P2 + A2 | POSIÇÃO

        for ii = 1:3
            LF{ii}.pPos.Qda = LF{ii}.pPos.Qd;
        end
        
        LF{1}.pPos.Qd = Xd + [ 0.0 ; 0.0 ; 0.0 ;   1.0   ;  0.0  ; 0.0  ];        
        LF{2}.pPos.Qd = Xd + [ 0.0 ; 0.0 ; 0.0 ;   1.0   ;  pi/2 ; 0.0  ];
        LF{3}.pPos.Qd = Xd + [ 1.0 ; 0.0 ; 0.0 ;   1.0  ;  0.0  ; 0.0  ];

        
        for ii = 1:3
            LF{ii}.pPos.dQd = (LF{ii}.pPos.Qd - LF{ii}.pPos.Qda)/LF{ii}.pPar.Ts;
            LF{ii}.mFormationControl;
%             LF{ii}.mFormationControlNSB('F');
            LF{ii}.mInvTrans;
        end
   
        % Atribuindo posições desejadas       
        % P1
        P{1}.pPos.Xd(1:3) = (LF{1}.pPos.Xr(1:3) + LF{2}.pPos.Xr(1:3))/2;         % Posição desejada
        P{1}.pPos.Xd(7:9) = (LF{1}.pPos.dXr(1:3) + LF{2}.pPos.dXr(1:3))/2;        % Velocidade desejada
        
        % A1
        A{1}.pPos.Xda = A{1}.pPos.Xd;   % save previous posture
        A{1}.pPos.Xd(1:3) = LF{1}.pPos.Xr(4:6); % Posição desejada
        A{1}.pPos.Xd(7:9) = LF{1}.pPos.dXr(4:6);
        
        % P2
        P{2}.pPos.Xd(1:3) = (LF{2}.pPos.Xr(4:6) + LF{3}.pPos.Xr(1:3))/2; % Posição desejada
        P{2}.pPos.Xd(7:9) = (LF{2}.pPos.dXr(4:6) + LF{3}.pPos.dXr(1:3))/2;        % Velocidade desejada
        
        % A2
        A{2}.pPos.Xda = A{2}.pPos.Xd;   % save previous posture
        A{2}.pPos.Xd(1:3) = LF{3}.pPos.Xr(4:6);
        A{2}.pPos.Xd(7:9) = LF{3}.pPos.dXr(4:6);        % Velocidade desejada
                       
        % Variable to feed plotResults function    
        data(kk,:) = [  P{1}.pPos.Xd'     P{1}.pPos.X'        P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)'...
                        P{2}.pPos.Xd'     P{2}.pPos.X'        P{2}.pSC.Ud(1:2)'    P{2}.pSC.U(1:2)'...
                        A{1}.pPos.Xd'     A{1}.pPos.X'        A{1}.pSC.Ud'         A{1}.pSC.U' ...
                        A{2}.pPos.Xd'     A{2}.pPos.X'        A{2}.pSC.Ud'         A{2}.pSC.U' ...
                        LF{1}.pPos.Qd'    LF{1}.pPos.Qtil'    LF{1}.pPos.Xr'       LF{1}.pPos.Xd'...
                        LF{2}.pPos.Qd'    LF{2}.pPos.Qtil'    LF{2}.pPos.Xr'       LF{2}.pPos.Xd'...
                        LF{3}.pPos.Qd'    LF{3}.pPos.Qtil'    LF{3}.pPos.Xr'       LF{3}.pPos.Xd'...
                        toc(t)];
                    
                        kk = kk + 1;
        try
        catch
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
        % %         %   121 -- 126          127 -- 132           133 -- 138            139 -- 144
        % %             LF{1}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
        % %         
        % %         %   145 -- 150          151 -- 156           157 -- 162            163 -- 168 
        % %             LF{2}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
        % %        
        % %         %   169 -- 174          175 -- 180           181 -- 186            187 -- 192 
        % %             LF{3}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
        % %
        % %         %   193
        % %         %   toc(t)  ]
        end
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
            delete(squared);
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
                        data(kk-2,[15 43 103 71 15]), 'Color',[1 0.5 0],'LineWidth',1);
                    
        squared = plot3( data(kk-2,[139 166 190 142 139]),...
                        data(kk-2,[140 167 191 143 140]),...
                        data(kk-2,[141 168 192 144 141]), 'Color',[1 0.5 0],'LineWidth',1,'LineStyle',':');                    
        drawnow;
        
    end 
end

axis equal;
set(gca,'Box','on');
