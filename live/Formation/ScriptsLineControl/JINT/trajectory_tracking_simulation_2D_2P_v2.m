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

P{2}.rSetPose([-1 -1 0 pi/4]');
P{2}.pSC.Ud = [0; 0];
P{2}.rSendControlSignals;    % Pioneer

% ArDrone
A{1}.pPos.X(1:3) = [-0.075 -0.075 0.25];
A{1}.pPos.X(6) = pi/4;

A{2}.pPos.X(1:3) = [-1.075 -1.075 0.25];
A{2}.pPos.X(6) = pi/4;

%% Configure simulation window

fig = figure(1);
axis([-3 3 -3 3 0 3]);
view(-21,30);
hold on;
grid on;

for i = 1:2
% Draw robots
try
    A{i}.mCADdel;
    P{i}.mCADdel;
catch
end

% Pioneer
P{i}.mCADplot(0.75,'k');

% ArDrone
A{i}.mCADcolor([0 0 1]);
A{i}.mCADplot;
end
drawnow;

%% Formation initial error

% Formation initial pose
LF{1}.pPos.X = [P{1}.pPos.X(1:3); A{1}.pPos.X(1:3)];
LF{1}.pPos.Xr = LF{1}.pPos.X;
LF{1}.mDirTrans;
LF{1}.pPos.Qda = LF{1}.pPos.Qd;

LF{2}.pPos.X = [P{2}.pPos.X(1:3); A{2}.pPos.X(1:3)];
LF{2}.pPos.Xr = LF{2}.pPos.X;
LF{2}.mDirTrans;
LF{2}.pPos.Qda = LF{2}.pPos.Qd;

LF{3}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];
LF{3}.pPos.Xr = LF{3}.pPos.X;
LF{3}.mDirTrans;
LF{3}.pPos.Qda = LF{3}.pPos.Qd;

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization

T_PLOT = 5;       % Período de plotagem em tempo real

T_FORMATION = LF.pPar.Ts; % 200ms
T_PIONEER = P.pPar.Ts; %0.100;
T_ARDRONE = A.pPar.Ts; %1/30

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
t_Pioneer_2 = tic;

t_ArDrone_1 = tic;
t_ArDrone_2 = tic;

t_integ = tic;

while toc(t)< T4
    % =====================================================================
    % Pioneer
    
    if toc(t_Pioneer_1) > T_PIONEER
        disp('P1');
        t_Pioneer_1 = tic;
        P{1}.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
        P{1} = fDynamicController(P{1});    % Pioneer Dynamic Compensator        
        P{1}.rSendControlSignals;           % Enviar sinal de controle para o robô
    end
    
    if toc(t_Pioneer_2) > T_PIONEER     
        t_Pioneer_2 = tic;
        P{2}.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
        P{2} = fDynamicController(P{2});    % Pioneer Dynamic Compensator        
        P{2}.rSendControlSignals;           % Enviar sinal de controle para o robô
    end
    
    % =====================================================================
    % ArDrone
    
    if toc(t_ArDrone_1) > T_ARDRONE
        t_ArDrone_1 = tic;
        A{1}.rGetSensorData;    % Adquirir dados dos sensores - ArDrone
        A{1}.pPos.Xda = A{1}.pPos.Xd;    % save previous posture
        A{1} = cUnderActuatedController(A{1});  % ArDrone
        A{1}.rSendControlSignals;
    end
    
    if toc(t_ArDrone_2) > T_ARDRONE
        t_ArDrone_2 = tic;
        A{2}.rGetSensorData;    % Adquirir dados dos sensores - ArDrone
        A{2}.pPos.Xda = A{2}.pPos.Xd;    % save previous posture
        A{2} = cUnderActuatedController(A{2});  % ArDrone
        A{2}.rSendControlSignals;
    end
        
    % =====================================================================
    % Laço de controle de formação
    if toc(t_Formation) > T_FORMATION
        t_Formation = tic;
        for i = 1:2
            P{i}.rGetSensorData;
            A{i}.rGetSensorData;
        end
        
        LF{1}.pPos.X = [P{1}.pPos.X(1:3); A{1}.pPos.X(1:3)];
        LF{1}.mDirTrans;

        LF{2}.pPos.X = [P{2}.pPos.X(1:3); A{2}.pPos.X(1:3)];
        LF{2}.mDirTrans;

        LF{3}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];
        LF{3}.mDirTrans;
        
        %% Trajectory
        
        if toc(t) < T1
            
            if caso == 1
                fprintf('\n\nCaso 1: Leminiscata\n');
                caso = 2;
            end
            
            t_traj = toc(t);
            a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
            tp = a*Tf;

            LF.pPos.Qda = LF.pPos.Qd;
            
            LF.pPos.Qd(1) = rX*sin(w*tp);                   % xF
            LF.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
            LF.pPos.Qd(3) = 0.00;                           % zF
            LF.pPos.Qd(4) = 1.50;                           % rho
            LF.pPos.Qd(5) = 0.00;                           % alpha (frente/trás)
            LF.pPos.Qd(6) = 0.00;                           % beta  (lateral)
            
            LF.pPos.dQd = (LF.pPos.Qd-LF.pPos.Qda)/LF.pPar.Ts;
            
        elseif toc(t) > T1 && toc(t) < T2
            
            if caso == 2
                fprintf('\n\nCaso 2: Aproximação + Emergency\n');
                caso = 3;
            end
            
            t_traj = toc(t);
            a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
            tp = a*Tf;
            
            LF.pPos.Qda = LF.pPos.Qd;
            
            LF.pPos.Qd(1) = rX*sin(w*tp);                   % xF
            LF.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
            LF.pPos.Qd(3) = 0.00;                           % zF
            LF.pPos.Qd(4) = 0.50;                           % rho
            LF.pPos.Qd(5) = 0.00;                           % alpha (frente/trás)
            LF.pPos.Qd(6) = 0.00;                           % beta  (lateral)
                        
            LF.pPos.dQd = (LF.pPos.Qd-LF.pPos.Qda)/LF.pPar.Ts; 
            
        elseif toc(t) > T2 && toc(t) < T3
            
            if caso == 3
                fprintf('\n\nCaso 3: Andando com o Drone pousado\n');
                caso = 4;

            end
            
            t_traj = toc(t) - Tf;
            a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
            tp = a*Tf;
            
            LF.pPos.Qda = LF.pPos.Qd;
            
            LF.pPos.Qd(1) = rX*sin(w*tp);                   % xF
            LF.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
            LF.pPos.Qd(3) = 0.00;                           % zF
            LF.pPos.Qd(4) = 0.50;                           % rho
            LF.pPos.Qd(5) = 0.00;                           % alpha (frente/trás)
            LF.pPos.Qd(6) = 0.00;                           % beta  (lateral)
                        
            LF.pPos.dQd = (LF.pPos.Qd-LF.pPos.Qda)/LF.pPar.Ts; 
            
        end
                
        %% Control
        
        % Formation Control
        for i = 1:2
            LF{i}.pPar.K1 = diag([   20.0   20.0   0    2.0   2.0   2.0   ]);     % kinematic control gain  - controls amplitude
            LF{i}.pPar.K2 = diag([    .1    .1     0    .05    .05    .05   ]);     % kinematic control gain - control saturation

            LF{i}.mFormationControl;                           
    %         LF.mFormationControl_NullSpace('P');   
        end
        
        % Atribuindo posições desejadas       
        % Pioneer {1}
        P{1}.pPos.Xd(1:3) = LF{1}.pPos.Xr(1:3);             % Posição desejada
        P{1}.pPos.Xd(7:9) = LF{1}.pPos.dXr(1:3);            % Velocidade desejada
        % ArDrone {1}
        A{1}.pPos.Xda = A{1}.pPos.Xd;    % save previous posture
        A{1}.pPos.Xd(1:3) = LF{1}.pPos.Xr(4:6);
        A{1}.pPos.Xd(7:9) = LF{1}.pPos.dXr(4:6);
        
        % Pioneer {2}
        P{2}.pPos.Xd(1:3) = LF{3}.pPos.Xr(4:6);             % Posição desejada
        P{2}.pPos.Xd(7:9) = LF{3}.pPos.dXr(4:6);            % Velocidade desejada
        % ArDrone {3}
        A{2}.pPos.Xda = A{2}.pPos.Xd;    % save previous posture
        A{2}.pPos.Xd(1:3) = LF{2}.pPos.Xr(4:6);
        A{2}.pPos.Xd(7:9) = LF{2}.pPos.dXr(4:6);

        
        if toc(t) > T3
            
            if caso == 4
                fprintf('\n\nCaso 4: Parando Pioneer\n');
                caso = 5;
            end
            
            for i = 1:3
                LF{i}.pPos.Qd = LF{i}.pPos.Q;
                LF{i}.pPos.dQd = zeros(6,1);
            end
            
        end
               
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
        try
            delete(fig1);
            delete(fig2);
            delete(fig3);
            delete(fig4);
            delete(fig5);
            delete(pl);
            delete(rho_line);
        catch
        end
        
        % Pioneer
        P.mCADdel
        P.mCADplot(.75,'k');
        
        % ArDrone
        A.mCADplot;
        
        % Percourse made
        fig1 = plot3(data(1:kk,13),data(1:kk,14),data(1:kk,15),'r','LineWidth',1.0);
        fig2 = plot3(data(1:kk,41),data(1:kk,42),data(1:kk,43),'b','LineWidth',1.0);
        fig3 = plot3(data(1:kk,61),data(1:kk,62),data(1:kk,63),'k--','LineWidth',0.5);
        fig4 = plot3(data(1:kk,76),data(1:kk,77),data(1:kk,78),'c--','LineWidth',0.5);
        
        % Plotar linha rhof
        
        xl = [LF.pPos.X(1)   LF.pPos.X(4)];
        yl = [LF.pPos.X(2)   LF.pPos.X(5)];
        zl = [LF.pPos.X(3)   LF.pPos.X(6)];
        
        rho_line = line(xl,yl,zl);
        rho_line.Color = 'g';
        rho_line.LineStyle = '-';
        rho_line.LineWidth = 1.5;
        
        fig5 = plot3(LF.pPos.Xr(4),LF.pPos.Xr(5),LF.pPos.Xr(6),'b.','MarkerSize',20,'LineWidth',1.0);
        
        % Plotar linha rhof das posições desejadas
        xl = [LF.pPos.Xr(1)   LF.pPos.Xr(4)];
        yl = [LF.pPos.Xr(2)   LF.pPos.Xr(5)];
        zl = [LF.pPos.Xr(3)   LF.pPos.Xr(6)];
        
        pl = line(xl,yl,zl);
        pl.Color = 'k';
        pl.LineStyle = '--';
        pl.LineWidth = 1.5;
        
        drawnow;
    end
    
end

%% Send control signals
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

%% Plot results
