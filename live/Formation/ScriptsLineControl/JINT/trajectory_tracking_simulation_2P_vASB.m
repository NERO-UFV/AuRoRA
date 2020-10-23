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
P1 = Pioneer3DX(1);
P2 = Pioneer3DX(2);

P2.pPos.X(2) = 1;
P2.pPos.X(3) = 0.01;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Formation 3D
LF = LineFormationControl;

LF.pPar.K1 = diag([  0.1   0.1   1.0   0.75   0.5   0.5]);     % kinematic control gain  - controls amplitude
LF.pPar.K2 = diag([  0.5   0.5   0.5   0.5    0.25  0.25 ]);     % kinematic control gain - control saturation

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Robot/Simulator conection
% P.rConnect;

%% Configure simulation window
fig = figure(1);
axis([-2 2 -2 2 0 3]);
view(-21,30);
hold on;
grid on;

% Draw robots
try
    P1.mCADdel;
    P2.mCADdel;
catch
end

% Pioneer
P1.mCADplot(0.75,'k');
P2.mCADplot(0.75,'b');

drawnow;

%% Formation initial error

% Formation initial pose
LF.pPos.X  = [P1.pPos.X(1:3); P2.pPos.X(1:3)];
LF.pPos.Xr = LF.pPos.X;

% Formation initial pose
LF.mDirTrans;
LF.pPos.Qda = LF.pPos.Qd;

% pause(3);

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization
T_PLOT = 2;             % Período de plotagem em tempo real
LF.pPar.Ts = 0.1;
T_FORMATION = LF.pPar.Ts; % 200ms
T_R1 = P1.pPar.Ts;    % 100ms;
T_R2 = P2.pPar.Ts;    %1/30

rX = 1.0;   % [m]
rY = 1.0;   % [m]
T = 120;     % [s]
w = 2*pi/T; % [rad/s]

% Data variables
kk = 1;
data = zeros(round(T/T_FORMATION),69); % Data matrix

t  = tic;
t_plot = tic;

t_Formation = tic;      % Formation control
t_R1 = tic;        % pioneer cycle
t_R2 = tic;        % ardrone cycle
t_integ = tic;

ttt = [];
while toc(t)< T
    % =====================================================================
    if toc(t_R1) > T_R1
        t_R1 = tic;
        disp('L')
        
        P1.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        
        % Controlador Cinemático
        P1 = fControladorCinematico(P1);
  
        % Send control signals to robots
        P1.rSendControlSignals;
        
    end
    
    % =====================================================================
    if toc(t_R2) > T_R2
        t_R2 = tic;
        disp('S')
        
        P2.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        
        % Controlador Cinemático
        P2 = fControladorCinematico(P2);
  
        % Send control signals to robots
        P2.rSendControlSignals;
    end

    % =====================================================================
    
    % Laço de controle de formação
    if toc(t_Formation) > T_FORMATION
        t_Formation = tic;
        disp('F')
        
        LF.pPos.X = [P1.pPos.X(1:3); P2.pPos.X(1:3)];   % Posição dos membros da formação
        LF.mDirTrans;                                 % Transformada Direta X --> Q
               
        tt = toc(t); % Tempo atual
        tp = (3*(tt/T)^2 - 2*(tt/T)^3)*T;        
        
        LF.pPos.Qda = LF.pPos.Qd;
        
        LF.pPos.Qd(1) = rX*sin(w*tp);                   % xF
        LF.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
        LF.pPos.Qd(3) = 0.0;                            % zF
        LF.pPos.Qd(4) = 1.50;                           % rho
        LF.pPos.Qd(5) = pi/2;                              % alpha (frente/trás)
        LF.pPos.Qd(6) = 0.00;                           % beta  (lateral)
        
        LF.pPos.dQd = (LF.pPos.Qd-LF.pPos.Qda)/LF.pPar.Ts;
        
        LF.mInvTrans;                                   % Transformada Inversa Qd --> Xd       
        
        % Control       
        % LF.mFormationControl;
        LF.mFormationControlNSB('P'); 
        
        % Atribuindo posições desejadas
        % Pioneer
        P1.pPos.Xd(1:3) = LF.pPos.Xr(1:3);             % Posição desejada
        P1.pPos.Xd(7:9) = LF.pPos.dXr(1:3);            % Velocidade desejada

        % ArDrone
        P2.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
        P2.pPos.Xd(7:9) = LF.pPos.dXr(4:6);        

        % Variable to feed plotResults function
        data(kk,:) = [P1.pPos.Xd'     P1.pPos.X' ...
            P2.pPos.Xd'        P2.pPos.X'        ...
            LF.pPos.Qd'       LF.pPos.Q'      ...
            P1.pSC.Ud'        P1.pSC.U'    ...
            P2.pSC.Ud'        P2.pSC.U'         ...
            toc(t)];
        kk = kk + 1;         
        
    end
    %% Draw robots

    if toc(t_plot) > T_PLOT
        t_plot = tic;
        P1.mCADdel;
        P2.mCADdel;
        try
            delete(fig1);
            delete(fig2);
            delete(fig3);
            delete(fig4);
            delete(fig5);
        catch
        end
        
        P1.mCADplot(0.75,'k');
        P2.mCADplot(0.75,'b');
        
        % Percourse made
        fig1 = plot3(data(2:kk-1,1),data(2:kk-1,2),data(2:kk-1,3),'r-','LineWidth',1.0);
        fig2 = plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r--','LineWidth',0.5);
        fig3 = plot3(data(2:kk-1,25),data(2:kk-1,26),data(2:kk-1,27),'b-','LineWidth',1);
        fig4 = plot3(data(2:kk-1,37),data(2:kk-1,38),data(2:kk-1,39),'b--','LineWidth',0.5);
        fig5 = plot3(data(2:kk-1,49),data(2:kk-1,50),data(2:kk-1,51),'k--','LineWidth',0.5);
                
        drawnow;
    end
end

%% Send control signals
% R1.pSC.Ud = [0; 0];
% R1.rSendControlSignals;    % Pioneer

%% Plot results
% % % figure;
% plotResultsLineControl(data);
% x
figure
plot(data(1:kk-1,end),data(1:kk-1,[49 55]))
% y
figure
plot(data(1:kk-1,end),data(1:kk-1,[50 56]))
% z
figure
plot(data(1:kk-1,end),data(1:kk-1,[51 57]))
% Rho
figure
plot(data(1:kk-1,end),data(1:kk-1,[52 58]))
figure
% Alpha e Beta
plot(data(1:kk-1,end),data(1:kk-1,[53 54])*180/pi,data(1:kk-1,end),data(1:kk-1,[59 60])*180/pi)


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
