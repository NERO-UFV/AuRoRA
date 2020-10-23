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

P = Pioneer3DX(1);
A = ArDrone(30);

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Formation 3D
LF = LineFormationControl;

LF.pPar.K1 = diag([  10.0   10.0   10.0   1.0   5.0   5.0]);     % kinematic control gain  - controls amplitude
LF.pPar.K2 = diag([  0.1   0.1   0.1   0.1   0.1   0.1 ]);     % kinematic control gain - control saturation

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
    A.mCADdel;
    P.mCADdel;
catch
end

% Pioneer
P.mCADplot(0.75,'k');

% ArDrone
A.mCADcolor([0 0 1]);
A.mCADplot;

drawnow;

%% Formation initial error

% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
LF.pPos.Xr = LF.pPos.X;

% Formation initial pose
LF.mDirTrans;
LF.pPos.Qda = LF.pPos.Qd;

% pause(3);

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization
T_PLOT = 2;             % Período de plotagem em tempo real
T_FORMATION = LF.pPar.Ts; % 200ms
T_PIONEER = P.pPar.Ts;    % 100ms;
T_ARDRONE = A.pPar.Ts;    %1/30

rX = 1.0;   % [m]
rY = 1.0;   % [m]
T = 90;     % [s]
w = 2*pi/T; % [rad/s]

% Data variables
kk = 1;
data = zeros(round(T/T_FORMATION),73); % Data matrix

t  = tic;
t_plot = tic;

t_Formation = tic;      % Formation control
t_Pioneer = tic;        % pioneer cycle
t_ArDrone = tic;        % ardrone cycle
t_integ = tic;

ttt = [];
while toc(t)< T
    % =====================================================================
    if toc(t_Pioneer) > T_PIONEER
        t_Pioneer = tic;
%         disp('PPP')
        
        P.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        
        % Controlador Cinemático
        P = fControladorCinematico(P);
        
        % Controlador Dinâmico        
        % % Ganhos pré-definidos
        % cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];
        %
        % P = fDynamicController(P,cgains);     % Pioneer Dynamic Compensator
        
        % Send control signals to robots
        P.rSendControlSignals;
        
        
    end
    
    % =====================================================================
    if toc(t_ArDrone) > T_ARDRONE
        t_ArDrone = tic;
%         disp('DDDDDD')
        
        A.rGetSensorData;    % Adquirir dados dos sensores - ArDrone
        
        % Drone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture        
        Agains =   [   0.1    2.00    0.1   2.00   5.00    2.00 ;  1   20   1   15   1   2.5]; % GANHOS QUENTE PELANDO
        
        A = cUnderActuatedController(A,Agains);  % ArDrone
        
        % Send control signals to robots
        A.rSendControlSignals;
    end

    % =====================================================================
    
    % Laço de controle de formação
    if toc(t_Formation) > T_FORMATION
        t_Formation = tic;
%         disp('F')
        
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];   % Posição dos membros da formação
        LF.mDirTrans;                                 % Transformada Direta X --> Q
               
        tt = toc(t); % Tempo atual
        tp = (3*(tt/T)^2 - 2*(tt/T)^3)*T;        
        
        LF.pPos.Qda = LF.pPos.Qd;
        
        LF.pPos.Qd(1) = rX*sin(w*tp);                   % xF
        LF.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
        LF.pPos.Qd(3) = 0.00;                           % zF
        LF.pPos.Qd(4) = 1.50;                           % rho
        LF.pPos.Qd(5) = 0;                              % alpha (frente/trás)
        LF.pPos.Qd(6) = 0.00;                           % beta  (lateral)
        
        LF.pPos.dQd = (LF.pPos.Qd-LF.pPos.Qda)/LF.pPar.Ts;
        
        LF.mInvTrans;                                   % Transformada Inversa Qd --> Xd       
        
        % Control       
        % LF.mFormationControl;
        LF.mFormationControlNSB('P'); 
        
        % Atribuindo posições desejadas
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xr(1:3);             % Posição desejada
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);            % Velocidade desejada

        % ArDrone
        A.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);        

        % Variable to feed plotResults function
        data(kk,:) = [P.pPos.Xd'     P.pPos.X' ...
            A.pPos.Xd'        A.pPos.X'        ...
            LF.pPos.Qd'       LF.pPos.Q'      ...
            P.pSC.Ud(1:2)'    P.pSC.U(1:2)'    ...
            A.pSC.Ud'         A.pSC.U'         ...
            toc(t)];
        kk = kk + 1;         
        
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
        catch
        end
        
        % Pioneer
        P.mCADdel
        P.mCADplot(.75,'k');
        
        % ArDrone
        A.mCADplot;
        
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
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

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
