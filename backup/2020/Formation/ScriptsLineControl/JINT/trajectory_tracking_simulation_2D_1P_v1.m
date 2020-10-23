%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [ xf yf zf rho alfa beta ]
% Initial Comands

clear; close all; clc;
try
    fclose(instrfindall);
catch
end

% Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Load Classes

loop_inc = 0.030;

% Robots
P{1} = Pioneer3DX(1);
A{1} = ArDrone(61);


% Formation 3D
LF{1} = LineFormationControl;

%% Robots initial pose
% Pioneer
Xo = [0 0 0 0];
P{1}.rSetPose(Xo');
P{1}.pSC.Ud = [0; 0];
P{1}.rSendControlSignals;    % Pioneer

% ArDrone
A{1}.pPos.X(1:3) = [-0.15 0 0.25];
A{1}.pPos.X(6) = 0;

%% Formation initial error
% Formation initial pose
LF{1}.pPos.X = [ P{1}.pPos.X(1:3); A{1}.pPos.X(1:3) ];
LF{1}.pPos.Xr = LF{1}.pPos.X;
LF{1}.mDirTrans;

%% Configure simulation window

data = [];

fig = figure(1);
axis([-4 4 -4 4 0 3]);
view(-21,30);
hold on;
grid on;

% Draw robots
try
    A{1}.mCADdel;
    P{1}.mCADdel;
catch
end

% Pioneer
P{1}.mCADplot(0.75,'k');

% ArDrone
A{1}.mCADcolor([0 0 1]);
A{1}.mCADplot;

drawnow;

%% Variable initialization
data = [];

% Time variables initialization
T_control = loop_inc;
T_plot = 5;       % Período de plotagem em tempo real

rX = 1.0;      % [m]
rY = 1.0;      % [m]
rho = 1.5;      % [m]
alpha = 0;
beta = 0;
T = 45;         % [s]
Tf = 90;        % [s]
w = 2*pi/T;     % [rad/s]

T1 = 85;             % Lemniscata
T2 =  3.0 + T1;         % Aproximação + Emergency
T3 =  0.1 + T2;         % Andando com o Drone pousado
T4 =  2.0 + T3;         % Parando o Pioneer

caso = 1;
pause(4);
fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_integA = tic;
t  = tic;
t_control = tic;
t_plot = tic;

while toc(t) < T4

    if toc(t_control) > T_control             
        
        t_control = tic;
        
        P{1}.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        A{1}.rGetSensorData;    % Adquirir dados dos sensores - ArDrone  
        
        %% Trajectory Planner        
        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf; 
        X_traj = rX*sin(w*tp);
        Y_traj = rY*sin(2*w*tp);
        dX_traj = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);
        dY_traj = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);
        
        if toc(t) < T1     
            if caso == 1
                fprintf('\n\nCaso 1: Leminiscata\n');
                caso = 2;
            end
            LF{1}.pPos.Qd(1) = X_traj;       % xF
            LF{1}.pPos.Qd(2) = Y_traj;       % yF
            LF{1}.pPos.Qd(4) = rho;          % rho
            LF{1}.pPos.dQd(1) = dX_traj;     % dxF
            LF{1}.pPos.dQd(2) = dY_traj;     % dyF
            
            LF{1}.pPos.Qd(4) = rho;                            % rho
            LF{1}.pPos.Qd(5) = alpha;                          % alpha (frente/trás)
            LF{1}.pPos.Qd(6) = beta;                           % beta  (lateral)

        elseif toc(t) > T1 && toc(t) < T2       
            if caso == 2
                fprintf('\n\nCaso 2: Aproximação + Emergency\n');
                caso = 3;
            end      
            LF{1}.pPos.Qd(1) = X_traj;       % xF
            LF{1}.pPos.Qd(2) = Y_traj;       % yF
            LF{1}.pPos.Qd(4) = rho/2;        % rho
            LF{1}.pPos.dQd(1) = dX_traj;     % dxF
            LF{1}.pPos.dQd(2) = dY_traj;     % dyF
            
        elseif toc(t) > T2 && toc(t) < T3  
            if caso == 3
                fprintf('\n\nCaso 3: Andando com o Drone pousado\n');
                caso = 4;           
            end
            LF{1}.pPos.Qd(1) = X_traj;       % xF
            LF{1}.pPos.Qd(2) = Y_traj;       % yF
            LF{1}.pPos.Qd(4) = rho/2;        % rho
            LF{1}.pPos.dQd(1) = dX_traj;     % dxF
            LF{1}.pPos.dQd(2) = dY_traj;     % dyF
        end


        A{1}.pPos.X(7:12) = (A{1}.pPos.X(1:6)-A{1}.pPos.Xa(1:6))/toc(t_integA);
        A{1}.pSC.U = [1 ; -1 ; 1; -1].*[A{1}.pPos.X(4);A{1}.pPos.X(5);A{1}.pPos.X(9);A{1}.pPos.X(12)]./A{1}.pPar.uSat; % populates actual control signal to save data
        t_integA = tic;
        
        LF{1}.pPos.X = [P{1}.pPos.X(1:3); A{1}.pPos.X(1:3)];   % Posição dos membros da formação
        LF{1}.mDirTrans;                                 % Transformada Direta X --> Q

        %% Control
        LF{1}.mFormationControl(toc(t_integF));

%         LF{1}.mFormationControl_NullSpace('F',toc(t_integF)); % 'P' ---> Position    'F' ---> Form
        t_integF = tic;
        
        % Ganhos
        
        % Ganhos NP (VALIDADOS)
%         LF.pPar.K1 = diag([   1.0    1.0    1.0    1.0   1.0    1.0   ]);     % FORA
%         LF.pPar.K2 = diag([   0.4    0.4    0.4    0.2   0.2    0.2   ]);     % DENTRO
        
        % Ganhos F (VALIDADOS)
        LF{1}.pPar.K1 = diag([   20.0  20.0    20.0   2.0   2.0    2.0   ]);     % kinematic control gain  - controls amplitude
        LF{1}.pPar.K2 = diag([    0.1   0.1     0.1   0.1   0.1    0.1   ]);     % kinematic control gain - control saturation
   
        % Ganhos P
%         LF.pPar.K1 = diag([   1.0   1.0     1.0    5.0   5.0    5.0   ]);     % kinematic control gain  - controls amplitude
%         LF.pPar.K2 = diag([   0.4   0.4     0.4    0.1   0.1    0.1   ]);     % kinematic control gain - control saturation
 
        LF{1}.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        
        P{1}.pPos.Xda = P{1}.pPos.Xd;    % save previous posture
        P{1}.pPos.Xd(1:2) = LF{1}.pPos.Xr(1:2);             % Posição desejada
        P{1}.pPos.Xd(7:8) = LF{1}.pPos.dXr(1:2);            % Velocidade desejada
        
        A{1}.pPos.Xda = A{1}.pPos.Xd;    % save previous posture
        A{1}.pPos.Xd(1:3) = LF{1}.pPos.Xr(4:6);
        A{1}.pPos.Xd(7:9) = LF{1}.pPos.dXr(4:6);

        A{1}.pPos.Xd(6) = P{1}.pPos.X(6);
        A{1}.pPos.Xd(12) = P{1}.pPos.X(12);  % dPsi
            
            %% Save data

        % Variable to feed plotResults function
        data = [  data  ; P{1}.pPos.Xd'     P{1}.pPos.X'        P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)'...
                          A{1}.pPos.Xd'     A{1}.pPos.X'        A{1}.pSC.Ud'         A{1}.pSC.U' ...
                          LF{1}.pPos.Qd'    LF{1}.pPos.Qtil'    LF{1}.pPos.Xd'...
                          toc(t)];
           
% %         %   1 -- 12      13 -- 24     25 -- 26          27 -- 28 
% %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
% %             
% %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60    
% %             A.pPos.Xd'   A.pPos.X'    A.pSC.Ud'         A.pSC.U'
% %             
% %         %   61 -- 66     67 -- 72       73 -- 78       79
% %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];
   
        %% Send control signals to robots
        % Pioneer
        cgains = [ 0.1  0.1  0.75  0.75  0.75  0.75  0.10  0.03 ];
        P{1} = fDynamicController(P{1},cgains);     % Pioneer Dynamic Compensator
        P{1}.rSendControlSignals;

        if toc(t) > T3
            if caso == 4
                fprintf('\n\nCaso 4: Parando Pioneer\n');
                caso = 5;
            end
            LF{1}.pPos.Qd = LF{1}.pPos.Q;
            LF{1}.pPos.dQd = zeros(6,1);
            P{1}.pSC.Ud = [0; 0];
        end
             
        % Drone
        % The Gains must be given in the folowing order
        % Rolagem Arfagem e Guinada (cabeceo)
        % kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
        Agains =   [   0.3   3.00    0.3   3.00   10.00    2.00 ;  1   20  1   20   1   3.0]; % GANHOS NP
        
        A{1} = cUnderActuatedController(A{1},Agains);  % ArDrone
        A{1}.rSendControlSignals; 
        
        
                %% Draw robots
        
        if toc(t_plot) > T_plot
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
            P{1}.mCADdel
            P{1}.mCADplot(.75,'k');

            % ArDrone
            A{1}.mCADplot;

            % Percourse made
            fig1 = plot3(data(:,13),data(:,14),data(:,15),'r','LineWidth',1.0);
            fig2 = plot3(data(:,41),data(:,42),data(:,43),'b','LineWidth',1.0);
            fig3 = plot3(data(:,61),data(:,62),data(:,63),'m--','LineWidth',0.5);
            fig4 = plot3(data(:,76),data(:,77),data(:,78),'c--','LineWidth',0.5);

            % Plotar linha rhof
                       
            xl = [LF{1}.pPos.X(1)   LF{1}.pPos.X(4)];
            yl = [LF{1}.pPos.X(2)   LF{1}.pPos.X(5)];
            zl = [LF{1}.pPos.X(3)   LF{1}.pPos.X(6)];

            rho_line = line(xl,yl,zl);
            rho_line.Color = 'g';
            rho_line.LineStyle = '-';
            rho_line.LineWidth = 1.5;

            fig5 = plot3(LF{1}.pPos.Xd(4),LF{1}.pPos.Xd(5),LF{1}.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',1.0);

            % Plotar linha rhof das posições desejadas
            xl = [LF{1}.pPos.Xd(1)   LF{1}.pPos.Xd(4)];
            yl = [LF{1}.pPos.Xd(2)   LF{1}.pPos.Xd(5)];
            zl = [LF{1}.pPos.Xd(3)   LF{1}.pPos.Xd(6)];

            pl = line(xl,yl,zl);
            pl.Color = 'k';
            pl.LineStyle = '--';
            pl.LineWidth = 1.5;
            
            drawnow;
        end
        
    end
end

%% Plot results
% % figure;
plotResultsLineControl(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
