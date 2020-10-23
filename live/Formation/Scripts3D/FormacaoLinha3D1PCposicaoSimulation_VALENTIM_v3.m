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

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Open file to save data

NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_FormacaoLinha3D')
Arq = fopen(['FL3dSIMU_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Load Classes

% Robots

PA = Pioneer3DX(1);
PB = Pioneer3DX(2);

A = ArDrone(1);
A.pPar.Ts = 1/30;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Formation 3D
LF_3D = LineFormation3D;

% Formation 2D
LF_2D = LineFormation2D('robot');


% Ganhos do Valentim
LF_3D.pPar.K1 = diag([  1.0   1.0   1.0   3.0   0.1   0.1  ]);     % kinematic control gain  - controls amplitude
LF_3D.pPar.K2 = diag([  0.1   0.1   0.1   0.1   0.1   0.1  ]);     % kinematic control gain - control saturation


% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Robot/Simulator conection

% P.rConnect;

% Robot initial pose
% Pioneer P3DX
Xo_A = [0 -2 0 0];
PA.rSetPose(Xo_A');
PA.pSC.Ud = [0; 0];
PA.rSendControlSignals;    % Pioneer

Xo_B = [0 2 0 0];
PB.rSetPose(Xo_B');
PB.pSC.Ud = [0; 0];
PB.rSendControlSignals;    % Pioneer

pause(3);

% ArDrone
A.pPos.X(1:3) = [-2 0 0];

%% Variable initialization

% Desired formation

%       [   xf     yf     zf     rhof   alfaf  betaf     ]
Qd_3D = [   2.0    2.0    0.0    2.0    0.0    pi/2.25   ;
            2.0   -2.0    0.0    2.0    0.0    pi/2.25   ;
           -2.0   -2.0    0.0    1.0    0.0    pi/2.25   ;
           -2.0    2.0    0.0    2.0    0.0    pi/2.25   ;
            0.0    0.0    0.0    1.0    0.0    pi/2.25   ];
     
%       [   xf     yf     rof    alfaf  ]
Qd_2D = [   2.0    2.0    1.0    2*pi/3    ;
            2.0   -2.0    1.0    2*pi/3    ;
           -2.0   -2.0    1.0    2*pi/3    ;
           -2.0    2.0    1.0    2*pi/3    ;
            0.0    0.0    1.0    2*pi/3    ];
         
cont = 1;     % counter to change desired position through simulation

data = [];

%% Configure simulation window

fig = figure(1);
axis([-4 4 -4 4 0 4]);
view(-21,30);
hold on;
grid on;

% Plot robots initial positions
plot3(PA.pPos.X(1),PA.pPos.X(2),PA.pPos.X(3),'k^','LineWidth',0.8);
plot3(PB.pPos.X(1),PB.pPos.X(2),PB.pPos.X(3),'r^','LineWidth',0.8);
plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'b^','LineWidth',0.8);

% Draw robots
try
    A.mCADdel;
    PA.mCADdel;
    PB.mCADdel;
catch
end

% Pioneer
PA.mCADplot(0.75,'k');
PB.mCADplot(0.75,'r');

% ArDrone
A.mCADload;
A.mCADcolor([0 0 1]);
A.mCADplot;

    
for kk = 1:size(Qd_3D,1)
    
    % 3D
    LF_3D.pPos.Qd = Qd_3D(kk,:)';
    LF_3D.mInvTrans;
    
    plot3(Qd_3D(:,1),Qd_3D(:,2),Qd_3D(:,3),'k.','MarkerSize',20,'LineWidth',2);
    plot3(LF_3D.pPos.Xd(4),LF_3D.pPos.Xd(5),LF_3D.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',2);
    
    % Plotar linha rhof das posições desejadas
    xl_3D = [LF_3D.pPos.Xd(1)   LF_3D.pPos.Xd(4)];
    yl_3D = [LF_3D.pPos.Xd(2)   LF_3D.pPos.Xd(5)];
    zl_3D = [LF_3D.pPos.Xd(3)   LF_3D.pPos.Xd(6)];
    
    pl_3D = line(xl_3D,yl_3D,zl_3D);
    pl_3D.Color = 'b';
    pl_3D.LineStyle = '--';
    pl_3D.LineWidth = 2;
    
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %
    
    % 2D
    LF_2D.pPos.Qd = Qd_2D(kk,:)';
    LF_2D.mInvTrans('d');
    
    plot(LF_2D.pPos.Xd(3),LF_2D.pPos.Xd(4),'r.','MarkerSize',20,'LineWidth',2);
    
    % Plotar linha rhof das posições desejadas
    xl_2D = [LF_2D.pPos.Xd(1)   LF_2D.pPos.Xd(3)];
    yl_2D = [LF_2D.pPos.Xd(2)   LF_2D.pPos.Xd(4)];
   
    pl_2D = line(xl_2D,yl_2D);
    pl_2D.Color = 'r';
    pl_2D.LineStyle = '--';
    pl_2D.LineWidth = 2;
end

drawnow;
% pause(3);

%% Formation initial error 3D

% Formation initial pose
LF_3D.pPos.X = [PA.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
LF_3D.mDirTrans;

% First desired position
LF_3D.pPos.Qd = Qd_3D(1,:)';

% Robots desired pose
LF_3D.mInvTrans;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Formation initial error 2D

% Formation initial pose
LF_2D.pPos.X = [PA.pPos.X(1:6); PB.pPos.X(1:6)];

% Formation initial pose
LF_2D.mDirTrans;

% First desired position
LF_2D.pPos.Qd = Qd_2D(1,:)';

% Robots desired pose
LF_2D.mInvTrans('d');

%% Simulation
clc;
disp('Start..............');

% Time variables initialization

T_CHANGE = 30;      % time to change desired positions [s]
T_PLOT = 0.5;       % Período de plotagem em tempo real

T_CONTROL = 0.020;
T_PIONEER = 0.100;
T_ARDRONE = 1/30;

t  = tic;
t_control = tic;
t_plot = tic;

t_Pioneer = tic;        % pioneer cycle
t_ArDrone = tic;        % ardrone cycle

while toc(t)< size(Qd_3D,1)*(T_CHANGE)
    
    if toc(t_control) > 1/30
                
        t_control = tic;
        
        PA.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        PB.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        A.rGetSensorData;    % Adquirir dados dos sensores - ArDrone   

        LF_3D.pPos.X = [PA.pPos.X(1:3); A.pPos.X(1:3)];   % Posição dos membros da formação
        LF_3D.mDirTrans;                                 % Transformada Direta X --> Q
        
        LF_2D.pPos.X = [PA.pPos.X(1:6); PB.pPos.X(1:6)];   % Posição dos membros da formação
        LF_2D.mDirTrans;                                 % Transformada Direta X --> Q
        
        %% Desired positions
        
        if cont <= size(Qd_3D,1)
            LF_3D.pPos.Qd = Qd_3D(cont,:)';
            LF_2D.pPos.Qd = Qd_2D(cont,:)';
        end
        
        if toc(t)> cont*T_CHANGE
            cont = cont + 1;
        end
        
        LF_3D.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        LF_2D.mInvTrans('d');                            % Transformada Inversa Qd --> Xd

        
        %% Control
        
        % Formation Control
        LF_3D.mFormationControl;
        LF_2D.mFormationControl;
        
        % Pioneer
        PA.pPos.Xd(1:2) = LF_3D.pPos.Xd(1:2);             % Posição desejada
        PA.pPos.Xd(7:8) = LF_3D.pPos.dXr(1:2);            % Velocidade desejada
        
        PB.pPos.Xd(1:2) = LF_2D.pPos.Xd(3:4);             % Posição desejada
        PB.pPos.Xd(7:8) = LF_2D.pPos.dXr(3:4);            % Velocidade desejada

        % Controlador Dinâmico

        PA = fDynamicController(PA);     % Pioneer Dynamic Compensator
        PB = fDynamicController(PB);     % Pioneer Dynamic Compensator

        % ArDrone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        A.pPos.Xd(1:3) = LF_3D.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = LF_3D.pPos.dXr(4:6);
%         A.pPos.Xd(6)  = atan2(A.pPos.Xd(8),A.pPos.Xd(7));
        A = cUnderActuatedController(A);
               
        % Send control signals to robots
        
        if toc(t_Pioneer) > T_PIONEER
            PA.rSendControlSignals;
            PB.rSendControlSignals;
        end
        
        if toc(t_ArDrone) > T_ARDRONE
            A.rSendControlSignals;
        end          
               
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[PA.pPos.Xd' PA.pPos.X' PA.pSC.Ud(1:2)' PA.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' ...
            LF_3D.pPos.Qd' LF_3D.pPos.Qtil' toc(t) ...
            PB.pPos.Xd' PB.pPos.X' PB.pSC.Ud(1:2)' PB.pSC.U(1:2)' ...
            LF_2D.pPos.Qd' LF_2D.pPos.Qtil']);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        data = [data; PA.pPos.Xd' PA.pPos.X' PA.pSC.Ud(1:2)' PA.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' ...
            LF_3D.pPos.Qd' LF_3D.pPos.Qtil' toc(t) ...
            PB.pPos.Xd' PB.pPos.X' PB.pSC.Ud(1:2)' PB.pSC.U(1:2)' ...
            LF_2D.pPos.Qd' LF_2D.pPos.Qtil'];
       
        %% Draw robots
        
        if toc(t_plot) > T_PLOT
            t_plot = tic;
            try
                delete(fig1);
                delete(fig2);
                delete(fig3);
                delete(rho_line_3D);
                delete(rho_line_2D);
            catch
            end

            % Pioneer
            PA.mCADdel
            PA.mCADplot(.75,'k');
            
            PB.mCADdel
            PB.mCADplot(.75,'r');

            % ArDrone
            A.mCADplot;

            % Percourse made
            fig1 = plot3(data(:,13),data(:,14),data(:,15),'k--','LineWidth',1.0);
            fig2 = plot3(data(:,41),data(:,42),data(:,43),'b--','LineWidth',1.0);
            fig3 = plot3(data(:,86),data(:,87),data(:,88),'r--','LineWidth',1.0);

            % Plotar linha rhof
                       
            xl_3D = [LF_3D.pPos.X(1)   LF_3D.pPos.X(4)];
            yl_3D = [LF_3D.pPos.X(2)   LF_3D.pPos.X(5)];
            zl_3D = [LF_3D.pPos.X(3)   LF_3D.pPos.X(6)];

            rho_line_3D = line(xl_3D,yl_3D,zl_3D);
            rho_line_3D.Color = 'b';
            rho_line_3D.LineStyle = '-';
            rho_line_3D.LineWidth = 1.0;
            
            xl_2D = [LF_2D.pPos.X(1)   LF_2D.pPos.X(7)];
            yl_2D = [LF_2D.pPos.X(2)   LF_2D.pPos.X(8)];

            rho_line_2D = line(xl_2D,yl_2D);
            rho_line_2D.Color = 'r';
            rho_line_2D.LineStyle = '-';
            rho_line_2D.LineWidth = 1.0;
            
            drawnow;
        end
        
        
    end
   
    % Simulation timeout
    if toc(t)> size(Qd_3D,1)*size(Qd_3D,1)*(T_CHANGE)
        disp('Timeout man!');
        break
    end
    
end

%% Close file and stop robot
fclose(Arq);

%% Send control signals

PA.pSC.Ud = [0; 0];
PA.rSendControlSignals;

PB.pSC.Ud = [0; 0];
PB.rSendControlSignals;    % Pioneer

%% Plot results
figure;
plotResults(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
