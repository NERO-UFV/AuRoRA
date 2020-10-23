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

P = Pioneer3DX(1);
A = ArDrone(1);

A.pPar.Ts = 1/30;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Formation 3D
LF = LineFormationControl;

LF.pPar.K1 = diag([  1.0   1.0   0.0   1.0   1.0   1.0  ]);     % kinematic control gain  - controls amplitude
LF.pPar.K2 = diag([  0.1   0.1   0.0   0.1   0.1   0.1 ]);     % kinematic control gain - control saturation


% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Robot/Simulator conection

% P.rConnect;

% Robot initial pose
% Pioneer P3DX
Xo = [0 -1 0 pi/2];
P.rSetPose(Xo');
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

% ArDrone
A.pPos.X(1:3) = [-0.15 -1 0.25];

%% Variable initialization

% Desired formation
  
%%%  [   xf     yf     zf     rhof   alfaf  betaf  ]
Qd = [   
         1.0    1.0    0.0    1.5    pi/4    pi/4   ;
         1.0   -1.0    0.0    1.5    pi/4    pi/4   ;
        -1.0   -1.0    0.0    1.5    pi/4    pi/4   ;
        -1.0    1.0    0.0    1.5    pi/4    pi/4   ;
         0.0    0.0    0.0    1.5    pi/4    pi/4   ];
     
%      Qd = [     0.0    0.0    0.0     1.5     0   pi/2    ]; 
         
cont = 1;     % counter to change desired position through simulation

data = [];

%% Configure simulation window

fig = figure(1);
axis([-2 2 -2 2 0 3]);
view(-21,30);
hold on;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
% Plot robots initial positions
plot3(P.pPos.X(1),P.pPos.X(2),P.pPos.X(3),'r^','LineWidth',0.8);
plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'b^','LineWidth',0.8);

% Draw robots
try
    A.mCADdel;
    P.mCADdel;
catch
end

% Pioneer
P.mCADplot(0.75,'k');

% ArDrone
% A.mCADload;
A.mCADcolor([0 0 1]);
A.mCADplot;

    
for kk = 1:size(Qd,1)
    
    LF.pPos.Qd = Qd(kk,:)';
    LF.mInvTrans;
    
    plot3(Qd(:,1),Qd(:,2),Qd(:,3),'r.','MarkerSize',20,'LineWidth',2);
    plot3(LF.pPos.Xd(4),LF.pPos.Xd(5),LF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',2);
    
    % Plotar linha rhof das posi��es desejadas
    xl = [LF.pPos.Xd(1)   LF.pPos.Xd(4)];
    yl = [LF.pPos.Xd(2)   LF.pPos.Xd(5)];
    zl = [LF.pPos.Xd(3)   LF.pPos.Xd(6)];
    
    pl = line(xl,yl,zl);
    pl.Color = 'k';
    pl.LineStyle = '--';
    pl.LineWidth = 2;

end

drawnow;

%% Formation initial error

% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
LF.pPos.Xr = LF.pPos.X;

% Formation initial pose
LF.mDirTrans;

% First desired position
LF.pPos.Qd = Qd(1,:)';

% Robots desired pose
LF.mInvTrans;

pause(3);

%% Simulation

fprintf('\nStart..............\n\n');


% Time variables initialization

T_CHANGE = 15;      % time to change desired positions [s]
T_PLOT = 30;       % Per�odo de plotagem em tempo real

T_CONTROL = 0.020;
T_PIONEER = 0.100;
T_ARDRONE = 1/30;

t  = tic;
t_control = tic;
t_plot = tic;

t_Pioneer = tic;        % pioneer cycle
t_ArDrone = tic;        % ardrone cycle
t_integ = tic;

while toc(t)< size(Qd,1)*(T_CHANGE)
    
    
    if toc(t_control) > T_CONTROL
               
        t_control = tic;
        
        P.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        A.rGetSensorData;    % Adquirir dados dos sensores - ArDrone   

        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];   % Posi��o dos membros da forma��o
        LF.mDirTrans;                                 % Transformada Direta X --> Q
        
        %% Desired positions
        
        if cont <= size(Qd,1)
            LF.pPos.Qd = Qd(cont,:)';
        end
        
        if toc(t)> cont*T_CHANGE
            cont = cont + 1;
        end
        
        LF.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        
        %% Control
        
        % Formation Control
%         LF.mFormationControl_J_de_x;
        LF.mFormationControl_INTEGRATE_Xr(toc(t_integ));
        t_integ = tic;
%         LF.mFormationControl;

%         disp([LF.pPos.Xr' ; LF.pPos.Xd']);
        
        % Pioneer
        P.pPos.Xd(1:2) = LF.pPos.Xr(1:2);             % Posi��o desejada
        P.pPos.Xd(7:8) = LF.pPos.dXr(1:2);            % Velocidade desejada
        
        % Compensador Din�mico
        
        % Ganhos pr�-definidos
%       cGains = [  0.75 0.75 0.12 0.035  ];

        % Ganhos Valentim
%         cGains = [  7.0  7.0  0.1  0.1  ];

%         sInvKinematicModel(P,LF.pPos.dXr(1:2));  % sem essa convers�o o compensador n�o tem o valor de pSC.Ur, por isso o pioneer estava ficando parado
%         P = fCompensadorDinamico(P,cGains);

        % Controlador Din�mico
        
        % Ganhos pr�-definidos
      cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];  

%         cgains = [ 0.10  0.10  0.75  0.75  0.75  0.75  0.10  0.05 ];  
        P = fDynamicController(P,cgains);     % Pioneer Dynamic Compensator

        % Drone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        
        A.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        
        
        % The Gains must be given in the folowing order
        % Rolagem Arfagem e Guinada (cabeceo)
        % kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
%         Agains = [   0.50    2.00    0.50   2.00   5.00    2.00 ;   1   20   1   15   1   2.5];% Default
        
        Agains =   [   0.1    2.00    0.1   2.00   5.00    2.00 ;  1   20   1   15   1   2.5]; % GANHOS QUENTE PELANDO
         
        A = cUnderActuatedController(A,Agains);  % ArDrone
               
        % Send control signals to robots
        
        if toc(t_Pioneer) > T_PIONEER
            P.rSendControlSignals;
        end
        
        if toc(t_ArDrone) > T_ARDRONE
            A.rSendControlSignals;
        end          
               
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)];
       
        %% Draw robots
        
        if toc(t_plot) > T_PLOT
            t_plot = tic;
            try
                delete(fig1);
                delete(fig2);
                delete(rho_line);
            catch
            end

            % Pioneer
            P.mCADdel
            P.mCADplot(.75,'k');

            % ArDrone
            A.mCADplot;

            % Percourse made
            fig1 = plot3(data(:,13),data(:,14),data(:,15),'r--','LineWidth',1.0);
            fig2 = plot3(data(:,41),data(:,42),data(:,43),'b--','LineWidth',1.0);

            % Plotar linha rhof
                       
            xl = [LF.pPos.X(1)   LF.pPos.X(4)];
            yl = [LF.pPos.X(2)   LF.pPos.X(5)];
            zl = [LF.pPos.X(3)   LF.pPos.X(6)];

            rho_line = line(xl,yl,zl);
            rho_line.Color = 'g';
            rho_line.LineStyle = '-';
            rho_line.LineWidth = 1.5;
            
            drawnow;
        end
        
        
    end
   
    % Simulation timeout
    if toc(t)> size(Qd,1)*size(Qd,1)*(T_CHANGE)
        disp('Timeout man!');
        break
    end
    
end

%% Close file and stop robot
fclose(Arq);

%% Send control signals
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

%% Plot results
% figure;
plotResults(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
