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
Xo = [0 -1 0 0];
P.rSetPose(Xo');
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

% ArDrone
A.pPos.X(1:3) = [-0.15 -1 0.25];

data = [];

%% Configure simulation window

fig = figure(1);
axis([-4 4 -4 4 0 3]);
view(-21,30);
hold on;
grid on;

% Plot robots initial positions
% plot3(P.pPos.X(1),P.pPos.X(2),P.pPos.X(3),'r^','LineWidth',0.8);
% plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'b^','LineWidth',0.8);

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

LF.pPos.Qd(1) = P.pPos.X(1);            % xF
LF.pPos.Qd(2) = P.pPos.X(2);            % yF
LF.pPos.Qd(3) = P.pPos.X(3);            % zF
LF.pPos.Qd(4) = 1.50;                   % rho
LF.pPos.Qd(5) = 0.00;     % alpha
LF.pPos.Qd(6) = pi/3;     % beta
            
LF.mInvTrans;

fig1 = plot3(LF.pPos.Qd(1),LF.pPos.Qd(2),LF.pPos.Qd(3),'r.','MarkerSize',20,'LineWidth',2);
fig2 = plot3(LF.pPos.Xd(4),LF.pPos.Xd(5),LF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',2);

% Plotar linha rhof das posições desejadas
xl = [LF.pPos.Xd(1)   LF.pPos.Xd(4)];
yl = [LF.pPos.Xd(2)   LF.pPos.Xd(5)];
zl = [LF.pPos.Xd(3)   LF.pPos.Xd(6)];

pl = line(xl,yl,zl);
pl.Color = 'k';
pl.LineStyle = '--';
pl.LineWidth = 1.5;


drawnow;

%% Formation initial error

% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
LF.pPos.Xr = LF.pPos.X;

% Formation initial pose
LF.mDirTrans;

pause(3);

%% Simulation

fprintf('\nStart..............\n\n');



% Time variables initialization

T_PLOT = inf;       % Período de plotagem em tempo real

T_CONTROL = 0.020;
T_PIONEER = 0.100;
T_ARDRONE = 1/30;

t  = tic;
t_control = tic;
t_plot = tic;

t_Pioneer = tic;        % pioneer cycle
t_ArDrone = tic;        % ardrone cycle
t_integ = tic;


rX = 1.5; % [m]
rY = 1.5; % [m]
T = 30;   % [s]
w = 2*pi/T; % [rad/s]

T1 = 15;             % Drone finds Pioneer
T2 = 45 + T1;        %
T3 = 45 + T2;        %
T4 = 15 + T3;        %
T5 = 15 + T4;        % Land
T6 =  5 + T5;        % Emergency

while toc(t)< T6
    
    if toc(t_control) > T_CONTROL
               
        t_control = tic;
        
        P.rGetSensorData;    % Adquirir dados dos sensores - Pioneer
        A.rGetSensorData;    % Adquirir dados dos sensores - ArDrone   

        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];   % Posição dos membros da formação
        LF.mDirTrans;                                 % Transformada Direta X --> Q
        
        %% Trajectory
           
        if toc(t) > T1 && toc(t) < T4
            
            LF.pPos.Qd(1) = rX*cos(w*toc(t));       % xF
            LF.pPos.Qd(2) = rY*sin(w*toc(t));       % yF
            LF.pPos.Qd(3) = 0.00;                   % zF
            
            LF.pPos.dQd(1) = -w*rX*sin(w*toc(t));   % dxF
            LF.pPos.dQd(2) = w*rY*cos(w*toc(t));    % dyF
            LF.pPos.dQd(3) = 0.00;                  % dzF
            LF.pPos.dQd(4) = 0.00;                  % drho
            LF.pPos.dQd(5) = 0.00;                  % dalpha
            LF.pPos.dQd(6) = 0.00;   % dbeta
        
        end
        
        if toc(t) > T1 && toc(t) < T2
                
            LF.pPos.Qd(4) = 1.50;                   % rho
            LF.pPos.Qd(5) = 0.00;                   % alpha (frente/trás)
            LF.pPos.Qd(6) = pi/3;                   % beta  (lateral)
           
        elseif toc(t) > T2 && toc(t) < T3
            
            LF.pPos.Qd(4) = 1.50;                   % rho
            LF.pPos.Qd(5) = 0.00;                   % alpha
            LF.pPos.Qd(6) = -pi/3;                   % beta
            
        elseif toc(t) > T3 && toc(t) < T4
            
            LF.pPos.Qd(4) = 1.50;                   % rho
            LF.pPos.Qd(5) = 0.00;                   % alpha
            LF.pPos.Qd(6) = 0.00;                  % beta
        end
        
        if toc(t) > T4 && toc(t) < T5
            
            LF.pPos.Qd(1) = P.pPos.X(1);                                    % xF
            LF.pPos.Qd(2) = P.pPos.X(2);                                    % yF
            LF.pPos.Qd(3) = P.pPos.X(3);                                    % z
            LF.pPos.Qd(4) = 1.5 - (1.5 - 0.5)*(toc(t) - T4)/(T5 - T4);      % rho
            LF.pPos.Qd(5) = 0.00;                                           % alpha
            LF.pPos.Qd(6) = 0.00;                                           % beta
        
            LF.pPos.dQd = zeros(6,1);
            
        elseif toc(t) > T5
            
%             A.rEmergency;
            P.pSC.Ud = [0; 0];
            P.rSendControlSignals;
%             A.rLand;
            
        end
        
        LF.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        
        %% Control
        
        % Formation Control
%         LF.mFormationControl_J_de_x;
        LF.mFormationControl_INTEGRATE_Xr(toc(t_integ));
        t_integ = tic;
%         LF.mFormationControl;
        
        % Pioneer
        P.pPos.Xd(1:2) = LF.pPos.Xr(1:2);             % Posição desejada
        P.pPos.Xd(7:8) = LF.pPos.dXr(1:2);            % Velocidade desejada
        
        % Compensador Dinâmico
        
        % Ganhos pré-definidos
%       cGains = [  0.75 0.75 0.12 0.035  ];

        % Ganhos Valentim
%         cGains = [  7.0  7.0  0.1  0.1  ];

%         sInvKinematicModel(P,LF.pPos.dXr(1:2));  % sem essa conversão o compensador não tem o valor de pSC.Ur, por isso o pioneer estava ficando parado
%         P = fCompensadorDinamico(P,cGains);

        % Controlador Dinâmico
        
        % Ganhos pré-definidos
      cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];  

%         cgains = [ 0.10  0.10  0.75  0.75  0.75  0.75  0.10  0.05 ];  
        P = fDynamicController(P,cgains);     % Pioneer Dynamic Compensator
        [P.pSC.Ud(1:2)' ; P.pSC.Ur(1:2)']
        % Drone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        
        A.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        
%         A.pPos.Xd(6) = P.pPos.X(6); 
%         A.pPos.Xd(12) = P.pPos.X(12);  % dPsi
        
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
        
%         data = [  data  ; P.pPos.Xd'     P.pPos.X'        P.pSC.Ud(1:2)'    P.pSC.U(1:2)'...
%                           A.pPos.Xd'     A.pPos.X'        A.pSC.Ud'         A.pSC.U' ...
%                           LF.pPos.Qd'    LF.pPos.Qtil'    LF.pPos.Xd'...
%                           toc(t)];
           
% %         %   1 -- 12      13 -- 24     25 -- 26          27 -- 28 
% %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
% %             
% %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60    
% %             A.pPos.Xd'   A.pPos.X'    A.pSC.Ud'         A.pSC.U'
% %             
% %         %   61 -- 66     67 -- 72       73 -- 78       79
% %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];
       
        %% Draw robots
        
% % % %         if toc(t_plot) > T_PLOT
% % % %             t_plot = tic;
% % % %             delete(pl);
% % % %             try
% % % %                 delete(fig1);
% % % %                 delete(fig2);
% % % %                 delete(fig3);
% % % %                 delete(fig4);
% % % %                 delete(fig5);
% % % %                 delete(pl);
% % % %                 delete(rho_line);
% % % %             catch
% % % %             end
% % % % 
% % % %             % Pioneer
% % % %             P.mCADdel
% % % %             P.mCADplot(.75,'k');
% % % % 
% % % %             % ArDrone
% % % %             A.mCADplot;
% % % % 
% % % %             % Percourse made
% % % %             fig1 = plot3(data(:,13),data(:,14),data(:,15),'r','LineWidth',1.0);
% % % %             fig2 = plot3(data(:,41),data(:,42),data(:,43),'b','LineWidth',1.0);
% % % %             fig3 = plot3(data(:,61),data(:,62),data(:,63),'m--','LineWidth',0.5);
% % % %             fig4 = plot3(data(:,76),data(:,77),data(:,78),'c--','LineWidth',0.5);
% % % % 
% % % %             % Plotar linha rhof
% % % %                        
% % % %             xl = [LF.pPos.X(1)   LF.pPos.X(4)];
% % % %             yl = [LF.pPos.X(2)   LF.pPos.X(5)];
% % % %             zl = [LF.pPos.X(3)   LF.pPos.X(6)];
% % % % 
% % % %             rho_line = line(xl,yl,zl);
% % % %             rho_line.Color = 'g';
% % % %             rho_line.LineStyle = '-';
% % % %             rho_line.LineWidth = 1.5;
% % % % 
% % % %             fig5 = plot3(LF.pPos.Xd(4),LF.pPos.Xd(5),LF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',1.0);
% % % % 
% % % %             % Plotar linha rhof das posições desejadas
% % % %             xl = [LF.pPos.Xd(1)   LF.pPos.Xd(4)];
% % % %             yl = [LF.pPos.Xd(2)   LF.pPos.Xd(5)];
% % % %             zl = [LF.pPos.Xd(3)   LF.pPos.Xd(6)];
% % % % 
% % % %             pl = line(xl,yl,zl);
% % % %             pl.Color = 'k';
% % % %             pl.LineStyle = '--';
% % % %             pl.LineWidth = 1.5;
% % % %             
% % % %             drawnow;
% % % %         end
              
    end
    
end

%% Close file and stop robot
fclose(Arq);

%% Send control signals
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

%% Plot results
% % % figure;
plotResults(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
