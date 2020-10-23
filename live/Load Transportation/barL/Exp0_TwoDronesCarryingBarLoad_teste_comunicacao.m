close all; clear; clc

try
    fclose(instrfindall);
catch
end

%% Buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% gains = [1 2 1 2 2 15;
%         2 13 2 15 1 5];
    
% gains = [.5 1 .5 1 10 4; 1 15 1 15 1 2.5];
gains = [.1 2 .1 2 5 2; 1 20 1 15 1 2.5]; % Ganhos Valentim

%% Variable Initialization
data = [];
barL.pPos.X = zeros(6,1);
barL.pPos.Xd = zeros(6,1);
barL.pPos.Xr = zeros(6,1);

barL.pPos.dXr = zeros(6,1);
barL.pPos.dXd = zeros(6,1);

barL.pPos.X_load = zeros(6,1);

barL.pPos.Qd = zeros(6,1);


%% Parâmetros da Carga barL 
% Comprimento dos cabos
barL.pPar.l1 = 0.5;
barL.pPar.l2 = 0.5;

% Comprimento da barL
barL.pPar.L = 2;  

% Massa da barL
barL.pPar.m = .1;         %Kg

% Constantes de controle
barL.pPar.K1 = 1*diag([0.8 0.8 0.5 .8 .8 .5]);    % kinematic control gain  - controls amplitude
barL.pPar.K2 = 1*diag([0.1 0.1 0.5 0.5 0.5 0.5]);        % kinematic control gain - control saturation

%% Robot Initialization
A{1} = ArDrone;
A{2} = ArDrone;

mCADcolor(A{1},[1 0 0])
mCADcolor(A{2},[0 1 0])

%% Joystick Initialization
J = JoyControl;

%% Optitrack Initialization
% OPT = OptiTrack;
% OPT.Initialize;

% Initial Pose
% idA{1} = getID(OPT,A{1});
% idA{2} = getID(OPT,A{2});
% idL{1} = getID(OPT,L{1});
% idL{2} = getID(OPT,L{2});

% rb = OPT.RigidBody;
% A{1} = getOptData(rb(idA{1}),A{1});
% A{2} = getOptData(rb(idA{2}),A{2});
% L{1} = getOptData(rb(idbarL{1}),L{1});
% L{2} = getOptData(rb(idbarL{2}),L{2});
% L{1}.pPos.Xa(1:6) = L{1}.pPos.X(1:6)
% L{2}.pPos.Xa(1:6) = L{2}.pPos.X(1:6)

% Envio de comando take off para os drones
A{1}.pPar.ip = '192.168.1.30'
A{1}.rConnect;
% 
A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
A{2}.pPar.ip = '192.168.1.1';
A{2}.rConnect;

A{1}.rTakeOff;
A{2}.rTakeOff;


%% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
% barL.pPos.Qd = [1 1 3 deg2rad(45) deg2rad(60) barL.pPar.L]';

% Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d
% barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
% barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
% barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
% barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
% barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
% barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
% barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

% Povoando a variável X e X_load da classe
% barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
% barL.pPos.X_load = [barL.pPos.X(1) barL.pPos.X(2) barL.pPos.X(3)-barL.pPar.l1 barL.pPos.X(4) barL.pPos.X(5) barL.pPos.X(6)-barL.pPar.l2]';

% barL.pPos.X_load = [L{1}.pPos.X(1:3) L{2}.pPos.X(1:3)];

% Cálculo do erro nos drones
% barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

%% Simulation Window
fig = figure(1);
axis([-6 6 -6 6 0 6])
grid on

% Draw Robots
A{1}.mCADplot; 
A{2}.mCADplot;

% Draw Load and cables
hold on
plot_load = plot3([barL.pPos.X_load(1) barL.pPos.X_load(4)],[barL.pPos.X_load(2) barL.pPos.X_load(5)],[barL.pPos.X_load(3) barL.pPos.X_load(6)],'r','LineWidth',4);
plot_l1 =  plot3([barL.pPos.X(1) barL.pPos.X_load(1)],[barL.pPos.X(2) barL.pPos.X_load(2)],[barL.pPos.X(3) barL.pPos.X_load(3)],'k');
plot_l2 =  plot3([barL.pPos.X(4) barL.pPos.X_load(4)],[barL.pPos.X(5) barL.pPos.X_load(5)],[barL.pPos.X(6) barL.pPos.X_load(6)],'k');
pause(3);
%% Simulation LOOP

% timers
T_exp = 120; % tempo de experimento
T_run = 1/30; % período de amostragem do experimento
t_run = tic;
t_total = tic; % tempo que o experimento está rodando (running)
t_plot = tic; % 
T_plot = .3; % período de plotagem

while toc(t_run) < T_exp
    if toc(t_run) > T_run
        t_run = tic;
        
        A{1} = J.mControl(A{1});           % joystick command (priority)
        A{1}.rSendControlSignals;
 
        A{2} = J.mControl(A{2});           % joystick command (priority)
        A{2}.rSendControlSignals;
        
        % Obter os dados dos sensores
%         rb = OPT.RigidBody;
%         A{1} = getOptData(rb(idA{1}),A{1});
%         A{2} = getOptData(rb(idA{2}),A{2});
        
        %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
%         barL.pPos.Qd = [1 1 3 deg2rad(25) deg2rad(0) barL.pPar.L]';
% 
%         % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d
%         barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
%         barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
%         barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
%         barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
%         barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
%         barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
%         barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

%         % Povoando a variável X e X_load da classe
%         barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
%         barL.pPos.X_load = [barL.pPos.X(1) barL.pPos.X(2) barL.pPos.X(3)-barL.pPar.l1 barL.pPos.X(4) barL.pPos.X(5) barL.pPos.X(6)-barL.pPar.l2]';
% 
%         % Cálculo do erro nos drones
%         barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;
% 
%         
        %% Implementação do controle
%         barL.pPos.dXr = barL.pPos.dXd + barL.pPar.K1*tanh(barL.pPar.K2*barL.pPos.Xtil);
%         A{1}.pPos.Xd(1:3) = barL.pPos.X(1:3);
%         A{1}.pPos.Xd(7:9) = barL.pPos.dXr(1:3);
%         A{2}.pPos.Xd(1:3) = barL.pPos.X(4:6);
%         A{2}.pPos.Xd(7:9) = barL.pPos.dXr(4:6);
%         
%         % Decomposição de trações - cálculo alpha beta
%         barL.pPar.beta(1) = acos((barL.pPos.X(3) - barL.pPos.X_load(3))/barL.pPar.l1);
%         barL.pPar.alpha(1) = atan2((barL.pPos.X_load(2) - barL.pPos.X(2)),(barL.pPos.X_load(1) - barL.pPos.X(1)));
%         barL.pPar.beta(2) = acos((barL.pPos.X(6) - barL.pPos.X_load(6))/barL.pPar.l2);
%         barL.pPar.alpha(2) = atan2((barL.pPos.X_load(5) - barL.pPos.X(5)),(barL.pPos.X_load(4) - barL.pPos.X(4)));
%         
%         % Decomposição de trações - cálculo trações
%         barL.pPar.T = [barL.pPar.m/2*A{1}.pPar.g*sin(barL.pPar.beta(1))*sin(barL.pPar.alpha(1));
%                        barL.pPar.m/2*A{1}.pPar.g*sin(barL.pPar.beta(1))*cos(barL.pPar.alpha(1));
%                        barL.pPar.m/2*A{1}.pPar.g*cos(barL.pPar.beta(1));
%                        barL.pPar.m/2*A{2}.pPar.g*sin(barL.pPar.beta(2))*sin(barL.pPar.alpha(2));
%                        barL.pPar.m/2*A{2}.pPar.g*sin(barL.pPar.beta(2))*cos(barL.pPar.alpha(2));
%                        barL.pPar.m/2*A{2}.pPar.g*cos(barL.pPar.beta(2))];
%         
%         
%         A{1}.pPar.D(1:3) = barL.pPar.T(1:3);
%         A{2}.pPar.D(1:3) = barL.pPar.T(4:6);
%         
%       
%         barL.pPar.T  %disp
%         
%         % Controlador
%         A{1} = cUnderActuatedController(A{1},gains);
%         A{2} = cUnderActuatedController(A{2},gains);
%         
%         A{1} = J.mControl(A{1});                       % joystick command (priority)
%         A{2} = J.mControl(A{2});                       % joystick command (priority)
%         
%         A{1}.rSendControlSignals;
%         A{2}.rSendControlSignals;
%              
%         % Coleta de dados
%         data = [data; A{1}.pPos.Xd' A{1}.pPos.X' A{2}.pPos.Xd' A{2}.pPos.Xd' toc(t_run)];
                    
        %% Plotar evolução temporal
        if toc(t_plot) > T_plot
            t_plot = tic;
            try
                delete(plot_load)
                delete(plot_l1)
                delete(plot_l2)
                
            end
            
            A{1}.mCADplot;
            A{2}.mCADplot;
            plot_load = plot3([barL.pPos.X_load(1) barL.pPos.X_load(4)],[barL.pPos.X_load(2) barL.pPos.X_load(5)],[barL.pPos.X_load(3) barL.pPos.X_load(6)],'r','LineWidth',4);
            plot_l1 =  plot3([barL.pPos.X(1) barL.pPos.X_load(1)],[barL.pPos.X(2) barL.pPos.X_load(2)],[barL.pPos.X(3) barL.pPos.X_load(3)],'k');
            plot_l2 =  plot3([barL.pPos.X(4) barL.pPos.X_load(4)],[barL.pPos.X(5) barL.pPos.X_load(5)],[barL.pPos.X(6) barL.pPos.X_load(6)],'k');
            
%             barL.pPos.Xtil    %disp

            drawnow
            
        % Simulation timout
        if toc(t_run) > T_exp
            disp('Timeout man!');
            break
        end
                
        
    end
    
    end
    
%     subplot(3,1,3)
%     plot(data(:,1),data(:,49),data(:,13),data(:,49))



%% Rascunho
% % Posição
% barL.pPos.X1 = [A{1}.pPos.X(1); A{1}.pPos.X(2); A{1}.pPos.X(3) - barL.pPar.l1];
% barL.pPos.X2 = [A{2}.pPos.X(1); A{2}.pPos.X(2); A{2}.pPos.X(3) - barL.pPar.l2];
% %% Direct Transformation (x1,y1,z1,x2,y2,z2) -> (xc,yc,zc,alpha,gamma,L)
% barL.pPos.Q(1) = (barL.pPos.X2(1) - barL.pPos.X1(1))/2; 
% barL.pPos.Q(2) = (barL.pPos.X2(2) - barL.pPos.X1(2))/2; 
% barL.pPos.Q(3) = (barL.pPos.X2(3) - barL.pPos.X1(3))/2; 
% barL.pPos.Q(4) = atan2((barL.pPos.X2(2) - barL.pPos.X1(2))/(barL.pPos.X2(1) - barL.pPos.X1(1)));
% barL.pPos.Q(5) = atan2((barL.pPos.X2(3) - barL.pPos.X1(3))/(norm([barL.pPos.X2(2)-barL.pPos.X1(2) barL.pPos.X2(1)-barL.pPos.X1(1)])));
% barL.pPos.Q(6) = barL.pPar.L;
end
