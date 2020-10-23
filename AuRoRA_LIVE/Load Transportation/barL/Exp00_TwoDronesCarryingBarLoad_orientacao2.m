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


% gains1 = [0.5    2.00    0.5   2.00   08    2.00 ;  1   15   1   15   1   1 ; 0 1 0 1 0 1];
% gains2 = [0.5    2.00    0.5   2.00   10    2.00 ;  1   15   1   15   1   1 ; 0 1 0 1 0 1];


%% ATENÇÃO, COLOQUEI UM GANHO MANUAL EM KZ
gains1 = [0.13    2.00    0.13   2.00   3    15.00 ;  1   15   1   15   1   1 ; 0 1 0 1 1 1];
gains2 = [0.13    2.00    0.13   2.00   3    15.00 ;  1   15   1   15   1   1 ; 0 1 0 1 1 1];

gains3 = gains1;
gains4 = gains2;
% gains3 = [0.55    2.00    0.55   2.00   10    2.00 ;  1   15   1   15   1   1 ; 0 1 0 1 2 1];
% gains4 = [0.55    2.00    0.55   2.00   10    2.00 ;  1   15   1   15   1   1 ; 0 1 0 1 2 1];

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
barL.pPar.l1 = .91;
barL.pPar.l2 = .91;

% Comprimento da barL
barL.pPar.L = 1.5;  
% barL.pPar.L = 1.43; Antes

% Massa da barL
barL.pPar.m = .1;      %Kg

% Constantes de control
barL.pPar.K1 = .5*diag([1 1 1 1 1 1]);    % kinematic control gain  - controls amplitude
% barL.pPar.K1 = 1*diag([2.5 2.5 .7 2.5 2.5 .7]);    % como estava
barL.pPar.K2 = 1*diag([0.2 0.2 0.5 0.2 0.2 0.5]);        % kinematic control gain - control saturation

%% Robot Initialization
A{1} = ArDrone;
A{2} = ArDrone;
L{1} = Load
L{2} = Load;

mCADcolor(A{1},[1 0 0])
mCADcolor(A{2},[0 1 0])

%% Joystick Initialization
J = JoyControl;


%% Optitrack Initialization
OPT = OptiTrack;
OPT.Initialize;

% Initial Pose
idA{1} = getID(OPT,ArDrone,1);
idA{2} = getID(OPT,ArDrone,2);
idL{1} = getID(OPT,Load,3);
idL{2} = getID(OPT,Load,4);

% Valores: exposure 2000 micro_s threshold 200
rb = OPT.RigidBody;
    if rb(idA{1}).isTracked
    A{1} = getOptData(rb(idA{1}),A{1});
    end

    if rb(idA{2}).isTracked
    A{2} = getOptData(rb(idA{2}),A{2});
    end
    
    if rb(idL{1}).isTracked
    L{1} = getOptData(rb(idL{1}),L{1});
    end

    if rb(idL{2}).isTracked
    L{2} = getOptData(rb(idL{2}),L{2});
    end
    
%     pause(3)

% Envio de comando take off para os drones
A{1}.pPar.ip = '192.168.1.61';
A{1}.rConnect;

A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
A{2}.pPar.ip = '192.168.1.71';
A{2}.rConnect;

A{1}.rTakeOff;
A{2}.rTakeOff;

%% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
barL.pPos.Qd = [.25 0 .4 deg2rad(90) deg2rad(0) barL.pPar.L]';

% Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d
barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

% Povoando a variável X e X_load da classe
barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

% Cálculo do erro nos drones
barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

%% Simulation Window
% fig = figure(1);
% axis([-6 6 -6 6 0 6])
% grid on
% 
% % Draw Robots
% A{1}.mCADplot; 
% A{2}.mCADplot;
% 
% % Draw Load and cables
% hold on
% plot_load = plot3([barL.pPos.X_load(1) barL.pPos.X_load(4)],[barL.pPos.X_load(2) barL.pPos.X_load(5)],[barL.pPos.X_load(3) barL.pPos.X_load(6)],'r','LineWidth',4);
% plot_l1 =  plot3([barL.pPos.X(1) barL.pPos.X_load(1)],[barL.pPos.X(2) barL.pPos.X_load(2)],[barL.pPos.X(3) barL.pPos.X_load(3)],'k');
% plot_l2 =  plot3([barL.pPos.X(4) barL.pPos.X_load(4)],[barL.pPos.X(5) barL.pPos.X_load(5)],[barL.pPos.X(6) barL.pPos.X_load(6)],'k');
% pause(3);
%% Simulation LOOP

% timers
T_exp = 120; % tempo de experimento
T_run = 1/30; % período de amostragem do experimento
t_run = tic;
t_total = tic; % tempo que o experimento está rodando (running)
t_plot = tic; % 
T_plot = .3; % período de plotagem
t_task = tic;
T_task = 6;
i_task = 0;
t_exp = tic;

while toc(t_run) < T_exp

    if toc(t_run) > T_run 
        t_run = tic;
                 
        
        %% TAREFA DE POSIÇÃO
        if sum(abs(barL.pPos.Xtil) > .15) == 0 && i_task == 0
               t_task = tic;
               i_task = i_task + 1;
               barL.pPos.Qd = [.25 0 .6 deg2rad(180) deg2rad(0) barL.pPar.L]';

               disp('Tarefa 2')

                %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
                % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d        
                barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
                barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
                barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

                % Povoando a variável X e X_load da classe
                barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
                barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

                % Cálculo do erro nos drones
                barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;


        end
        
        
        if sum(abs(barL.pPos.Xtil) > .35) == 0 && i_task == 1 && toc(t_task) > T_task
               t_task = tic;
               i_task = i_task + 1;
               barL.pPos.Qd = [.25 0 .6 deg2rad(180) deg2rad(20) barL.pPar.L]';
               disp('Tarefa 3') 

               %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
                % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d        
                barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
                barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
                barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

                % Povoando a variável X e X_load da classe
                barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
                barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

                % Cálculo do erro nos drones
                barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

        end
        
           if sum(abs(barL.pPos.Xtil) > .35) == 0 && i_task == 2 && toc(t_task) > T_task
               t_task = tic;
               i_task = i_task + 1;
               barL.pPos.Qd = [.25 0 .6 deg2rad(270) deg2rad(20) barL.pPar.L]';
               disp('Tarefa 4') 

               %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
                % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d        
                barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
                barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
                barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

                % Povoando a variável X e X_load da classe
                barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
                barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

                % Cálculo do erro nos drones
                barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

           end

           if sum(abs(barL.pPos.Xtil) > .35) == 0 && i_task == 3 && toc(t_task) > T_task
               t_task = tic;
               i_task = i_task + 1;
               barL.pPos.Qd = [.25 0 .6 deg2rad(270) deg2rad(0) barL.pPar.L]';
               disp('Tarefa 5') 

               %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
                % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d        
                barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
                barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
                barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

                % Povoando a variável X e X_load da classe
                barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
                barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

                % Cálculo do erro nos drones
                barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

           end
           if sum(abs(barL.pPos.Xtil) > .35) == 0 && i_task == 4 && toc(t_task) > T_task
               t_task = tic;
               i_task = i_task + 1;
               barL.pPos.Qd = [.25 0 .6 deg2rad(270) deg2rad(-20) barL.pPar.L]';
               disp('Tarefa 6') 

               %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
                % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d        
                barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
                barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
                barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

                % Povoando a variável X e X_load da classe
                barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
                barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

                % Cálculo do erro nos drones
                barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

           end
           
           if sum(abs(barL.pPos.Xtil) > .35) == 0 && i_task == 5 && toc(t_task) > T_task
               t_task = tic;
               i_task = i_task + 1;
               barL.pPos.Qd = [.25 0 .6 deg2rad(0) deg2rad(-20) barL.pPar.L]';               disp('Tarefa 7') 

               %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
                % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d        
                barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
                barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
                barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

                % Povoando a variável X e X_load da classe
                barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
                barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

                % Cálculo do erro nos drones
                barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

           end
                
               if sum(abs(barL.pPos.Xtil) > .35) == 0 && i_task == 6 && toc(t_task) > T_task
               t_task = tic;
               i_task = i_task + 1;
               barL.pPos.Qd = [.25 0 .6 deg2rad(0) deg2rad(0) barL.pPar.L]';               
               disp('Tarefa 8') 

               %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
                % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d        
                barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
                barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
                barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

                % Povoando a variável X e X_load da classe
                barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
                barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

                % Cálculo do erro nos drones
                barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

                end
        
         if sum(abs(barL.pPos.Xtil) > .35) == 0 && i_task == 7 && toc(t_task) > T_task
                        
               A{1}.rLand;
               A{2}.rLand;
               disp('Pouso')
        end
        
        
        % Obter os dados dos sensores
        rb = OPT.RigidBody;
            if rb(idA{1}).isTracked
            A{1} = getOptData(rb(idA{1}),A{1});

            end

            if rb(idA{2}).isTracked
            A{2} = getOptData(rb(idA{2}),A{2});

            end

            if rb(idA{1}).isTracked
            L{1} = getOptData(rb(idL{1}),L{1});
            end

            if rb(idA{2}).isTracked
            L{2} = getOptData(rb(idL{2}),L{2});
            end
            
%         A{1}.pSC.U = [A{1}.pPos.X(4);A{1}.pPos.X(5);A{1}.pPos.X(9);A{1}.pPos.X(12)]; % populates actual control signal to save data
%         A{2}.pSC.U = [A{2}.pPos.X(4);A{2}.pPos.X(5);A{2}.pPos.X(9);A{2}.pPos.X(12)]; % populates actual control signal to save data

       
        %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
        % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d
        barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
        barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
        barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
        barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
        barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
        barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
        barL.pPos.dXd = zeros(6,1);  % Como é tarefa de posição, não há dXd. 

        % Povoando a variável X e X_load da classe
        barL.pPos.X = [A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
        barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

        % Cálculo do erro nos drones
        barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

        
        %% Implementação do controle
        barL.pPos.dXr = barL.pPos.dXd + barL.pPar.K1*tanh(barL.pPar.K2*barL.pPos.Xtil);
        A{1}.pPos.Xd(1:3) = barL.pPos.Xd(1:3);
        A{1}.pPos.Xd(7:9) = barL.pPos.dXr(1:3);
        A{2}.pPos.Xd(1:3) = barL.pPos.Xd(4:6);
        A{2}.pPos.Xd(7:9) = barL.pPos.dXr(4:6);
        
        % Aceleração da carga
        L{1}.pPos.dX = (L{1}.pPos.X - L{1}.pPos.Xa)/L{1}.pPar.Ts;
        L{2}.pPos.dX = (L{2}.pPos.X - L{2}.pPos.Xa)/L{2}.pPar.Ts;
        
        % Decomposição de trações - cálculo alpha beta
        barL.pPar.beta(1) = acos((barL.pPos.X(3) - barL.pPos.X_load(3))/barL.pPar.l1);
        barL.pPar.alpha(1) = atan2((barL.pPos.X_load(2) - barL.pPos.X(2)),(barL.pPos.X_load(1) - barL.pPos.X(1)));
        barL.pPar.beta(2) = acos((barL.pPos.X(6) - barL.pPos.X_load(6))/barL.pPar.l2);
        barL.pPar.alpha(2) = atan2((barL.pPos.X_load(5) - barL.pPos.X(5)),(barL.pPos.X_load(4) - barL.pPos.X(4)));
        
        % Decomposição de trações - cálculo trações
                
%         barL.pPar.t = [((barL.pPar.m/2*L{1}.pPos.dX(9) - barL.pPar.m/2*A{1}.pPar.g)/cos(barL.pPar.beta(1))) ((barL.pPar.m/2*L{2}.pPos.dX(9) - barL.pPar.m/2*A{2}.pPar.g)/cos(barL.pPar.beta(2)))]';
%         barL.pPar.T = [barL.pPar.t(1)*sin(barL.pPar.beta(1))*cos(barL.pPar.alpha(1)) + barL.pPar.m/2*L{1}.pPos.dX(7);
%                        barL.pPar.t(1)*sin(barL.pPar.beta(1))*sin(barL.pPar.alpha(1)) + barL.pPar.m/2*L{1}.pPos.dX(8);
%                        barL.pPar.t(1)*cos(barL.pPar.beta(1));
%                        barL.pPar.t(2)*sin(barL.pPar.beta(2))*cos(barL.pPar.alpha(2)) + barL.pPar.m/2*L{2}.pPos.dX(7);
%                        barL.pPar.t(2)*sin(barL.pPar.beta(2))*sin(barL.pPar.alpha(2)) + barL.pPar.m/2*L{2}.pPos.dX(8);
%                        barL.pPar.t(2)*cos(barL.pPar.beta(2))];
%         
        barL.pPar.T = [0;
                       0;
                       barL.pPar.m/2*A{1}.pPar.g;
                       0;
                       0;
                       barL.pPar.m/2*A{2}.pPar.g;];

%         
        A{1}.pPar.D(1:3) = barL.pPar.T(1:3);
        A{2}.pPar.D(1:3) = barL.pPar.T(4:6);
% %         
      
               % Controlador
        if i_task >= 1
            zdev = 6;
            A{1} = cUnderActuatedControllerDaniel(A{1},gains3,zdev);
            A{2} = cUnderActuatedControllerDaniel(A{2},gains4,zdev);
        else
            zdev = 1;
            A{1} = cUnderActuatedControllerDaniel(A{1},gains1,zdev);
            A{2} = cUnderActuatedControllerDaniel(A{2},gains2,zdev);
        end
       
        A{1} = J.mControl(A{1});                       % joystick command (priority)
        A{2} = J.mControl(A{2});                       % joystick command (priority)
        
        A{1}.rSendControlSignals;
        A{2}.rSendControlSignals;
        
        % Coleta de dados
        data = [data; A{1}.pPos.Xd(1:3)' A{1}.pPos.X(1:3)'  A{2}.pPos.Xd(1:3)' A{2}.pPos.X(1:3)' L{1}.pPos.X(1:3)' L{2}.pPos.X(1:3)' toc(t_exp) A{1}.pPos.intXtil' A{2}.pPos.intXtil'];
%                       1:3                    4:6                 7:9                10:12           13:15               16:18          19           20:22             23:25          
%                       
        %% Plotar evolução temporal
        if toc(t_plot) > T_plot
            t_plot = tic;
%             try
%                 delete(plot_load)
%                 delete(plot_l1)
%                 delete(plot_l2)
%                 
%             end
%             
%             A{1}.mCADplot;
%             A{2}.mCADplot;
%             plot_load = plot3([barL.pPos.X_load(1) barL.pPos.X_load(4)],[barL.pPos.X_load(2) barL.pPos.X_load(5)],[barL.pPos.X_load(3) barL.pPos.X_load(6)],'r','LineWidth',4);
%             plot_l1 =  plot3([barL.pPos.X(1) barL.pPos.X_load(1)],[barL.pPos.X(2) barL.pPos.X_load(2)],[barL.pPos.X(3) barL.pPos.X_load(3)],'k');
%             plot_l2 =  plot3([barL.pPos.X(4) barL.pPos.X_load(4)],[barL.pPos.X(5) barL.pPos.X_load(5)],[barL.pPos.X(6) barL.pPos.X_load(6)],'k');
            
%             barL.pPos.Xtil    %disp
%             [A{1}.pPos.X(1:3) A{1}.pPos.Xd(1:3) A{1}.pPos.Xtil(1:3) barL.pPos.dXr(1:3); A{2}.pPos.X(1:3) A{2}.pPos.Xd(1:3) A{2}.pPos.Xtil(1:3) barL.pPos.dXr(4:6)]   %disp
%                 barL.pPar.T   %disp
%             drawnow
            
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
% 
%         barL.pPar.t = [((barL.pPar.m/2*L{1}.pPos.dX(3) - barL.pPar.m/2*A{1}.pPar.g)/cos(barL.pPar.beta(1))) ((barL.pPar.m/2*L{2}.pPos.dX(3) - barL.pPar.m/2*A{2}.pPar.g)/cos(barL.pPar.beta(2)))]';
%         barL.pPar.T = [barL.pPar.t(1)*sin(barL.pPar.beta(1))*sin(barL.pPar.alpha(1));
%                        barL.pPar.t(1)*sin(barL.pPar.beta(1))*cos(barL.pPar.alpha(1));
%                        barL.pPar.t(1)*cos(barL.pPar.beta(1));
%                        barL.pPar.t(2)*sin(barL.pPar.beta(2))*sin(barL.pPar.alpha(2));
%                        barL.pPar.t(2)*sin(barL.pPar.beta(2))*cos(barL.pPar.alpha(2));
%                        barL.pPar.t(2)*cos(barL.pPar.beta(2))];
end
