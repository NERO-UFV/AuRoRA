%% Triangular Formation 3D

% Q = [  | x    y    z  |    | p   q     beta |    | phi   theta   psi |  ]
% X = [  | x1   y1   z1 |    | x2   y2   z2 |      | x3    y3      z3  |  ]

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  %

% Initial Comands
clear variables; close all; clc;
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

%% Creating the robots

P{1} = Pioneer3DX(1);
A{2} = ArDrone(2);
A{3} = ArDrone(3);

% Formation 3D
F{1} = TriangularFormation3D;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Robot initial pose
% ArDrone
% P{1}.pPos.X(1:3) = [ 1  0  0 ];
% P{1}.pPos.X(6) = 0;

P{1}.rSetPose([1 0 0 0]');

A{2}.pPos.X(1:3) = [ -1  0  0 ];
A{2}.pPos.X(6) = 0;

A{3}.pPos.X(1:3) = [ 0 1  0 ];
A{3}.pPos.X(6) = 0;

F{1}.pPos.Xr = [ P{1}.pPos.X(1:3); A{2}.pPos.X(1:3); A{3}.pPos.X(1:3) ];

% Atribuindo posições desejadas   
Ts = F{1}.pPar.Ts;
F{1}.mTrajectoryPlanner_TF3D(0);

%% Configure simulation window

fig = figure(1);
axis([-3 3 -3 3 0 3]);
view(-21,30);
hold on;
grid on;

try
    delete(fig1);
    delete(fig2);
    delete(fig3);
    delete(fig4);
    delete(fig5);
    delete(triangle);
catch
end

P{1}.mCADplot(0.75,'r');
A{2}.mCADcolor([0 1 0]);
A{2}.mCADplot;
A{3}.mCADcolor([0 0 1]);
A{3}.mCADplot;

triangle = plot3([P{1}.pPos.X(1) A{2}.pPos.X(1) A{3}.pPos.X(1) P{1}.pPos.X(1)],...
    [P{1}.pPos.X(2) A{2}.pPos.X(2) A{3}.pPos.X(2) P{1}.pPos.X(2)],...
    [P{1}.pPos.X(3) A{2}.pPos.X(3) A{3}.pPos.X(3) P{1}.pPos.X(3)], '-b','LineWidth',1.5);

triangle_d = plot3([P{1}.pPos.Xd(1) A{2}.pPos.Xd(1) A{3}.pPos.Xd(1) P{1}.pPos.Xd(1)],...
    [P{1}.pPos.Xd(2) A{2}.pPos.Xd(2) A{3}.pPos.Xd(2) P{1}.pPos.Xd(2)],...
    [P{1}.pPos.Xd(3) A{2}.pPos.Xd(3) A{3}.pPos.Xd(3) P{1}.pPos.Xd(3)], '--m','LineWidth',1.5);

set(gca,'Box','on');

drawnow;

%% Simulation
pause(3);

fprintf('\nStart..............\n\n');

% Time variables initialization

T_PLOT = 1;       % Período de plotagem em tempo real

T_FORMATION = F{1}.pPar.Ts; % 150ms
T_ARDRONE = A{2}.pPar.Ts; %1/30

rX = 1.0; % [m]
rY = 1.0; % [m]
rho = 1.5;
T = 15;   % [s]
Tf = 45;
w = 2*pi/T; % [rad/s]

T1 = 115.0;             % Lemniscata
T2 =  15.0 + T1;        % Aproximação + Emergency
T3 =  15.1 + T2;        % Andando com o Drone pousado
T4 =  5.0 + T3;         % Parando o Pioneer

caso = 1;

% Data variables
kk = 1;
data = zeros(round(T4/T_FORMATION),133); % Data matrix

t  = tic;
t_plot = tic;

t_Formation = tic;      % Formation control

t_ArDrone_1 = tic;
t_ArDrone_2 = tic;
t_ArDrone_3 = tic;


while toc(t)< Tf
    % =====================================================================
    % Laço de controle dos robôs
    
    % ArDrone
    
    if toc(t_ArDrone_1) > 0.1
        t_ArDrone_1 = tic;
        P{1}.rGetSensorData;
        P{1} = fDynamicController(P{1});
%         P{1}.pPos.X(1:3) = F{1}.pPos.Xr(1:3) + 0*(real(rand()- rand()));
        P{1}.rSetPose([F{1}.pPos.Xr(1:3)' 0]');

        P{1}.rSendControlSignals;


    end
    
    if toc(t_ArDrone_2) > T_ARDRONE
        t_ArDrone_2 = tic;
        A{2}.pPos.X(1:3) = F{1}.pPos.Xr(4:6)+ 0*(real(rand()- rand()));

    end
    
    if toc(t_ArDrone_3) > T_ARDRONE
        t_ArDrone_3 = tic;
        A{3}.pPos.X(1:3) = F{1}.pPos.Xr(7:9)+ 0*(real(rand()- rand()));

    end
        
    % =====================================================================
    % Laço de controle de formação
    
    % Trajectory WEST VILLAGE Planner 2020
    if toc(t_Formation) > T_FORMATION
        t_Formation = tic;
        F{1}.mTrajectoryPlanner_TF3D(toc(t));
         
        F{1}.pPos.X = [ P{1}.pPos.X(1:3); A{2}.pPos.X(1:3); A{3}.pPos.X(1:3) ];

%         F{1}.cNoPriority_TF3D;
        F{1}.cNullSpaceBased_TF3D('N');

        % Atribuindo posições desejadas       
        % A1
        P{1}.pPos.Xda = P{1}.pPos.Xd;   % save previous posture
        P{1}.pPos.Xd(1:2) = F{1}.pPos.Xr(1:2);
        P{1}.pPos.Xd(7:8) = F{1}.pPos.dXr(1:2);
        
        % A2
        A{2}.pPos.Xda = A{2}.pPos.Xd;   % save previous posture
        A{2}.pPos.Xd(1:3) = F{1}.pPos.Xr(4:6);
        A{2}.pPos.Xd(7:9) = F{1}.pPos.dXr(4:6);
        
        % A3
        A{3}.pPos.Xda = A{3}.pPos.Xd;   % save previous posture
        A{3}.pPos.Xd(1:3) = F{1}.pPos.Xr(7:9);
        A{3}.pPos.Xd(7:9) = F{1}.pPos.dXr(7:9);
               
        % Variable to feed plotResults function    
        data(kk,:) = [  P{1}.pPos.Xd'     P{1}.pPos.X'        [P{1}.pSC.Ud'  0  0]       [P{1}.pSC.U'  0 0] ...
                        A{2}.pPos.Xd'     A{2}.pPos.X'        A{2}.pSC.Ud'         A{2}.pSC.U' ...
                        A{3}.pPos.Xd'     A{3}.pPos.X'        A{3}.pSC.Ud'         A{3}.pSC.U' ...
                        F{1}.pPos.Qd'     F{1}.pPos.Qtil'     F{1}.pPos.Q'         F{1}.pPos.Xr'...
                        toc(t)];
                    
                        kk = kk + 1;
        
        % %         %   1 -- 12             13 -- 24             25 -- 28            29 -- 32
        % %             A{1}.pPos.Xd'       A{1}.pPos.X'         A{1}.pSC.Ud'        A{1}.pSC.U'
        % %         
        % %         %   33 -- 44            45 -- 56             57 -- 60            61 -- 64 
        % %             A{2}.pPos.Xd'       A{2}.pPos.X'         A{2}.pSC.Ud'        A{2}.pSC.U'
        % %         
        % %         %   65 -- 76            77 -- 88             89 -- 92            93 -- 96
        % %             A{3}.pPos.Xd'       A{3}.pPos.X'         A{3}.pSC.Ud'        A{3}.pSC.U'
        % %        
        % %         %   97 -- 105           106 -- 114           115 -- 123         124 -- 132 
        % %             F{1}.pPos.Qd'       F{1}.pPos.Qtil'      F{1}.pPos.Q'       F{1}.pPos.dXr'
        % %
        % %         %   133 
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
        catch
        end
        
        try
            delete(triangle);
            delete(triangle_d);
        catch
        end

        P{1}.mCADdel
        P{1}.mCADplot(.75,'r');
%         A{2}.mCADcolor([0 1 0]);
        A{2}.mCADplot;
%         A{3}.mCADcolor([0 0 1]);
        A{3}.mCADplot;

        triangle = plot3([P{1}.pPos.X(1) A{2}.pPos.X(1) A{3}.pPos.X(1) P{1}.pPos.X(1)],...
            [P{1}.pPos.X(2) A{2}.pPos.X(2) A{3}.pPos.X(2) P{1}.pPos.X(2)],...
            [P{1}.pPos.X(3) A{2}.pPos.X(3) A{3}.pPos.X(3) P{1}.pPos.X(3)], '-b','LineWidth',1.5);

        triangle_d = plot3([P{1}.pPos.Xd(1) A{2}.pPos.Xd(1) A{3}.pPos.Xd(1) P{1}.pPos.Xd(1)],...
            [P{1}.pPos.Xd(2) A{2}.pPos.Xd(2) A{3}.pPos.Xd(2) P{1}.pPos.Xd(2)],...
            [P{1}.pPos.Xd(3) A{2}.pPos.Xd(3) A{3}.pPos.Xd(3) P{1}.pPos.Xd(3)], '--m','LineWidth',1.5);
      
        % Percourse made
        fig1 = plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
        fig2 = plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
        fig3 = plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
        fig4 = plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
        fig5 = plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);

        drawnow;
        
    end 
end

axis equal;
set(gca,'Box','on');

%% 

% x y z
figure()
subplot(311)
plot(data(1:end-50,133),data(1:end-50,106));
axis([0 Tf -.1 .1])
grid on
title('Xtil')

subplot(312)
plot(data(1:end-50,133),data(1:end-50,107));
axis([0 Tf -.1 .1])
grid on
title('Ytil')

subplot(313)
plot(data(1:end-50,133),data(1:end-50,108));
axis([0 Tf -.1 .1])
grid on
title('Ztil')

% p q beta
figure()
subplot(311)
plot(data(1:end-50,133),data(1:end-50,109));
axis([0 Tf -.1 .1])
grid on
title('ptil')

subplot(312)
plot(data(1:end-50,133),data(1:end-50,110));
axis([0 Tf -.1 .1])
grid on
title('qtil')

subplot(313)
plot(data(1:end-50,133),180/pi*data(1:end-50,111));
axis([0 Tf -40 40])
grid on
title('betatil')

% phi theta psi
figure()
subplot(311)
plot(data(1:end-50,133),180/pi*data(1:end-50,112));
axis([0 Tf -40 40])
grid on
title('phitil')

subplot(312)
plot(data(1:end-50,133),180/pi*data(1:end-50,113));
axis([0 Tf -40 40])
grid on
title('thetatil')

subplot(313)
plot(data(1:end-50,133),180/pi*data(1:end-50,114));
axis([0 Tf -40 40])
grid on
title('psitil')

