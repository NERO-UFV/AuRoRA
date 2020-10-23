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

%% Load Class
try
    % Load Classes
    RI = RosInterface; % 
    setenv('ROS_IP','192.168.0.158') % IP do computador que está rodando o código principal
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311') % IP do computador que está rodando o mestre
    RI.rConnect('192.168.0.146'); % Conectar ao mestre
    
    B{1} = Bebop(1,'B1');
%     B{2} = Bebop(2,'B2');
%     B{3} = Bebop(3,'B3');
    
    % Joystick
    J = JoyControl;
    
    disp('  -------------------  Load Class Success  -------------------');
    
catch ME
    disp(' ');
    disp('  -------------------  Load Class Issues  -------------------');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

% Botão de Emergencia
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% detect rigid body ID from optitrack
idB{1} = getID(OPT,B{1},1);    % drone ID on optitrack
% idB{2} = getID(OPT,B{2},2);    % drone ID on optitrack
rb = OPT.RigidBody;          % read optitrack data

% Take Off
disp('Start Take Off Timming....');
B{1}.rTakeOff;
% B{2}.rTakeOff;
pause(2);
disp('Taking Off End Time....');


%% Creating the virtual robots
A{1} = ArDrone(1);
A{2} = ArDrone(2);
A{3} = ArDrone(3);

% Formation 3D
F{1} = TriangularFormation3D;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Robot initial pose
B{1} = getOptData(rb(idB{1}),B{1});
% B{2} = getOptData(rb(idB{2}),B{2});

% ArDrone
A{2}.pPos.X(1:3) = [ -1  0  0 ];
A{2}.pPos.X(6) = 0;

A{3}.pPos.X(1:3) = [ 0 1  0 ];
A{3}.pPos.X(6) = 0;

F{1}.pPos.Xr = [ B{1}.pPos.X(1:3) ; A{2}.pPos.X(1:3) ; A{3}.pPos.X(1:3) ];

Ts = F{1}.pPar.Ts;
F{1}.mTrajectoryPlanner_TF3D(0);

%% Configure simulation window

fig = figure();
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

triangle = plot3([B{1}.pPos.X(1) A{2}.pPos.X(1) A{3}.pPos.X(1) B{1}.pPos.X(1)],...
    [B{1}.pPos.X(2) A{2}.pPos.X(2) A{3}.pPos.X(2) B{1}.pPos.X(2)],...
    [B{1}.pPos.X(3) A{2}.pPos.X(3) A{3}.pPos.X(3) B{1}.pPos.X(3)], '-b','LineWidth',1.5);

set(gca,'Box','on');

drawnow;

%% Simulation
fprintf('\nStart..............\n\n');

% Time variables initialization
T_PLOT = 1;       % Período de plotagem em tempo real

T_FORMATION = F{1}.pPar.Ts; % 150ms
T_BEBOP = B{1}.pPar.Ts; %1/30

T = 30;   % [s]

% Data variables
kk = 1;
data = zeros(round(T/T_FORMATION),133); % Data matrix

t  = tic;
t_plot = tic;

t_Formation = tic;      % Formation control

t_Bebop_1 = tic;
t_Bebop_2 = tic;
t_ArDrone_3 = tic;

try
    while toc(t)< T
        % =====================================================================
        % Laço de controle dos robôs
        % ArDrone

        if toc(t_Bebop_1) > T_BEBOP
            t_Bebop_1 = tic;
            rb = OPT.RigidBody; % read optitrack data
            B{1} = getOptData(rb(idB{1}),B{1});   % Adquirir dados dos sensores - ArDrone
            B{1}.pSC.Kinematics_control = 1;
            B{1}.cInverseDynamicController_Compensador;  % Controlador 
%             B{1}.pSC.Ud = [  0   0   0   0   0   0  ]';
            B{1} = J.mControl(B{1});
            B{1}.rCommand;               % Enviar sinal de controle para o robô
        end

        if toc(t_Bebop_2) > T_BEBOP
            
            t_ArDrone_2 = tic;
            A{2}.pPos.X(1:3) = F{1}.pPos.Xr(4:6);
%             t_Bebop_2 = tic;
%             rb = OPT.RigidBody; % read optitrack data
%             B{2} = getOptData(rb(idB{2}),B{2});   % Adquirir dados dos sensores - ArDrone
%             B{2}.pSC.Kinematics_control = 1;
%             B{2}.cInverseDynamicController_Compensador;  % Controlador 
% %             B{2}.pSC.Ud = [  0   0   0   0   0   0  ]';
%             B{2} = J.mControl(B{2});
%             B{2}.rCommand;               % Enviar sinal de controle para o robô
        end

        if toc(t_ArDrone_3) > T_BEBOP
            t_ArDrone_3 = tic;
            A{3}.pPos.X(1:3) = F{1}.pPos.Xr(7:9);
        end

        % =====================================================================
        % Laço de controle de formação

        % Trajectory WEST VILLAGE Planner 2020
        if toc(t_Formation) > T_FORMATION
            t_Formation = tic;

            % Get sensor data
            rb = OPT.RigidBody;          % read optitrack data
            B{1} = getOptData(rb(idB{1}),B{1});
%             B{2} = getOptData(rb(idB{2}),B{2});
            
            F{1}.mTrajectoryPlanner_TF3D(toc(t));

            F{1}.pPos.X = [ B{1}.pPos.X(1:3); A{2}.pPos.X(1:3); A{3}.pPos.X(1:3) ];
            
%             F{1}.cNoPriority_TF3D;
            
            F{1}.cNullSpaceBased_TF3D('PSO');
            
            % Atribuindo posições desejadas       
            % B1
            B{1}.pPos.Xda = B{1}.pPos.Xd;   % save previous posture
            B{1}.pPos.Xd(1:3) = F{1}.pPos.Xr(1:3);
            B{1}.pPos.Xr(7:9) = F{1}.pPos.dXr(1:3);

            % A2
            A{2}.pPos.Xda = A{2}.pPos.Xd;   % save previous posture
            A{2}.pPos.Xd(1:3) = F{1}.pPos.Xr(4:6);
            A{2}.pPos.Xd(7:9) = F{1}.pPos.dXr(4:6);
            
            % A3
            A{3}.pPos.Xda = A{3}.pPos.Xd;   % save previous posture
            A{3}.pPos.Xd(1:3) = F{1}.pPos.Xr(7:9);
            A{3}.pPos.Xd(7:9) = F{1}.pPos.dXr(7:9);

            % Variable to feed plotResults function    
            data(kk,:) = [  B{1}.pPos.Xd'     B{1}.pPos.X'        B{1}.pSC.Ud([1 2 3 6])'       B{1}.pSC.U' ...
                            A{2}.pPos.Xd'     A{2}.pPos.X'        A{2}.pSC.Ud'                  A{2}.pSC.U' ...
                            A{3}.pPos.Xd'     A{3}.pPos.X'        A{3}.pSC.Ud'                  A{3}.pSC.U' ...
                            F{1}.pPos.Qd'     F{1}.pPos.Qtil'     F{1}.pPos.Q'                  F{1}.pPos.Xr'...
                            toc(t)];

                            kk = kk + 1;

            % %         %   1 -- 12             13 -- 24             25 -- 28            29 -- 32
            % %             B{1}.pPos.Xd'       B{1}.pPos.X'         B{1}.pSC.Ud'        B{1}.pSC.U'
            % %         
            % %         %   33 -- 44            45 -- 56             57 -- 60            61 -- 64 
            % %             B{2}.pPos.Xd'       B{2}.pPos.X'         B{2}.pSC.Ud'        B{2}.pSC.U'
            % %         
            % %         %   65 -- 76            77 -- 88             89 -- 92            93 -- 96
            % %             A{3}.pPos.Xd'       A{3}.pPos.X'         A{3}.pSC.Ud'        A{3}.pSC.U'
            % %        
            % %         %   97 -- 105           106 -- 114           115 -- 123         124 -- 132 
            % %             F{1}.pPos.Qd'       F{1}.pPos.Qtil'      F{1}.pPos.Q'       F{1}.pPos.Xr'
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

            triangle = plot3([B{1}.pPos.X(1) A{2}.pPos.X(1) A{3}.pPos.X(1) B{1}.pPos.X(1)],...
                [B{1}.pPos.X(2) A{2}.pPos.X(2) A{3}.pPos.X(2) B{1}.pPos.X(2)],...
                [B{1}.pPos.X(3) A{2}.pPos.X(3) A{3}.pPos.X(3) B{1}.pPos.X(3)], '-b','LineWidth',1.5);

%             triangle_d = plot3([F{1}.pPos.Xr(1) B{2}.pPos.Xd(1) A{3}.pPos.Xd(1) F{1}.pPos.Xr(1)],...
%                 [F{1}.pPos.Xr(2) B{2}.pPos.Xd(2) B{3}.pPos.Xd(2) F{1}.pPos.Xr(2)],...
%                 [F{1}.pPos.Xr(3) B{2}.pPos.Xd(3) B{3}.pPos.Xd(3) F{1}.pPos.Xr(3)], '--m','LineWidth',1.5);

            % Percourse made
            fig1 = plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
            fig2 = plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
            fig3 = plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
            fig4 = plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
            fig5 = plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);

            drawnow;           
        end
        
        B{1}.pFlag.EmergencyStop = 0;
        
        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        if btnEmergencia ~= 0 || J.pFlag == 1 || ...
                B{1}.pFlag.EmergencyStop ~= 0 || B{1}.pFlag.isTracked ~= 1

            disp('Bebop Landing through Emergency Command ');
            B{1}.rCmdStop;
            B{1}.rLand;
            
            break;
        end
            
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B{1}.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    
end

axis equal;
set(gca,'Box','on');

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B{1}.rCmdStop;
    B{1}.rLand
    pause(1)
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%% Plot

            % %         %   1 -- 12             13 -- 24             25 -- 28            29 -- 32
            % %             B{1}.pPos.Xd'       B{1}.pPos.X'         B{1}.pSC.Ud'        B{1}.pSC.U'
            % %         
            % %         %   33 -- 44            45 -- 56             57 -- 60            61 -- 64 
            % %             B{2}.pPos.Xd'       B{2}.pPos.X'         B{2}.pSC.Ud'        B{2}.pSC.U'
            % %         
            % %         %   65 -- 76            77 -- 88             89 -- 92            93 -- 96
            % %             A{3}.pPos.Xd'       A{3}.pPos.X'         A{3}.pSC.Ud'        A{3}.pSC.U'
            % %        
            % %         %   97 -- 105           106 -- 114           115 -- 123         124 -- 132 
            % %             F{1}.pPos.Qd'       F{1}.pPos.Qtil'      F{1}.pPos.Q'       F{1}.pPos.Xr'
            % %
            % %         %   133 
            % %         %   toc(t)  ]
% x y z
figure()
subplot(311)
plot(data(1:end-50,133),data(1:end-50,106));
axis([0 T -.1 .1])
grid on
title('Xtil')

subplot(312)
plot(data(1:end-50,133),data(1:end-50,107));
axis([0 T -.1 .1])
grid on
title('Ytil')

subplot(313)
plot(data(1:end-50,133),data(1:end-50,108));
axis([0 T -.1 .1])
grid on
title('Ztil')

% p q beta
figure()
subplot(311)
plot(data(1:end-50,133),data(1:end-50,109));
axis([0 T -.1 .1])
grid on
title('ptil')

subplot(312)
plot(data(1:end-50,133),data(1:end-50,110));
axis([0 T -.1 .1])
grid on
title('qtil')

subplot(313)
plot(data(1:end-50,133),180/pi*data(1:end-50,111));
axis([0 T -40 40])
grid on
title('betatil')

% phi theta psi
figure()
subplot(311)
plot(data(1:end-50,133),180/pi*data(1:end-50,112));
axis([0 T -40 40])
grid on
title('phitil')

subplot(312)
plot(data(1:end-50,133),180/pi*data(1:end-50,113));
axis([0 T -40 40])
grid on
title('thetatil')

subplot(313)
plot(data(1:end-50,133),180/pi*data(1:end-50,114));
axis([0 T -40 40])
grid on
title('psitil')
