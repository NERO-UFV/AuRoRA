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
    setenv('ROS_IP','192.168.0.147') % IP do computador que está rodando o código principal
    setenv('ROS_MASTER_URI','http://192.168.0.101:11311') % IP do computador que está rodando o mestre
    RI.rConnect('192.168.0.101'); % Conectar ao mestre
    
    B{1} = Bebop(1,'B1');
    B{2} = Bebop(2,'B2');
    B{3} = Bebop(3,'B3');
    
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


% Take Off
disp('Start Take Off Timming....');
B{1}.rTakeOff;
B{2}.rTakeOff;
B{3}.rTakeOff;
pause(1);
disp('Taking Off End Time....');


%% Creating the virtual robots

% Formation 3D
F{1} = TriangularFormation3D;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Robot initial pose
B{1}.rGetSensorDataLocal;
B{2}.rGetSensorDataLocal;
B{3}.rGetSensorDataLocal;

F{1}.pPos.Xr = [ B{1}.pPos.X(1:3) ; B{2}.pPos.X(1:3) ; B{3}.pPos.X(1:3) ];

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

triangle = plot3([B{1}.pPos.X(1) B{2}.pPos.X(1) B{3}.pPos.X(1) B{1}.pPos.X(1)],...
    [B{1}.pPos.X(2) B{2}.pPos.X(2) B{3}.pPos.X(2) B{1}.pPos.X(2)],...
    [B{1}.pPos.X(3) B{2}.pPos.X(3) B{3}.pPos.X(3) B{1}.pPos.X(3)], '-b','LineWidth',1.5);

set(gca,'Box','on');

drawnow;

gains = [  2     2    3     1.5 ...
           2     2    3     5 ...
           1     1     1     1.5 ...
           1     1     1     1];


%% Simulation
fprintf('\nStart..............\n\n');

% Time variables initialization
T_PLOT = 29.5;       % Período de plotagem em tempo real

T_FORMATION = F{1}.pPar.Ts; % 150ms

B{1}.pPar.Ts = .100;
B{2}.pPar.Ts = .100;
B{3}.pPar.Ts = .100;

T_BEBOP = B{1}.pPar.Ts; %1/30

T = 30;   % [s]

% Data variables
kk = 1;
data = zeros(round(T/T_FORMATION),142); % Data matrix

t  = tic;
t_plot = tic;

t_Formation = tic;      % Formation control

t_Bebop_1 = tic;
t_Bebop_2 = tic;
t_Bebop_3 = tic;

% try
    while toc(t)< T
        % =====================================================================
        % Laço de controle dos robôs
        % ArDrone

        if toc(t_Bebop_1) > T_BEBOP
            t_Bebop_1 = tic;
            B{1}.rGetSensorDataLocal;
            B{1}.pSC.Kinematics_control = 1;
            B{1}.cInverseDynamicController_Compensador(gains);  % Controlador 
%             B{1}.pSC.Ud = [  0   0   0   0   0   0  ]';
            B{1} = J.mControl(B{1});
            B{1}.rCommand;               % Enviar sinal de controle para o robô
        end

        if toc(t_Bebop_2) > T_BEBOP
            t_Bebop_2 = tic;
            B{2}.rGetSensorDataLocal;
            B{2}.pSC.Kinematics_control = 1;
            B{2}.cInverseDynamicController_Compensador(gains);  % Controlador 
%             B{2}.pSC.Ud = [  0   0   0   0   0   0  ]';
            B{2} = J.mControl(B{2});
            B{2}.rCommand;               % Enviar sinal de controle para o robô
        end

        if toc(t_Bebop_3) > T_BEBOP
            t_Bebop_3 = tic;
            B{3}.rGetSensorDataLocal;
            B{3}.pSC.Kinematics_control = 1;
            B{3}.cInverseDynamicController_Compensador(gains);  % Controlador 
%             B{3}.pSC.Ud = [  0   0   0   0   0   0  ]';
            B{3} = J.mControl(B{3});
            B{3}.rCommand;               % Enviar sinal de controle para o robô
        end

        % =====================================================================
        % Laço de controle de formação

        % Trajectory WEST VILLAGE Planner 2020
        if toc(t_Formation) > T_FORMATION
            t_Formation = tic;

            % Get sensor data
            B{1}.rGetSensorDataLocal;
            B{2}.rGetSensorDataLocal;
            B{3}.rGetSensorDataLocal;
            
            F{1}.mTrajectoryPlanner_TF3D(toc(t));

            F{1}.pPos.X = [ B{1}.pPos.X(1:3); B{2}.pPos.X(1:3); B{3}.pPos.X(1:3) ];
            
%             F{1}.cNoPriority_TF3D;
            
            F{1}.cNullSpaceBased_TF3D('OSP');
            
            % Atribuindo posições desejadas       
            % B1
            B{1}.pPos.Xda = B{1}.pPos.Xd;   % save previous posture
            B{1}.pPos.Xd(1:3) = F{1}.pPos.Xr(1:3);
            B{1}.pPos.Xr(7:9) = F{1}.pPos.dXr(1:3);

            % A2
            B{2}.pPos.Xda = B{2}.pPos.Xd;   % save previous posture
            B{2}.pPos.Xd(1:3) = F{1}.pPos.Xr(4:6);
            B{2}.pPos.Xr(7:9) = F{1}.pPos.dXr(4:6);
            
            % A3
            B{3}.pPos.Xda = B{3}.pPos.Xd;   % save previous posture
            B{3}.pPos.Xd(1:3) = F{1}.pPos.Xr(7:9);
            B{3}.pPos.Xr(7:9) = F{1}.pPos.dXr(7:9);

            % Variable to feed plotResults function    
            data(kk,:) = [  B{1}.pPos.Xd'     B{1}.pPos.X'        B{1}.pSC.Ud([1 2 3 6])'       B{1}.pSC.U' ...
                            B{2}.pPos.Xd'     B{2}.pPos.X'        B{2}.pSC.Ud([1 2 3 6])'       B{2}.pSC.U' ...
                            B{3}.pPos.Xd'     B{3}.pPos.X'        B{3}.pSC.Ud([1 2 3 6])'       B{3}.pSC.U' ...
                            F{1}.pPos.Qd'     F{1}.pPos.Qtil'     F{1}.pPos.Q'                  F{1}.pPos.Xr'      F{1}.pPos.Xd'...
                            toc(t)];

                            kk = kk + 1;

            % %         %   1 -- 12             13 -- 24             25 -- 28            29 -- 32
            % %             B{1}.pPos.Xd'       B{1}.pPos.X'         B{1}.pSC.Ud'        B{1}.pSC.U'
            % %         
            % %         %   33 -- 44            45 -- 56             57 -- 60            61 -- 64 
            % %             B{2}.pPos.Xd'       B{2}.pPos.X'         B{2}.pSC.Ud'        B{2}.pSC.U'
            % %         
            % %         %   65 -- 76            77 -- 88             89 -- 92            93 -- 96
            % %             B{3}.pPos.Xd'       B{3}.pPos.X'         B{3}.pSC.Ud'        B{3}.pSC.U'
            % %        
            % %         %   97 -- 105           106 -- 114           115 -- 123         124 -- 132          132 -- 141
            % %             F{1}.pPos.Qd'       F{1}.pPos.Qtil'      F{1}.pPos.Q'       F{1}.pPos.Xr'       F{1}.pPos.Xd'
            % %
            % %         %   142 
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

            triangle = plot3([B{1}.pPos.X(1) B{2}.pPos.X(1) B{3}.pPos.X(1) B{1}.pPos.X(1)],...
                [B{1}.pPos.X(2) B{2}.pPos.X(2) B{3}.pPos.X(2) B{1}.pPos.X(2)],...
                [B{1}.pPos.X(3) B{2}.pPos.X(3) B{3}.pPos.X(3) B{1}.pPos.X(3)], '-b','LineWidth',1.5);

%             triangle_d = plot3([F{1}.pPos.Xd(1) B{2}.pPos.Xd(1) A{3}.pPos.Xd(1) F{1}.pPos.Xd(1)],...
%                 [F{1}.pPos.Xr(2) B{2}.pPos.Xd(2) B{3}.pPos.Xd(2) F{1}.pPos.Xd(2)],...
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
        B{2}.pFlag.EmergencyStop = 0;
        B{3}.pFlag.EmergencyStop = 0;

        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        if btnEmergencia ~= 0 || J.pFlag == 1 || ...
                B{1}.pFlag.EmergencyStop ~= 0 || B{1}.pFlag.isTracked ~= 1 || ...
                B{2}.pFlag.EmergencyStop ~= 0 || B{2}.pFlag.isTracked ~= 1 || ...
                B{3}.pFlag.EmergencyStop ~= 0 || B{3}.pFlag.isTracked ~= 1
            
            disp('Bebop Landing through Emergency Command ');
            B{1}.rCmdStop;
            B{1}.rLand;
            B{2}.rCmdStop;
            B{2}.rLand;
            B{3}.rCmdStop;
            B{3}.rLand;
            break;
        end
            
    end
% catch ME
%     
%     disp('Bebop Landing through Try/Catch Loop Command');
%     B{1}.rCmdStop;
%     B{2}.rCmdStop;
%     B{3}.rCmdStop;
%       
%     disp('');
%     disp(ME);
%     disp('');
%     
% end

axis equal;
set(gca,'Box','on');

%% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B{1}.rCmdStop;
    B{1}.rLand
    B{2}.rCmdStop;
    B{2}.rLand
    B{3}.rCmdStop;
    B{3}.rLand
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
            % %             B{3}.pPos.Xd'       B{3}.pPos.X'         B{3}.pSC.Ud'        B{3}.pSC.U'
            % %        
            % %         %   97 -- 105           106 -- 114           115 -- 123         124 -- 132 
            % %             F{1}.pPos.Qd'       F{1}.pPos.Qtil'      F{1}.pPos.Q'       F{1}.pPos.Xr'
            % %
            % %         %   133 
            % %         %   toc(t)  ]

% Vista 3D
figure()
plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
hold on;
plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);
plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Perspective view (3D)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(-30,30);   

% Vista Superior (x-y)
figure()
plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
hold on;
plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);
plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Top view (XY-plane)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(0,90); 

% triangle = plot3([B{1}.pPos.X(1) B{2}.pPos.X(1) B{3}.pPos.X(1) B{1}.pPos.X(1)],...
%     [B{1}.pPos.X(2) B{2}.pPos.X(2) B{3}.pPos.X(2) B{1}.pPos.X(2)],...
%     [B{1}.pPos.X(3) B{2}.pPos.X(3) B{3}.pPos.X(3) B{1}.pPos.X(3)], '-b','LineWidth',1.5);

% Vista Lateral (x-z)
figure()
plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
hold on;
plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);
plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Side view (XZ-plane)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(0,0); 

% Vista Lateral (y-z)
figure()
plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
hold on;
plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);
plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Side view (YZ-plane)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(-90,0); 

% x y z

last = length(data)-5;
figure()
subplot(311)
plot(data(1:last,142),data(1:last,106),'LineWidth',1.5);
title('Position','FontSize',15,'interpreter','Latex')
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{x}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,142),data(1:last,107),'LineWidth',1.5);
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{y}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,142),data(1:last,108),'LineWidth',1.5);
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{z}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')


% p q beta
figure()
subplot(311)
plot(data(1:last,142),data(1:last,109),'LineWidth',1.5);
title('Shape','FontSize',15,'interpreter','Latex')
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{p}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,142),data(1:last,110),'LineWidth',1.5);
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{q}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,142),180/pi*data(1:last,111),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\beta}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')

% phi theta psi
figure()
subplot(311)
plot(data(1:last,142),180/pi*data(1:last,112),'LineWidth',1.5);
title('Orientation','FontSize',15,'interpreter','Latex')
axis([0 T -40 40])
grid on
ylabel('$\tilde{\phi}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,142),180/pi*data(1:last,113),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\theta}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,142),180/pi*data(1:last,114),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\psi}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')