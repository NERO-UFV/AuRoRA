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
    B{2} = Bebop(2,'B2');
    B{3} = Bebop(3,'B3');
    A = ArDrone(1);

    
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
idB{2} = getID(OPT,B{2},2);    % drone ID on optitrack
idB{3} = getID(OPT,B{3},3);    % drone ID on optitrack

idB{4} = getID(OPT,ArDrone);    % drone ID on optitrack


% Take Off
disp('Start Take Off Timming....');
B{1}.rTakeOff;
B{2}.rTakeOff;
B{3}.rTakeOff;
pause(3);
disp('Taking Off End Time....');


%% Creating the virtual robots

% Formation 3D
F{1} = TriangularFormation3D;

% Robot initial pose
rb = OPT.RigidBody;          % read optitrack data
B{1} = getOptData(rb(idB{1}),B{1});
B{2} = getOptData(rb(idB{2}),B{2});
B{3} = getOptData(rb(idB{3}),B{3});
A = getOptData(rb(idB{4}),A);


F{1}.pPos.Xr = [ B{1}.pPos.X(1:3) ; B{2}.pPos.X(1:3) ; B{3}.pPos.X(1:3) ];

Ts = F{1}.pPar.Ts;
F{1}.mTrajectoryPlanner_TF3D(0);

% Bebop Controller Gains

gains = [  2     2     3     1.5     ...
           2     2     4     5     ...
           1     1     1     1.5     ...
           1     1     1     1        ];


%% Simulation
fprintf('\nStart..............\n\n');

% Time variables initialization

F{1}.pPar.Ts = 0.050;

B{1}.pPar.Ts = 0.030;
B{2}.pPar.Ts = 0.030;
B{3}.pPar.Ts = 0.030;

T_FORMATION = F{1}.pPar.Ts; % 100ms
T_BEBOP = B{1}.pPar.Ts; %1/30

T = 40;   % [s]

% Data variables
kk = 1;
data = zeros(round(T/T_FORMATION),154); % Data matrix

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
            rb = OPT.RigidBody; % read optitrack data
            B{1} = getOptData(rb(idB{1}),B{1});   % Adquirir dados dos sensores - ArDrone
            B{1}.pSC.Kinematics_control = 1;
            B{1}.cInverseDynamicController_Compensador(gains);  % Controlador 
%             B{1}.pSC.Ud = [  0   0   0   0   0   0  ]';
            B{1} = J.mControl(B{1});
            B{1}.rCommand;               % Enviar sinal de controle para o robô
        end

        if toc(t_Bebop_2) > T_BEBOP
            t_Bebop_2 = tic;
            rb = OPT.RigidBody; % read optitrack data
            B{2} = getOptData(rb(idB{2}),B{2});   % Adquirir dados dos sensores - ArDrone
            B{2}.pSC.Kinematics_control = 1;
            B{2}.cInverseDynamicController_Compensador(gains);  % Controlador 
%             B{2}.pSC.Ud = [  0   0   0   0   0   0  ]';
            B{2} = J.mControl(B{2});
            B{2}.rCommand;               % Enviar sinal de controle para o robô
        end

        if toc(t_Bebop_3) > T_BEBOP
            t_Bebop_3 = tic;
            rb = OPT.RigidBody; % read optitrack data
            B{3} = getOptData(rb(idB{3}),B{3});   % Adquirir dados dos sensores - ArDrone
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
            rb = OPT.RigidBody;          % read optitrack data
            B{1} = getOptData(rb(idB{1}),B{1});
            B{2} = getOptData(rb(idB{2}),B{2});
            B{3} = getOptData(rb(idB{3}),B{3});
            A = getOptData(rb(idB{4}),A);

            F{1}.mTrajectoryPlanner_TF3D(toc(t));

            F{1}.pPos.X = [ B{1}.pPos.X(1:3); B{2}.pPos.X(1:3); B{3}.pPos.X(1:3) ];
                      
            F{1}.cNullSpaceBased_TF3D('N');
            
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
                            A.pPos.X'      toc(t)];

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
        
        drawnow;
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

% Vista 3D
Sc = 1;
figure()
plot3(Sc*data(2:kk-1,13),Sc*data(2:kk-1,14),0.1+data(2:kk-1,15),'r-','LineWidth',1.0);
hold on;
plot3(Sc*data(2:kk-1,45),Sc*data(2:kk-1,46),0.1+data(2:kk-1,47),'g-','LineWidth',1.0);
plot3(Sc*data(2:kk-1,77),Sc*data(2:kk-1,78),0.1+data(2:kk-1,79),'b-','LineWidth',1.0);
plot3(data(2:kk-1,115),data(2:kk-1,116),0.1+data(2:kk-1,117),'k-','LineWidth',1.0);
plot3(data(2:kk-1,97),data(2:kk-1,98),0.1+data(2:kk-1,99),'m--','LineWidth',1.5);

grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$');
% lgX = legend('$X$','$X_d$','Load');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'FontSize',12,'Interpreter','latex');
title('Perspective view (3D)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(-30,30);   

%% Vista Superior (x-y)
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

%% Erros
% x y z
Atil = data(:,142:144) - data(:,97:99);
Atil(:,3) = Atil(:,3) + 1.25;

last = length(data)-45;
figure()
subplot(311)
plot(data(1:last,154),NO_LOAD(1:last,106),'LineWidth',1.5);
hold on;
plot(data(1:last,154),data(1:last,106),'LineWidth',1.5);
plot(data(1:last,154),Atil(1:last,1),'LineWidth',1.5);
lgX = legend('F without a load','F carrying a load','Load');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Position','FontSize',15,'interpreter','Latex')
axis([0 T -.25 .25])
grid on
ylabel('$\tilde{x}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,154),NO_LOAD(1:last,107),'LineWidth',1.5);
hold on;
plot(data(1:last,154),data(1:last,107),'LineWidth',1.5);
plot(data(1:last,154),Atil(1:last,2),'LineWidth',1.5);
axis([0 T -.25 .25])
grid on
ylabel('$\tilde{y}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,154),NO_LOAD(1:last,108),'LineWidth',1.5);
hold on;
plot(data(1:last,154),data(1:last,108),'LineWidth',1.5);
plot(data(1:last,154),Atil(1:last,2),'LineWidth',1.5);
axis([0 T -.25 .25])
grid on
ylabel('$\tilde{z}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')

% p q beta
figure()
subplot(311)
plot(data(1:last,154),NO_LOAD(1:last,109),'LineWidth',1.5);
hold on
plot(data(1:last,154),data(1:last,109),'LineWidth',1.5);
plot(data(1:last,154),zeros(last,1),'LineWidth',1.5);
lgX = legend('F without a load','F carrying a load','Load');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Shape','FontSize',15,'interpreter','Latex')
axis([0 T -.25 .25])
grid on
ylabel('$\tilde{p}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,154),NO_LOAD(1:last,110),'LineWidth',1.5);
hold on
plot(data(1:last,154),data(1:last,110),'LineWidth',1.5);
plot(data(1:last,154),zeros(last,1),'LineWidth',1.5);
axis([0 T -.25 .25])
grid on
ylabel('$\tilde{q}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,154),180/pi*NO_LOAD(1:last,111),'LineWidth',1.5);
hold on
plot(data(1:last,154),180/pi*data(1:last,111),'LineWidth',1.5);
plot(data(1:last,154),zeros(last,1),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\beta}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')


Atil = 180/pi*(data(:,145:147) - data(:,103:105));
for i = 1:length(Atil)
    if Atil(i,3) > 180
       Atil(i,3) = Atil(i,3) - 360;
    end
end
% phi theta psi
figure()
subplot(311)
plot(data(1:last,154),180/pi*NO_LOAD_O(1:last,112),'LineWidth',1.5);
hold on
plot(data(1:last,154),180/pi*data(1:last,112),'LineWidth',1.5);
plot(data(1:last,154),Atil(1:last,1),'LineWidth',1.5);
lgX = legend('F without a load','F carrying a load','Load');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Orientation','FontSize',15,'interpreter','Latex')
axis([0 T -40 40])
grid on
ylabel('$\tilde{\phi}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,154),180/pi*NO_LOAD_O(1:last,113),'LineWidth',1.5);
hold on
plot(data(1:last,154),180/pi*data(1:last,113),'LineWidth',1.5);
plot(data(1:last,154),Atil(1:last,2),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\theta}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,154),180/pi*NO_LOAD_O(1:last,114),'LineWidth',1.5);
hold on
plot(data(1:last,154),180/pi*data(1:last,114),'LineWidth',1.5);
plot(data(1:last,154),Atil(1:last,3),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\psi}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')

% phi theta psi real
figure()
subplot(311)
plot(data(1:last,154),180/pi*data(1:last,121),'LineWidth',1.5);
title('Orientation','FontSize',15,'interpreter','Latex')
axis([0 T -40 40])
grid on
ylabel('$\tilde{\phi}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,154),180/pi*data(1:last,122),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\theta}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,154),180/pi*data(1:last,123),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\psi}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')