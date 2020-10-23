%% Trajectory Controller for Parrot Bebop 2

% Valentim Ernandes Neto
% Date: jun-20-2019

% Work developed to Journal of Intelligent & Robotic Systems (JINT), with 
% a special section on Unmanned Systems. 

% ISSN: 0921-0296 (Print) 1573-0409 (Online)
% Publisher: Springer Netherlands
% Impact Factor: 2.020

% ----------------------------------------------------------------------- %

% Initial Comands
clear; 
close all; 
clc;

try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
addpath(genpath(pwd));

%% Load Classes
try
    
    % Load Classes
    
    % ROS
    RI = RosInterface;
    RI.rConnect('192.168.0.144');
    
    r = ROSNetwork; % ROS with windows for Pioneer
    r.InitROS('/master');
    r.InitROS('/P1','192.168.0.144');
    r.PublisherROS(r.node,'P1/vel');
    r.SubscriberROS(r.node,'P1/pose');

    r.PublisherROS(r.node,'P1/pose');
    r.PublisherROS(r.node,'P1/chatter');

    r.SendROS('P1/vel','[0 1 2 3]');            % Beboop



    % Bebop
    B = Bebop(2,'B2'); 
    
    % Pioneer
    P = Pioneer3DX(1);
    P.pPar.Ts = 0.100;
    
    % Line Formation 3D
    LF3D = LineFormationControl;

    % Joystick
    J = JoyControl;
    
    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    fprintf('\n\n------------------------   Load Class Success   ------------------------\n');
    
catch EM
    
    fprintf('\n\n-------------------------   Load Class Issue   -------------------------\n');
    disp(EM);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

%% Emergency settings

% Emmergency button
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'Emergency!', ...
    'Callback', 'btnEmergencia = 1;', ...
    'Position', [50 50 400 300]);

nLandMsg = 3; % Number of redundant landing messages.

%% Variable initialization

% -----------------------    Pioneer   ----------------------------
% Detect rigid body ID from optitrack
rb = OPT.RigidBody;            % read optitrack data
idP = getID(OPT,P);         % Pioneer ID on optitrack
P = getOptData(rb(idP),P);  % Get Pioneer data
% Gains
Pgains = [ 0.35  0.35  0.75  0.75  0.75  0.75  0.10  0.05 ];

% -------------------------   Bebop   -----------------------------
% Estimated model
B.pPar.Model_simp = [  0.8417   ,  0.18227  ,    0.8354   ,  0.17095  ...
                       3.9660   ,  4.00100  ,    9.8524   ,  4.7295   ]';
% Take off Bebop
fprintf('\n\n (1) Start take off timming....');
% B.rTakeOff;
% pause(5);
fprintf('\n\n (2) Take off ends....');

% -----------------------   Formation   ---------------------------
% Gains
LF3D.pPar.K1 = diag([   2.0  ,   2.0  ,   2.0  ... % Saturation
                        1.0  ,   1.0  ,   1.0  ]);     
LF3D.pPar.K2 = diag([   0.1  ,   0.1  ,   0.1  ... % Inclination
                        0.1  ,   0.1  ,   0.1  ]);     
% Formation initial position and shape
LF3D.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];
LF3D.pPos.Xr = LF3D.pPos.X; % Initial position and shape references
LF3D.mDirTrans; % Direct Transform
% Shape reference
rho = 1.5;
alpha = pi/4;
beta = pi/4;

% ------------------------   Others   -----------------------------
% Time variables 
Tcontrol = 0.250; % 200 ms de Amostragem | 5 Hz de Frequência
Tsim = 50; % Simulation time

% Data variables
% data = zeros(round(Tsim/Tcontrol),100); % Data matrix
% data = zeros(round(Tsim/Tcontrol),191);
% inc = 0;

data = [];

% Trajectory variables
%       (1) 'Lemniscate - Horizontal'
%       (2) 'Lemniscate - Horizontal with double parameterization in time'
%       (3) 'Lemniscate - Inclined'
%       (4) 'Lemniscate - Vertical ( XZ plan | Y = cte )'
%       (5) 'Lemniscate - Vertical ( YZ plan | X = cte )'
type = 'Lemniscate - Horizontal with double parameterization in time';
rX = 1;     % X-axis radius [m]
rY = 1;     % Y-axis radius [m]
rZ = 0;     % Z-axis radius [m]
T = 40;     % Period [s]
TF = T*2;   % Final time for two turns [s]
w = 2*pi/T; % Angular frequency[rad/s]

fprintf('\n\n (3) Starting timers....');
% pause(5);

% Timers start
t  = tic;               % Simulation loop timer
t_control = tic;        % General control loop timer
B.pPar.ti = tic;        % Bebop control loop timer
dt_psi = tic;

fprintf('\n\n (4) Simulation running....\n');
fprintf('\n         ...  ...  ...  ...  ...  ...  ...  ...  ...\n\n');


rX = 0.75;           % [m]
rY = 1.0;           % [m]
rho = 0.75;         % [m]
rhoArDrone = 1.5;   % [m]
T = 30;             % [s]
Tf = 60;            % [s]
w = 2*pi/T;         % [rad/s]

TrajConst = [  rX   rY   rZ  ...
               T    TF   w   ];

TimeVars = [  toc(t_control)    toc(t) ];
[ Xd , dXd , ddXd ] = funcTrajectoryPlanner(TimeVars,TrajConst,type);


%% Main Loop
try
    while toc(t) < Tsim
        
        if toc(t_control) > Tcontrol
            %% Updating variables and trajectory planner
            TrajConst = [  rX   rY   rZ  ...
                           T    TF   w   ];

            TimeVars = [  toc(t_control)    toc(t) ];
            
            toc(t_control)
            t_control = tic; % Reset general control loop timer
            
            % Trajectory Planner
%             [ Xd , dXd , ddXd ] = funcTrajectoryPlanner(TimeVars,TrajConst,type);
            
            t_traj = toc(t);
            a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
            tp = a*Tf; 
            Xd(1) = rX*sin(w*tp);
            Xd(2) = rY*sin(2*w*tp);
            dXd(1) = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);
            dXd(2) = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);

            % Getting Pioneer pose data from Optitrack
            rb = OPT.RigidBody;            % read optitrack data
            idP = getID(OPT,P);            % Pioneer ID on optitrack
            P = getOptData(rb(idP),P);  % Get Pioneer data
            
            % Getting Bebop pose data from Optitrack
            B.rGetSensorDataOpt;
            
            % Updating formation position and shape
            LF3D.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];
            LF3D.mDirTrans; % Direct Transform

            % Updating Formation reference variables.
            LF3D.pPos.Qd(1) = Xd(1);    % xF
            LF3D.pPos.Qd(2) = Xd(2);    % yF
            LF3D.pPos.Qd(3) = Xd(3);    % zF

            LF3D.pPos.Qd(4) = rho;      % rho
            LF3D.pPos.Qd(5) = alpha;    % alpha (+frente/-trás)
            LF3D.pPos.Qd(6) = beta;     % beta  (lateral  -dir/+ esq)

            LF3D.pPos.dQd(1) = dXd(1);  % dxF
            LF3D.pPos.dQd(2) = dXd(1);  % dyF
            LF3D.pPos.dQd(3) = dXd(1);  % dzF
            
            LF3D.mInvTrans;                                 % Transformada Inversa Qd --> Xd
            
            %% Control
            % Formation Controller      
            LF3D.mFormationControl_NullSpace('P',TimeVars(1)); % (S) Shape
           
            % Pioneer
            % Updating robots references based on formation controller.
            P.pPos.Xda = P.pPos.Xd;                    % Save previous desired position
            P.pPos.Xd(1:2) = LF3D.pPos.Xr(1:2);        % Reference position
            P.pPos.Xd(7:8) = LF3D.pPos.dXr(1:2);       % Reeference velocity
            % Trajectory controller
            P = fDynamicController(P,Pgains);
            P.pPar.Ts = TimeVars(1);

            % Pioneer
            % Updating robots references based on formation controller.
            P.pPos.Xda = P.pPos.Xd;                    % Save previous desired position
            P.pPos.Xd(1:2) = Xd(1:2);        % Reference position
            P.pPos.Xd(7:8) = dXd(1:2);       % Reeference velocity
            % Trajectory controller
            P = fDynamicController(P,Pgains);
            P.pPar.Ts = TimeVars(1);

            % Bebop
            % Updating robots references based on formation controller.
            B.pPos.Xda = B.pPos.Xd;                       % Save previous desired position
            B.pPos.Xd(1:3) = LF3D.pPos.Xr(4:6);           
            B.pPos.Xd(7:9) = LF3D.pPos.dXr(4:6);
            B.pPos.dXd(7:9) = (B.pPos.Xd(7:9) - B.pPos.Xda(7:9))/TimeVars(1);
            % Trajectory controller.
%             B.cInverseDynamicController_Compensador;
            B.cInverseDynamicController_Adaptativo;
            B.pPar.ti = tic;
            
            %% Save data
            
            % Variable to feed plotResults function
%             data = [  data  ; B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
%                       toc(t) B.pPar.Model_simp'];
            data(end+1,:) = [B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
                      toc(t) B.pPar.Model_simp'];
%          
%         inc = inc + 1;       
%         data(inc,:) = [   P.pPos.Xd'     P.pPos.X'        P.pSC.Ud(1:2)'       P.pSC.U(1:2)' ...
%                           B.pPos.Xd'        B.pPos.X'           B.pSC.Ud'            B.pSC.U' ...
%                           LF3D.pPos.Qd'    LF3D.pPos.Q'       LF3D.pPos.Qtil'     LF3D.pPos.Xd' ...
%                           toc(t)];
            
            % %         %   1 -- 12      13 -- 24     25 -- 26          27 -- 28
            % %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
            % %
            % %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60
            % %             B.pPos.Xd'   B.pPos.X'    B.pSC.Ud'         B.pSC.U'
            % %
            % %         %   61 -- 66     67 -- 72       73 -- 78       79
            % %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];
            
            %% Sending the control signals to robots
            % Pioneer
%             r.SendROS('P1/vel',P.pSC.Ud);            % Beboop
            r.SendROS('P1/vel','[0 1 2 3]');            % Beboop

            
            % Joystick Command Priority
            B = J.mControl(B);                    % joystick command (priority)
            B.rCommand;
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
                fprintf('\n----------------------------   Emergency   -----------------------------\n');
                fprintf('\n   ----> Bebop Landing through Emergency Command\n');
                fprintf('\n   ----> Pioneer Stopping through Emergency Command\n');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg     
                    fprintf(['\n   ----> Land Command (' num2str(i) ')\n']);
                    B.rCmdStop;
                    B.rLand;
                    
                    fprintf(['\n   ----> Pioneer Stop Command (' num2str(i) ')\n']);
                    P.pSC.Ud = [0  ;  0];
                    Rede.mSendMsg(P);    
                end
                break;
            end
            
            
        end
    end
catch EM
    
    fprintf('\n----------------------------   Code Error   ----------------------------\n');
    fprintf('\n   ----> Bebop Landing through Try/Catch Loop Command\n');
    B.rCmdStop;
    disp('');
    disp(EM);
    disp('');
    B.rLand
    
    fprintf('\n   ----> Pioneer Stopping through Try/Catch Loop Command\n');
    P.pSC.Ud = [0  ;  0];
    Rede.mSendMsg(P); 
    
end

% Send 3 times Commands 1 second delay to Drone Land
fprintf('\n-----------------------------   The End   ------------------------------\n');
for i=1:nLandMsg
    
    fprintf(['\n   ----> Land Command (' num2str(i) ')\n']);
    B.rCmdStop;
    B.rLand
    
    fprintf(['\n   ----> Pioneer Stop Command (' num2str(i) ')\n']);
    P.pSC.Ud = [0  ;  0];
    Rede.mSendMsg(P);    
    
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

fprintf('\n   ----> Shutdown completed...\n\n');

%% Plot results
Xtil = data(:,1:12) - data(:,13:24);

figure();
hold on;
grid on;
plot(data(:,35),Xtil(:,1));
plot(data(:,35),Xtil(:,2));
plot(data(:,35),Xtil(:,3));
plot(data(:,35),Xtil(:,6));
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Time [s]');
ylabel('Pose error [m] | [rad]');

figure();
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
xlabel('X-axis [m]');
ylabel('Y-axis [m]');

figure();
subplot(411)
hold on;
grid on;
plot(data(:,35),data(:,19));
plot(data(:,35),data(:,7));
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('dX', 'dXd');

subplot(412)
hold on;
grid on;
plot(data(:,35),data(:,20));
plot(data(:,35),data(:,8));
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('dY', 'dYd');

subplot(413)
hold on;
grid on;
plot(data(:,35),data(:,21));
plot(data(:,35),data(:,9));
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('dZ', 'dZd');

subplot(414)
hold on;
grid on;
plot(data(:,35),data(:,24));
plot(data(:,35),data(:,12));
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
legend('dphi', 'dPhid');


figure();
subplot(311)
hold on;
grid on;
plot(data(:,35),data(:,13));
plot(data(:,35),data(:,1));
xlabel('Time [s]');
ylabel('Xtilde [m]');
legend('X', 'Xd');

subplot(312)
hold on;
grid on;
plot(data(:,35),data(:,14));
plot(data(:,35),data(:,2));
xlabel('Time [s]');
ylabel('Ytilde [m]');
legend('Y', 'Yd');

subplot(313)
hold on;
grid on;
plot(data(:,35),data(:,15));
plot(data(:,35),data(:,3));
axis([0,70,1,2])
xlabel('Time [s]');
ylabel('Ztilde [m]');
legend('Z', 'Zd');

subplot(421)
hold on;
grid on;
plot(data(:,35),data(:,36));
xlabel('Time [s]');
ylabel('$K_1$','interpreter','latex')
legend(['$K_1 = $' num2str(data(end,36))],'interpreter','latex')

subplot(422)
hold on;
grid on;
plot(data(:,35),data(:,37));
xlabel('Time [s]');
ylabel('$K_2$','interpreter','latex')
legend(['$K_2 = $' num2str(data(end,37))],'interpreter','latex')

subplot(423)
hold on;
grid on;
plot(data(:,35),data(:,38));
xlabel('Time [s]');
ylabel('$K_3$','interpreter','latex')
legend(['$K_3 = $' num2str(data(end,38))],'interpreter','latex')

subplot(424)
hold on;
grid on;
plot(data(:,35),data(:,39));
xlabel('Time [s]');
ylabel('$K_4$','interpreter','latex')
legend(['$K_4 = $' num2str(data(end,39))],'interpreter','latex')

subplot(425)
hold on;
grid on;
plot(data(:,35),data(:,40));
xlabel('Time [s]');
ylabel('$K_5$','interpreter','latex')
legend(['$K_5 = $' num2str(data(end,40))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,35),data(:,41));
xlabel('Time [s]');
ylabel('$K_6$','interpreter','latex')
legend(['$K_6 = $' num2str(data(end,41))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,35),data(:,42));
xlabel('Time [s]');
ylabel('$K_7$','interpreter','latex')
legend(['$K_7 = $' num2str(data(end,42))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,35),data(:,43));
xlabel('Time [s]');
ylabel('$K_8$','interpreter','latex')
legend(['$K_8 = $' num2str(data(end,43))],'interpreter','latex')
