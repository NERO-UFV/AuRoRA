%% Trajectory Controller for Pioneer3DX

% Valentim Ernandes Neto
% Date: jun-20-2019

% Work developed to Journal of Intelligent & Robotic Systems (JINT), with 
% a special section on Unmanned Systems. 

% ISSN: 0921-0296 (Print) 1573-0409 (Online)
% Publisher: Springer Netherlands
% Impact Factor: 2.020

% ----------------------------------------------------------------------- %

% Initial Comands

clear data
% clear; 
% close all; 
% clc;
% 
% try
%     fclose(instrfindall);
% catch
% end

% Look for root folder
% % PastaAtual = pwd;
% % PastaRaiz = 'AuRoRA 2018';
% % cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
% % addpath(genpath(pwd));
                                         
%% Load Class

try
    
    % ROS
    % Connect to a ROS network. Start global node at given IP and NodeName
    masterHost = '192.168.0.144';
    rosinit(masterHost, 'NodeHost','192.168.0.200',...
            'NodeName','/DELL_OPTITRACK');
    fprintf('\n');
    
    % Defining publishers and subscribers for all related topics
    ID = 1;
    P1 = robotics.ros.Node(['/P',num2str(ID)], masterHost);
    % Publishers
    topic_Ud = robotics.ros.Publisher(P1,['/P',num2str(ID),'/Ud'],'geometry_msgs/Twist');
    
catch EM
    disp(EM);
% %     fprintf('\n\nThe global ROS node is already initialized and connected to the master. \n\n');
% %     rosshutdown;
% %     fprintf('\n   ----> Shutdown completed...\n\n');
% % 
% %     fprintf('\n   ----> Trying to initializate global node again...\n\n');
% %     % ROS
% %     % Connect to a ROS network. Start global node at given IP and NodeName
% %     masterHost = '192.168.0.144';
% %     rosinit(masterHost, 'NodeHost','192.168.0.200',...
% %             'NodeName','/DELL_OPTITRACK');
% %     fprintf('\n');
   
end

try
    
    % Load Classes
    
    % Pioneer
    P = Pioneer3DX(1);
    P.pPar.Ts = 0.050;
    
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
Pgains = [ 0.15  0.15  0.75  0.75  0.75  0.75  0.10  0.05 ];

% -----------------------   Formation   ---------------------------
% Gains
LF3D.pPar.K1 = diag([   2.0  ,   2.0  ,   2.0  ... % Saturation
                        1.0  ,   1.0  ,   1.0  ]);     
LF3D.pPar.K2 = diag([   0.1  ,   0.1  ,   0.1  ... % Inclination
                        0.1  ,   0.1  ,   0.1  ]);     
% Formation initial position and shape
LF3D.pPos.X = [P.pPos.X(1:3); P.pPos.X(1:3)];
LF3D.pPos.Xr = LF3D.pPos.X; % Initial position and shape references
LF3D.mDirTrans; % Direct Transform
% Shape reference
rho = 1.5;
alpha = pi/4;
beta = pi/4;

% ------------------------   Others   -----------------------------
% Time variables 
Tcontrol = P.pPar.Ts; % 200 ms de Amostragem | 5 Hz de Frequência
Tsim = 100; % Simulation time

% Data variables
data = zeros(round(Tsim/Tcontrol),81); % Data matrix
inc = 0;

% Trajectory variables
%       (1) 'Lemniscate - Horizontal'
%       (2) 'Lemniscate - Horizontal with double parameterization in time'
%       (3) 'Lemniscate - Inclined'
%       (4) 'Lemniscate - Vertical ( XZ plan | Y = cte )'
%       (5) 'Lemniscate - Vertical ( YZ plan | X = cte )'
type = 'Lemniscate - Horizontal with double parameterization in time';
rX = 0.75;     % X-axis radius [m]
rY = 1;     % Y-axis radius [m]
rZ = 0;     % Z-axis radius [m]
T = 50;     % Period [s]
TF = T*2;   % Final time for two turns [s]
w = 2*pi/T; % Angular frequency[rad/s]

fprintf('\n\n (3) Starting timers....');
pause(5);

% Timers start
t  = tic;               % Simulation loop timer
t_control = tic;        % General control loop timer
dt_psi = tic;

fprintf('\n\n (4) Simulation running....\n');
fprintf('\n         ...  ...  ...  ...  ...  ...  ...  ...  ...\n\n');

try
    while toc(t) < Tsim
        
        if toc(t_control) > Tcontrol
            %% Updating variables and trajectory planner
            TrajConst = [  rX   rY   rZ  ...
                           T    TF   w   ];

            TimeVars = [  toc(t_control)    toc(t) ];
            
            t_control = tic; % Reset general control loop timer
            
            % Trajectory Planner
            [ Xd , dXd , ddXd ] = funcTrajectoryPlanner(TimeVars,TrajConst,type);
                                 
            % Getting Pioneer pose data from Optitrack
            rb = OPT.RigidBody;            % read optitrack data
            idP = getID(OPT,P);            % Pioneer ID on optitrack
            P = getOptData(rb(idP),P);  % Get Pioneer data
            
            % Updating formation position and shape
            LF3D.pPos.X = [P.pPos.X(1:3); [P.pPos.X(1:2) ; 1]];
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
            
            LF3D.mInvTrans;             % Transformada Inversa Qd --> Xd
            
            %% Control
            % Formation Controller      
            LF3D.mFormationControl_NullSpace('P',TimeVars(1)); % (S) Shape
           
            % Pioneer
            % Updating robots references based on formation controller.
% %             P.pPos.Xda = P.pPos.Xd;                    % Save previous desired position
% %             P.pPos.Xd(1:2) = LF3D.pPos.Xr(1:2);        % Reference position
% %             P.pPos.Xd(7:8) = LF3D.pPos.dXr(1:2);       % Reeference velocity
% %             % Trajectory controller
% %             P = fDynamicController(P,Pgains);
% %             P.pPar.Ts = TimeVars(1);

            % Pioneer
            % Updating robots references based on formation controller.
            P.pPos.Xda = P.pPos.Xd;                    % Save previous desired position
            P.pPos.Xd(1:2) = Xd(1:2);        % Reference position
            P.pPos.Xd(7:8) = dXd(1:2);       % Reeference velocity
            % Trajectory controller
            P = fDynamicController(P,Pgains);
            P.pPar.Ts = TimeVars(1);
            
            %% Save data
            
            % Variable to feed plotResults function
            inc = inc + 1;
            data(inc,:) = [   P.pPos.Xd'       P.pPos.X'        P.pSC.Ud(1:2)'       P.pSC.U(1:2)' ...
                              P.pPos.Xd'       P.pPos.X'        P.pSC.Ud(1:2)'       P.pSC.U(1:2)' ...
                              LF3D.pPos.Qd'    LF3D.pPos.Q'     LF3D.pPos.Qtil'      LF3D.pPos.Xd' ...
                              toc(t)];
            
            % %         %   1 -- 12      13 -- 24     25 -- 26          27 -- 28
            % %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
            % %
            % %         %   29 -- 40     41 -- 52     53 -- 54          55 -- 56
            % %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud'         P.pSC.U'
            % %
            % %         %   57 -- 62     63 -- 68       69 -- 74       75
            % %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];
            
            %% Sending the control signals to robots
            % Pioneer
            msg_Ud = rosmessage(topic_Ud);
            msg_Ud.Linear.X = P.pSC.Ud(1);
            msg_Ud.Angular.Z = P.pSC.Ud(2);
            send(topic_Ud,msg_Ud);
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0
                fprintf('\n----------------------------   Emergency   -----------------------------\n');
                fprintf('\n   ----> Bebop Landing through Emergency Command\n');
                fprintf('\n   ----> Pioneer Stopping through Emergency Command\n');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg     
                    fprintf(['\n   ----> Pioneer Stop Command (' num2str(i) ')\n']);
                    P.pSC.Ud = [0  ;  0];
                    msg_Ud = rosmessage(topic_Ud);
                    msg_Ud.Linear.X = P.pSC.Ud(1);
                    msg_Ud.Angular.Z = P.pSC.Ud(2);
                    send(topic_Ud,msg_Ud);
                end
                break;
            end
            
            
        end
    end
catch EM
    
    fprintf('\n----------------------------   Code Error   ----------------------------\n\n');
    disp(EM);
    fprintf('\n   ----> Pioneer Stopping through Try/Catch Loop Command\n');
    P.pSC.Ud = [0  ;  0];
    msg_Ud = rosmessage(topic_Ud);
    msg_Ud.Linear.X = P.pSC.Ud(1);
    msg_Ud.Angular.Z = P.pSC.Ud(2);
    send(topic_Ud,msg_Ud);
    
end

% Send 3 times Commands 1 second delay to Drone Land
fprintf('\n-----------------------------   The End   ------------------------------\n');
for i=1:nLandMsg
    
    fprintf(['\n   ----> Pioneer Stop Command (' num2str(i) ')\n']);
    P.pSC.Ud = [0  ;  0];
    msg_Ud = rosmessage(topic_Ud);
    msg_Ud.Linear.X = P.pSC.Ud(1);
    msg_Ud.Angular.Z = P.pSC.Ud(2);
    send(topic_Ud,msg_Ud);
    
end

%% Plot results
Xtil = data(:,1:12) - data(:,13:24);

% Position Error
figure();
hold on;
grid on;
plot(data(:,end),Xtil(:,1));
plot(data(:,end),Xtil(:,2));
plot(data(:,end),Xtil(:,6));
legend('Xtilde','Ytilde', 'Psitilde');
xlabel('Time [s]');
ylabel('Pose error [m] | [rad]');

% Position XY-plan
figure();
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
xlabel('X-axis [m]');
ylabel('Y-axis [m]');

% Velocities
% dX 
figure();
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,19));
plot(data(:,end),data(:,7));
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('dX', 'dXd');
% dY
subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,20));
plot(data(:,end),data(:,8));
axis([0 50 -1 1])
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('dY', 'dYd');
% dPsi
subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,24));
plot(data(:,end),data(:,12));
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
legend('dPsi', 'dPsid');

% Pose
% X
figure();
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,13));
plot(data(:,end),data(:,1));
xlabel('Time [s]');
ylabel('X [m]');
legend('X', 'Xd');
% Y
subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,14));
plot(data(:,end),data(:,2));
xlabel('Time [s]');
ylabel('Y [m]');
legend('Y', 'Yd');
% Psi
subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,18));
plot(data(:,end),data(:,6));
axis([0,70,1,2])
xlabel('Time [s]');
ylabel('Psi [m]');
legend('Psi', 'Psid');

