%% 3D Line Formation Pioneer-Drone
% Position task using a 3D line virtual structure to control the formation
%
%  In this script, the ArDrone is commanded to land during the
%  experiment. Later, its take off again to continue the task until the
%  end.
%
% I.   Pioneer is the reference of the formation
% II.  The formation variables are:
%      Q = [xf yf zf rhof alfaf betaf]
%  *the formation variables are specified in formation class file (LineFormation3D.m)

% Close communication with raspberry (for pioneer)
clear; close all; clc;
try
    fclose(instrfindall);
catch
end

try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.146');
    B = Bebop(1,'B1');
    
    %P = Pioneer3DX(1);  % Pioneer Instance
    
    % Joystick
    J = JoyControl;
    
    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    idB = getID(OPT,B); % ID do Bebop
    

    
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

%% Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
% addpath(genpath(pwd));

%% Load Classes
% Robot
P = Pioneer3DX(1);
% pgains = [1.5 1 1.5 1]; % controller gains
% pgains = [0.3 0.3 0.45 0.45]; % controller gains
pgains = [0.1 0.1 1];         % gains (for extended controller)
P.pPar.a = 0.15;                 % control point (for extended controller) 

% Drone
B = Bebop(1,'B1');   
B.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ]';
B.pPar.Ts = 1/30;

% 3D Formation 
LF = LineFormation3D;
% Trajectory formation gains
LF.pPar.K1 = 1*diag([0.2 0.2 0.2 3 1 1.8]);        % kinematic control gain  - controls amplitude
LF.pPar.K2 = 1*diag([1 1 1 1 1 1]);        % kinematic control gain - control saturation

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network (Pioneer-Raspberry)
% Setup Ros Master, Ros Matlab environment variables 
% Tutorial para rodar o pioneer com raspberry (2020)
% 1. Conectar o raspberry ao serial do pioneer (e obviamente alimentá-lo corretamente);
% 2. No PC, abrir o terminal PuTTY; 
% 3. No terminal, login e senha: ubuntu
% 4. Digitar o comando: "rosrun rosaria RosAria 192.168.0.148"  e pressionar ENTER
% 5. No programa, utilizar as linhas de código adiante para conectar ao pioneer

setenv('ROS_IP','192.168.0.158')                        %Ip do desktop
setenv('ROS_HOSTNAME','192.168.0.158')                  %Host name = ip
setenv('ROS_MASTER_URI','http://192.168.0.146:11311/')  %Ip do master (raspberry)
% rosinit                                                 %iniciar a comunicação

[pub,cmd_vel] = rospublisher('/RosAria/cmd_vel','geometry_msgs/Twist');

%% Open file to save data
NomeArq = datestr(now,30);
% cd('DataFiles')
% cd('Log_Optitrack')
Arq = fopen(['FL3d_PositionUtimateBebop' NomeArq '.txt'],'w');
% cd(PastaAtual)
dados = [];


%% Robots initial pose
% detect rigid body ID from optitrack
idP = getID(OPT,P);          % pioneer ID on optitrack
idB = getID(OPT,B);          % drone ID on optitrack

% Variables to calculate velocities inside optitrack function
P.pPar.ti = tic;             % pioneer

rb = OPT.RigidBody;          % read optitrack data
B = getOptData(rb(idB),B);   % get ardrone data
P = getOptData(rb(idP),P);   % get pioneer data

B.rTakeOff;
pause(3);                     % time for drone stabilization
disp('READY?? GO! GO! GO!');

%% Variable initialization
% Saves data to plot
data = [];
% Desired positions vector [xf yf zf rho alpha beta]
% Qd = [  0    0    0   1.5     0     pi/2;
%         1.5  0.5  0   2     -pi/2     pi/6;
%        -1.5  0.5  0   2     -pi/2     pi/6;
%        -1.5 -0.5  0   2     pi/2     pi/6;
%         0    0    0   1.5     0     pi/2];

Qd = [  1.2  0     0   1.5    0     pi/2;      % funcionou legal (marcos 13/01/2020)
        0    1.5    0   1.5    0    pi/2;
       -1.2  1.2   0   1      0     pi/2;
       -1.2  0     0   1.5    0     pi/2;
       -1  -1.3    0   1.5     0     pi/2 
        1.2  -1.2  0   1.5     0     pi/2 
        0    0     0    1     0      pi/2 ];
%       1   -0.5    0   1       0     pi/2;
%       -0.5  0      0   1     0     pi/2];
% 
% Qd = [  1.2  0     0   1.5    pi    pi/4;     
%         0    1.5     0   1.5  0    pi/2;
%        -1.2  1.5   0   1      0     pi/2;
%        -1.2  -0    0   1.5    0     pi/2;
%        -1  -1.3    0   1.5     0     pi/2 
%         1.2  -1.2    0   1.5     0     pi/2 
%         0    0     0    1     0      pi/2 ];
% %       1   -0.5    0   1       0     pi/2;
% %       -0.5  0      0   1     0     pi/2];

% 
% Qd = [ 0   1   0   1.5     pi/2     pi/2;   % altitude test
%       0   -1   0   1       pi/2     pi/2;
%       0   0   0   0.50    pi/2     pi/2
%       0   0   0   1.5      pi     pi/3];

cont = 1;     % counter to change desired position through simulation
time = 10;    % time to change desired positions [s]
% First desired position
LF.pPos.Qd = Qd(1,:)';
% Robots desired pose
LF.mInvTrans;

%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];

% Formation initial pose
LF.mDirTrans;

% Formation Error
LF.mFormationError;

%% Simulation
% Time variables initialization

WantedFlag = 0;

tsim    = size(Qd,1)*(time);
t       = tic;
tc      = tic;
tp      = tic;
B.pSC.Kinematics_control = 1;

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);



while toc(t)< tsim
    
    %% Desired positions
    if toc(tc) > 1/30
        tc = tic;
        %% Change positions

            if toc(t)> cont*time  
                cont = cont + 1;
            end
                    

        
        %% Acquire sensors data        
        % Get optitrack data        
        rb = OPT.RigidBody;             % read optitrack
        % Ardrone
        B  = getOptData(rb(idB),B);     
        % Pioneer
        P = getOptData(rb(idP),P);
        
        %% Control
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired positions ...........................................
        LF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position / tem que integrar o dXr da formação para obter o Xd do robô
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);       % desired velocities
        
        % Drone
        B.pPos.Xr(7:9) = LF.pPos.dXr(4:6);
        B.pPos.Xd(6) = 0;   %yaw desejado
        B.pPos.Xd(12) = 0;  %vyaw desejado  
        
        % Dynamic compensation ................................................
        % Drone
        B.cInverseDynamicController_Compensador;        
        B = J.mControl(B);   
        % Pioneer
%         P = fKinematicControllerExtended(P);        % new controller (by timotiu 2020)
        P = fKinematicControllerExtended(P,pgains);        % new controller (by timotiu 2020)
    
        %% Save data (.txt file)
        % Data file structure
        % (1:12)      (13:24)    (25:26)   (27:28)  ...     
        % P.pPos.Xd   P.pPos.X   P.pSC.Ud  P.pSC.U  ...
        %
        % (29:40)     (41:52)    (53:56)   (57:60)  ...     
        % B.pPos.Xd   B.pPos.X   B.pSC.Ud  B.pSC.U  ...
        %
        % (61:66)     (67:72)    (73)
        % LF.pPos.Qd  B.pPos.Q   Simulation time
               
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
       
        dados(end+1,:) =  [P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)];
       
        
     
        
        %% Send control signals 
        % to Pioneer (by raspberry)
        cmd_vel.Linear.X = P.pSC.Ud(1);
        cmd_vel.Angular.Z = P.pSC.Ud(2);       
        send(pub,cmd_vel)
        
        % to bebop
        B.rCommand;
        
                
        if cont <= size(Qd,1)     % prevent vector index error          
            LF.pPos.Qd = Qd(cont,:)';
        end
        
        % Land and take off
        if toc(t) > 5*tsim/size(Qd,1)         % Taking off
            
                B.rTakeOff;                   % takeoff again
     
        elseif toc(t) > 3*tsim/size(Qd,1)-6    % Landing
                
                B.rCmdStop;      
                B.rLand;   % flying state flag (1=flying/0=landed)
                B.pFlag.EmergencyStop = 0;
                WantedFlag = 1;
                
        else
                WantedFlag = 0;
        end
        
        

        
        
        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        drawnow
        if btnEmergencia ~= 0 || (WantedFlag == 0 && B.pFlag.EmergencyStop ~= 0) || B.pFlag.isTracked ~= 1 
            disp('Bebop Landing through Emergency Command ');
            
            % Send 3 times Commands 1 second delay to Drone Land
            for i=1:nLandMsg
                disp("End Land Command");
                B.rCmdStop;
                B.rLand;
                P.pSC.Ud = [0;0];
                cmd_vel.Linear.X = P.pSC.Ud(1);
                cmd_vel.Angular.Z = P.pSC.Ud(2);
                send(pub,cmd_vel)

            end
            break;
        end
    
%         % Exit loop if stop button is pressed
%         if J.pFlag == 1
%            break 
%         end

        
    end
    
    
end

% %% Loop Aterrissagem
% tsim = toc(t);
% % tland = 5;
% tland = 0;
% while toc(t) < tsim+tland
%     if toc(tc) > 1/30
%         tc = tic;
%         %% Position
%         
%         ta = toc(t);
%         % Positions
%         LF.pPos.Qd(1)  = LF.pPos.Q(1);         % x position
%         LF.pPos.Qd(2)  = LF.pPos.Q(2);         % y position
%         LF.pPos.Qd(4)  = Qd(end,4) - (Qd(end,4)-0.5)*(ta-tsim)/(tland);                   % rho position
%         % Velocities
%         LF.pPos.dQd(1)  = 0;                  % x velocities
%         LF.pPos.dQd(2)  = 0;                  % y velocities
%         LF.pPos.dQd(3)  = 0;                  % z velocities
%         LF.pPos.dQd(4)  = 0;                  % rho velocities
%         LF.pPos.dQd(5)  = 0;                  % alpha velocities
%         LF.pPos.dQd(6)  = 0;                  % beta velocities
%         
%         %% Acquire sensors data
%         % Get network data
% %         Rede.mReceiveMsg;
% %         if length(Rede.pMSG.getFrom)>1
% %             P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);  % current velocities (robot sensors)
% %             PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
% %         end
%                 
% 
%         % Get optitrack data
%         P.pPar.ti = tic;
%         A.pPar.ti = tic;
%         rb = OPT.RigidBody;             % read optitrack
%         % Ardrone
%         A = getOptData(rb(idA),A);
% %         B.pSC.U = [B.pPos.X(4);B.pPos.X(5);B.pPos.X(9);B.pPos.X(12)]; % populates actual control signal to save data
%         P = getOptData(rb(idP),P);
%         
%         %% Control
%         % Formation Members Position
%         LF.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];
%         
%         % Formation Controller
%         LF.mFormationControl;
%         
%         % Desired position ...........................................
%         LF.mInvTrans;
%         % Pioneer
%         P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position
%         P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);       % desired velocities
%         
%         sInvKinematicModel(P,P.pPos.Xd(7:8));  % sem essa conversão o compensador não tem o valor de pSC.Ur  (por isso o pioneer estava ficando parado, Valentim)
% 
%         % Drone
% %         B.pPos.Xda     = B.pPos.Xd;              % save previous posture
%         B.pPos.Xd(1:3) = LF.pPos.Xd(4:6);    % desired position
%         B.pPos.Xd(7:9) = LF.pPos.dXr(4:6);   % desired velocities
%         B.pPos.Xd(6)   = 0; %P.pPos.X(6); % atan2(B.pPos.Xd(8),B.pPos.Xd(7)); % desired Psi
%         
%         
%         B.pPos.Xd(12) = 0; %(B.pPos.Xd(6) - B.pPos.Xda(6))/(1/30);
%         
%         % ............................................................
%         % Dynamic controllers
%          B.pSC.Kinematics_control = 1; % flag para evitar o compensador dinamico
%          A.pPar.Xr(7:9) = LF.pPos.dXr(4:6);%passando a referencia para o drone
%          A = cInverseDynamicController_Compensador_ArDrone(A);
%          A = J.mControl(A);                            % joystick command (priority)
% %         P = fDynamicController(P);                  % Pioneer Dynamic controller
% %         P = fCompensadorDinamico(P);
% %         P = fControladorCinematico(P);
%          P = fControladorCinematico(P,pgains);
% %         %
% %         A = cUnderActuatedController(A,gains);  % ArDrone
% %         A = J.mControl(A);                      % joystick command (priority)
% % %         P = fDynamicController(P);              % Pioneer Dynamic controller
% %         P = fCompensadorDinamico(P);
% %         
%         %% Save data (.txt file)
%         fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
%             B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
%         fprintf(Arq,'\n\r');
%        dados(end+1,:) =  [P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
%             B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)];
%         %% Send control signals to robots
%         P.pSC.Ud = [0; 0]; % stop Pioneer
%         cmd_vel.Linear.X = P.pSC.Ud(1);
%         cmd_vel.Angular.Z = P.pSC.Ud(2);
%         send(pub,cmd_vel)
% %         Rede.mSendMsg(P);  % send data to network
%         A.rSendControlSignals;
%     end
%     
% end


%% Close files
fclose(Arq);
%%  Stop robot
% Send control signals
P.pSC.Ud = [0;0];
cmd_vel.Linear.X = P.pSC.Ud(1);
cmd_vel.Angular.Z = P.pSC.Ud(2);
send(pub,cmd_vel)

% Land drone
for i=1:5
    disp("Stop Command");
    B.rCmdStop;
    B.rLand;
    P.pSC.Ud = [0;0];
    cmd_vel.Linear.X = P.pSC.Ud(1);
    cmd_vel.Angular.Z = P.pSC.Ud(2);
    send(pub,cmd_vel)
end

 % Close ROS Interface
RI.rDisconnect;
rosshutdown;

%% Plot results
figure
subplot(6,1,1)
plot(dados(:,end),(dados(:,60+(1))-dados(:,66+(1))))
grid on
title('X')
subplot(6,1,2)
plot(dados(:,end),(dados(:,60+(2))-dados(:,66+(2))))
grid on
title('Y')
subplot(6,1,3)
plot(dados(:,end),(dados(:,60+(3))-dados(:,66+(3))))
grid on
title('Z')
subplot(6,1,4)
plot(dados(:,end),(dados(:,60+(4))-dados(:,66+(4))))
grid on
title('rho')
subplot(6,1,5)
plot(dados(:,end),(dados(:,60+(5))-dados(:,66+(5))))
grid on
title('alpha')
subplot(6,1,6)
plot(dados(:,end),(dados(:,60+(6))-dados(:,66+(6))))
grid on
title('beta')

figure
subplot(6,1,1)
plot(dados(:,end),dados(:,60+(1)))
hold on
plot(dados(:,end),dados(:,66+(1)))
title('X')
subplot(6,1,2)
plot(dados(:,end),dados(:,60+(2)))
hold on
plot(dados(:,end),dados(:,66+(2)))
title('Y')
subplot(6,1,3)
plot(dados(:,end),dados(:,60+(3)))
hold on
plot(dados(:,end),dados(:,66+(3)))
title('Z')
subplot(6,1,4)
plot(dados(:,end),dados(:,60+(4)))
hold on
plot(dados(:,end),dados(:,66+(4)))
title('rho')
subplot(6,1,5)
plot(dados(:,end),dados(:,60+(5)))
hold on
plot(dados(:,end),dados(:,66+(5)))
title('alpha')
subplot(6,1,6)
plot(dados(:,end),dados(:,60+(6)))
hold on
plot(dados(:,end),dados(:,66+(6)))
title('beta')

% figure
% subplot(3,1,1)
% plot(dados(:,end),dados(:,28+(1)))
% hold on
% plot(dados(:,end),dados(:,40+(1)))
% title('X')
% subplot(3,1,2)
% plot(dados(:,end),dados(:,28+(2)))
% hold on
% plot(dados(:,end),dados(:,40+(2)))
% title('Y')
% subplot(3,1,3)
% plot(dados(:,end),dados(:,28+(3)))
% hold on
% plot(dados(:,end),dados(:,40+(3)))
% title('Z')
% 
% figure
% subplot(3,1,1)
% plot(dados(:,end),dados(:,40+(7)))
% title('Velocidade X')
% subplot(3,1,2)
% plot(dados(:,end),dados(:,40+(8)))
% title('Velocidade Y')
% subplot(3,1,3)
% plot(dados(:,end),dados(:,40+(9)))
% title('Velocidade Z')


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
