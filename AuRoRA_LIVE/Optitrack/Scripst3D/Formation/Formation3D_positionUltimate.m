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

clear; close all; clc;

try
    fclose(instrfindall);
catch
end

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
addpath(genpath(pwd));

%% Load Classes
% Robot
P = Pioneer3DX(1);
% P.pPar.a = 0.1;
A = ArDrone(30);
A.pPar.Ts = 1/30;
% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
% A.pPar.Cgains = [.5 2 .5 2 5 2; 1 20 1 15 1 2.5]; % underActuatedController gains

% gains = [1 5 1 5 2 15;  % ficou oscilando mas foi
%     10 3 8 3 1 5];

% gains = [1 5 1 5 2 15;  6.5 2 6.5 2 1 5];
gains = [0.5 3 0.6 3 2 15; 10 3 8 3 1 4];

% Formation 3D
LF = LineFormation3D;
% Ganho do controlador de trajetória
LF.pPar.K1 = 1*diag([1 1 1 1.5 0.75 0.6]);    % kinematic control gain  - controls amplitude
LF.pPar.K2 = 1*diag([0.2 0.2 0.5 0.82 0.6 1.57]);        % kinematic control gain - control saturation

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_Optitrack')
Arq = fopen(['FL3d_PositionUtimateExp' NomeArq '.txt'],'w');
cd(PastaAtual)

% Teste método do Igor
% DroneVar = CreateDataFile(A,P,'Exp','Log_Optitrack');
% OptData = CreateOptDataFile(A,P,'30Hz','Log_Optitrack');

%% Network communication check
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 1
        if isempty(Rede.pMSG.getFrom{2})
            Rede.mSendMsg(P);
            
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
            
        else
            break
        end
    end
end
clc
disp('Data received. Continuing program...');

%% Robot/Simulator conection
A.rConnect;

%% Robots initial pose
% detect rigid body ID from optitrack
idP = getID(OPT,P);         % pioneer ID on optitrack
idA = getID(OPT,A);         % drone ID on optitrack

rb = OPT.RigidBody;          % read optitrack data
A = getOptData(rb(idA),A);   % get ardrone data
P = getOptData(rb(idP),P);   % get pioneer data

A.rTakeOff;
pause(5);                     % time to drone stabilize
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

Qd = [  1  0      0   1     0     pi/2;
    0    1      0   1       0     pi/2;
    -1  0.5    0   0.5     0     pi/2;
    -1  -1     0   1       0     pi/2;
    0    -1     0   1       0     pi/2;
    1   -0.5    0   1       0     pi/2;
    -0.5  0      0   1     0     pi/2];


% Qd = [ 0   0   0   1.8     0     pi/2;   % altitude test
%        0   0   0   0.75    0     pi/2;
%        0   0   0   0.50    0     pi/2];
%
cont = 0;     % counter to change desired position through simulation
time = 20;    % time to change desired positions [s]
% First desired position
LF.pPos.Qd = Qd(1,:)';
% Robots desired pose
LF.mInvTrans;
%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
LF.mDirTrans;

% Formation Error
LF.mFormationError;

%% Simulation
% Time variables initialization
tsim    = size(Qd,1)*(time);
t       = tic;
tc      = tic;
tp      = tic;

while toc(t)< tsim
    
    %% Desired positions
    if toc(tc) > 1/30
        tc = tic;
        %% Change positions
        if toc(t)> cont*time
            cont = cont + 1;
        end
        if cont <= size(Qd,1)
            LF.pPos.Qd = Qd(cont,:)';
        end
        
        %% Land and take off
        if toc(t) > 5*tsim/size(Qd,1)         % Taking off
            if A.pCom.cStatus(32)==0          % flying state flag (1=flying/0=landed)
                A.rTakeOff;                   % takeoff again
            end
        elseif toc(t) > 3*tsim/size(Qd,1)-1    % Landing
            if A.pCom.cStatus(32)==1           % flying state flag (1=flying/0=landed)
                A.rLand;
            end
        end
        
        %% Acquire sensors data
        % Get network data
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);  % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        % Ardrone
        A       = getOptData(rb(idA),A);
        A.pSC.U = [A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]; % populates actual control signal to save data
        % Pioneer
        P = getOptData(rb(idP),P);
        
        %% Control
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired positions ...........................................
        LF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position / tem que integrar o dXr da formação para obter o Xd do robô
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);       % desired velocities
        
        sInvKinematicModel(P,P.pPos.Xd(7:8));  % sem essa conversão o compensador não tem o valor de pSC.Ur  (por isso o pioneer estava ficando parado, Valentim)

        % Drone
        A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        
        % Dynamic compensation ................................................
%         A = cUnderActuatedControllerMexido(A,gains);  % ArDrone
        A = cInverseDynamicController_Compensador_ArDrone;
        A = J.mControl(A);                            % joystick command (priority)
%         P = fDynamicController(P);                  % Pioneer Dynamic controller
        P = fCompensadorDinamico(P);
        
        %
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');     
        
        % Send control signals to robots
        Rede.mSendMsg(P);       % send Pioneer data to network
        
        % Send Ardrone control signals (only if in flying state)        
        A.rSendControlSignals;
        
    end
    
    
end

%% Loop Aterrissagem
tsim = toc(t);
tland = 5;
while toc(t) < tsim+tland
    if toc(tc) > 1/30
        tc = tic;
        %% Trajectory
        % Lemniscata (8')
        ta = toc(t);
        % Positions
        LF.pPos.Qd(1)  = LF.pPos.Q(1);         % x position
        LF.pPos.Qd(2)  = LF.pPos.Q(2);         % y position
        LF.pPos.Qd(4)  = Qd(end,4) - (Qd(end,4)-0.5)*(ta-tsim)/(tland);                   % rho position
        % Velocities
        LF.pPos.dQd(1)  = 0;                  % x velocities
        LF.pPos.dQd(2)  = 0;                  % y velocities
        LF.pPos.dQd(3)  = 0;                  % z velocities
        LF.pPos.dQd(4)  = 0;                  % rho velocities
        LF.pPos.dQd(5)  = 0;                  % alpha velocities
        LF.pPos.dQd(6)  = 0;                  % beta velocities
        
        %% Acquire sensors data
        % Get network data
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);  % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        % Ardrone
        A = getOptData(rb(idA),A);
        A.pSC.U = [A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]; % populates actual control signal to save data
        P = getOptData(rb(idP),P);
        
        %% Control
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired position ...........................................
        LF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);       % desired velocities
        
        sInvKinematicModel(P,P.pPos.Xd(7:8));  % sem essa conversão o compensador não tem o valor de pSC.Ur  (por isso o pioneer estava ficando parado, Valentim)

        % Drone
        A.pPos.Xda     = A.pPos.Xd;              % save previous posture
        A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);    % desired position
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);   % desired velocities
        A.pPos.Xd(6)   = 0; %P.pPos.X(6); % atan2(A.pPos.Xd(8),A.pPos.Xd(7)); % desired Psi
        
        
        A.pPos.Xd(12) = 0; %(A.pPos.Xd(6) - A.pPos.Xda(6))/(1/30);
        
        % ............................................................
        % Dynamic controllers
        A = cUnderActuatedController(A,gains);  % ArDrone
        A = J.mControl(A);                      % joystick command (priority)
%         P = fDynamicController(P);              % Pioneer Dynamic controller
        P = fCompensadorDinamico(P);
        
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
        
        %% Send control signals to robots
        P.pSC.Ud = [0; 0]; % stop Pioneer
        Rede.mSendMsg(P);  % send data to network
        A.rSendControlSignals;
    end
    
end


%% Close files
fclose(Arq);
%%  Stop robot
% Send control signals
% Send to network (a few times to be sure)
P.pSC.Ud = [0;0];
for ii = 1:5
    Rede.mSendMsg(P);
end
% Land drone
if A.pFlag.Connected == 1
    A.rLand;
end

%
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
