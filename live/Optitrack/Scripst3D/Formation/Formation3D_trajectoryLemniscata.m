%% 3D Line Formation Pioneer-Drone
% Trajectory task using a line virtual structure to control the formation
%
% I.   Pioneer is the reference of the formation
% II.  The formation variables are:
%      Q = [xf yf zf rhof alfaf betaf]
% pause(5)

clear
close all
clc

try
    fclose(instrfindall);
catch
end

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


%% Load Classes
% Robot
P = Pioneer3DX(1);  % load pioneer class
% P.pPar.a = 0.15;     % define new control point

A = ArDrone(2);     % load ardrone class
A.pPar.Ts = 1/30;   % redefine sample rate

% The Gains must be given in the folowing order
% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
% A.pPar.Cgains = [0.5 2 0.5 2 5 2; 1 20 1 15 1 2.5];
gains = [0.5 3 0.6 3 2 15;
    10 3 8 3 1 4];
% gains = [1.5 3.2 1.2 2 2 17;
%     2 15 2 15 1 5];



% Formation 3D
LF = LineFormation3D;
% LF.pPar.K1 = 1*diag([0.6 0.6 0.5 0.03 0.015 0.075]);    % kinematic control gain  - controls amplitude
% LF.pPar.K2 = 1*diag([0.5 0.5 0.5 0.8 0.6 1.5]);        % kinematic control gain - control saturation
% teste
LF.pPar.K1 = 1*diag([0.6 0.6 0.5 1.2 0.75 0.5]);    % kinematic control gain  - controls amplitude
LF.pPar.K2 = 1*diag([0.5 0.5 0.5 0.82 0.6 1.57]);        % kinematic control gain - control saturation

% LF.pPar.K1 = 1*diag([0.6 0.6 0.5 1.0 0.5 0.5]);    % kinematic control gain  - controls amplitude
% LF.pPar.K2 = 1*diag([0.5 0.5 0.5 0.82 0.6 1.57]);        % kinematic control gain - control saturation


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
Arq = fopen(['FL3d_TrajExp' NomeArq '.txt'],'w');
cd(PastaAtual)
%
% % % Teste método do Igor para salvar dados
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
disp('Data received. Proceding program...');

%% Robot/Simulator conection
A.rConnect;    % ardrone

%% Robots initial pose
% detect rigid body ID from optitrack
idP = getID(OPT,P);         % pioneer ID on optitrack
idA = getID(OPT,A);         % drone ID on optitrack

rb = OPT.RigidBody;         % read optitrack data
A = getOptData(rb(idA),A);  % get ardrone data
P = getOptData(rb(idP),P);  % get pioneer data

A.rTakeOff;
pause(5)
disp('Start..............');

%% Trajectory variables
a  = 1;           % x distance
b  = 0.5;           % y distance
w  = 0.15;           % angular velocity for pioneer

%% Variable initialization
data = [];
% Desired formation [xf yf zf rhof alfaf betaf]
Qd = [1 1 0 1.2 -pi/2 pi/3]';

LF.pPos.Qd = Qd;

% Robots desired pose
LF.mInvTrans;

%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
LF.mDirTrans;

% Formation Error
LF.mFormationError;

%% Configure simulation window
% fig = figure(1);
% plot robots desired position
% plot3(LF.pPos.Xd(1),LF.pPos.Xd(2),LF.pPos.Xd(3),'rsq','LineWidth',2),hold on;
% plot3(LF.pPos.Xd(4),LF.pPos.Xd(5),LF.pPos.Xd(6),'bsq','LineWidth',2);
% % plot  formation line
% xl = [LF.pPos.Xd(1)   LF.pPos.Xd(4)];
% yl = [LF.pPos.Xd(2)   LF.pPos.Xd(5)];
% zl = [LF.pPos.Xd(3)   LF.pPos.Xd(6)];
%
% pl = line(xl,yl,zl);
% pl.Color = 'g';
% pl.LineStyle = '-';
% pl.LineWidth = 1;

% axis([-4 4 -4 4 0 4]);
% % Draw robot
% P.mCADplot(1,'k');              % Pioneer
% A.mCADplot;                     % ArDrone
% grid on
% drawnow
% % pause

%% Simulation
% Time variables initialization
nvoltas = 2;
tsim = 2*pi*nvoltas/w;

timeout = 150;   % maximum simulation duration
% tsim = 65;
t  = tic;
tc = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle

% Loop while error > erroMax
while toc(t) < tsim
    
    
    
    if toc(tc) > 1/30
        tc = tic;
        
        
        if toc(t) > tsim/2
            %         LF.pPos.Qd(4) = Qd;  % rho
            Qd(5) = pi/2;    % alpha
            %         LF.pPos.Qd(6) = pi; % beta
        end

        %% Trajectory
        % Lemniscata (8')
        % Positions
        ta = toc(t);
        LF.pPos.Qd(1)  = a*sin(w*ta);         % x position
        LF.pPos.Qd(2)  = b*sin(2*w*ta);       % y position
        %         LF.pPos.Qd(3)  = 0;                   % z position
        %         LF.pPos.Qd(4)  = 1.2;                   % rho position
        LF.pPos.Qd(5)  = Qd(5) + P.pPos.X(6);          % alpha position
        if abs(LF.pPos.Qd(5)) > pi
            if LF.pPos.Qd(5) > 0
                LF.pPos.Qd(5) = LF.pPos.Qd(5) - 2*pi;
            else
                LF.pPos.Qd(5) = LF.pPos.Qd(5) + 2*pi;
            end
        end
        
        
        %         LF.pPos.Qd(6)  = deg2rad(90);         % beta position
        % Velocities
        LF.pPos.dQd(1)  = a*w*cos(w*ta);      % x velocities
        LF.pPos.dQd(2)  = b*2*w*cos(2*w*ta);  % y velocities
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
               
        % Drone
        A.pPos.Xda     = A.pPos.Xd;              % save previous posture
        A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);        % desired position
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);       % desired velocities
        A.pPos.Xd(6)   = 0; %atan2(A.pPos.Xd(8),A.pPos.Xd(7)); % desired Psi
        
        % Derivative (dPsi)
        if abs(A.pPos.Xd(6) - A.pPos.Xda(6)) > pi
            if A.pPos.Xda(6) < 0
                A.pPos.Xda(6) =  2*pi + A.pPos.Xda(6);
            else
                A.pPos.Xda(6) = -2*pi + A.pPos.Xda(6);
            end
        end
        A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/(1/30);
        
        % ............................................................
        % Dynamic controllers
        A = cUnderActuatedController(A,gains);  % ArDrone
        A = J.mControl(A);                      % joystick command (priority)
        P = fDynamicController(P);               % Pioneer Dynamic controller
        
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
 
        %% Send control signals to robots
        Rede.mSendMsg(P);  % send data to network
        A.rSendControlSignals;
    end
    
    %     %% Draw robot
    %     if toc(tp) > 0.5
    %         tp = tic;
    %
    %         P.mCADdel                      % Pioneer
    %         P.mCADplot(1,'k');             % Pioneer modelo bonitão
    %         A.mCADplot;                    % ArDrone
    %         grid on
    %         drawnow
    %     end
    %
    % Land before the end of simulation
    %     if toc(t) > (tsim - 4)
    %          A.rLand;
    %     end
end

%% Loop Aterrissagem
tland = 5;
while toc(t) < tsim+tland
       
    if toc(tc) > 1/30
        tc = tic;
        %% Trajectory
        % Lemniscata (8')
        ta = toc(t);
        % Positions
        LF.pPos.Qd(1)  = LF.pPos.Q(1);          % x position
        LF.pPos.Qd(2)  = LF.pPos.Q(2);          % y position
        % LF.pPos.Qd(3)  = 0;                   % z position
        LF.pPos.Qd(4)  = 1 - 0.7*(ta-tsim)/(tland);                   % rho position
        LF.pPos.Qd(5)  = Qd(5) + P.pPos.X(6);          % alpha position
        %         LF.pPos.Qd(6)  = deg2rad(90);         % beta position
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
        
        % Drone
        A.pPos.Xda     = A.pPos.Xd;              % save previous posture
        A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);    % desired position
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);   % desired velocities
        %         A.pPos.Xd(6)   = atan2(A.pPos.Xd(8),A.pPos.Xd(7)); % desired Psi
        %
        %         % Derivative (dPsi)
        %         if abs(A.pPos.Xd(6) - A.pPos.Xda(6)) > pi
        %             if A.pPos.Xda(6) < 0
        %                 A.pPos.Xda(6) =  2*pi + A.pPos.Xda(6);
        %             else
        %                 A.pPos.Xda(6) = -2*pi + A.pPos.Xda(6);
        %             end
        %         end
        %         A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/(1/30);
        
        % ............................................................
        % Dynamic controllers
        A = cUnderActuatedController(A,gains);  % ArDrone
        A = J.mControl(A);                      % joystick command (priority)
        P = fDynamicController(P);              % Pioneer Dynamic controller
        
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
  
        %% Send control signals to robots
        P.pSC.Ud = [0; 0]; % stop Pioneer
        Rede.mSendMsg(P);  % send data to network
        A.rSendControlSignals;
    end
    
    %     %% Draw robot
    %     if toc(tp) > 0.5
    %         tp = tic;
    %
    %         P.mCADdel                      % Pioneer
    %         P.mCADplot(1,'k');             % Pioneer modelo bonitão
    %         A.mCADplot;                    % ArDrone
    %         grid on
    %         drawnow
    %     end
    %
    %     % Simulation timeout
    %     if toc(t)> timeout
    %         disp('Timeout man!');
    %         break
    %     end
    % Land before the end of simulation
    %     if toc(t) > (tsim - 4)
    %          A.rLand;
    %     end
end


%% Close files
fclose(Arq);

%% Send control signals to stop robots
% Land drone
if A.pFlag.Connected == 1
    A.rLand;
end

P.pSC.Ud = [0; 0];
% Send to network (a few times to be sure)
for ii = 1:5
    Rede.mSendMsg(P);
end

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
