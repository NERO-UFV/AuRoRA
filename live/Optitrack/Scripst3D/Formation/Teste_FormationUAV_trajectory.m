%% 3D Line Formation Drone-Drone
% Trajectory task using a line virtual structure to control the formation
%
% I.   Ardrone 1 is the reference of the formation
% II.  The formation variables are:
%      Q = [xf yf zf rhof alfaf betaf]


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
% Robots
A1 = ArDrone(1);   % formation reference
A2 = ArDrone(2);

gains = [0.5 3 0.6 3 2 15; 10 3 8 3 1 4];

% Formation 3D
LF = LineFormation3D;
LF.pPar.K1 = 1*diag([0.6 0.6 0.5 1.2 0.75 0.5]);    % kinematic control gain  - controls amplitude
LF.pPar.K2 = 1*diag([0.5 0.5 0.5 0.82 0.6 1.57]);   % kinematic control gain - control saturation

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
Arq = fopen(['FL3d_2dronesTrajExp' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Network communication check
tm = tic;
while true
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(A1);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 1
        if isempty(Rede.pMSG.getFrom{2})
            Rede.mSendMsg(A1);
            
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
A2.rConnect;    % ardrone

%% Robots initial pose
% detect rigid body ID from optitrack
idA1 = getID(OPT,A1,1);         % pioneer ID on optitrack
idA2 = getID(OPT,A2,2);         % drone ID on optitrack

rb = OPT.RigidBody;             % read optitrack data
A1 = getOptData(rb(idA1),A1);   % get pioneer data
A2 = getOptData(rb(idA2),A2);   % get ardrone data

A2.rTakeOff;
pause(5)
disp('Start..............');

%% Trajectory variables
a  = 1.2;           % x distance
b  = 0.8;           % y distance
w  = 0.2;           % angular velocity for pioneer
laps = 2;
tsim    = 2*pi*laps/w;

%% Variable initialization
data = [];
% Desired formation [xf yf zf rhof alfaf betaf]
LF.pPos.Qd = [1 1 0.75 1 pi pi/3]';

% Robots desired pose
LF.mInvTrans;

%% Formation initial error
% Formation initial pose
LF.pPos.X = [A1.pPos.X(1:3); A2.pPos.X(1:3)];

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
% A1.mCADplot(1,'k');              % Ardrone 1
% A2.mCADplot;                     % ArDrone 2
% grid on
% drawnow
% % pause

%% Simulation
% Time variables initialization
% timeout = 150;   % maximum simulation duration
% tsim = 65;
t  = tic;
tc = tic;
tp = tic;

% Loop while error > erroMax
while toc(t) < tsim
        
    if toc(tc) > 1/30
        tc = tic;
        %% Trajectory
        % Lemniscata (8')
        ta = toc(t);
        LF.pPos.Qd(1)  = a*sin(w*ta);         % x position
        LF.pPos.Qd(2)  = b*sin(2*w*ta);       % y position
        %         LF.pPos.Qd(3)  = 0;                   % z position
        %         LF.pPos.Qd(4)  = 2;                   % rho position
        %         LF.pPos.Qd(5)  = deg2rad(0);          % alpha position
        %         LF.pPos.Qd(6)  = deg2rad(90);         % beta position
        % Velocities
        LF.pPos.dQd(1)  = a*w*cos(w*ta);      % x velocities
        LF.pPos.dQd(2)  = b*2*w*cos(2*w*ta);  % y velocities
        LF.pPos.dQd(3)  = 0;                  % z velocities
        LF.pPos.dQd(4)  = 0;                  % rho velocities
        LF.pPos.dQd(5)  = 0;                  % alpha velocities
        LF.pPos.dQd(6)  = 0;                  % beta velocities
        
        %% Acquire sensors data
        % Get network data (in this case not important since all data comes from optitrack)
%         Rede.mReceiveMsg;
%         if length(Rede.pMSG.getFrom)>1
%             A1.pSC.U  = Rede.pMSG.getFrom{2}(31:34);  % current velocities (robot sensors)
%             A1X       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
%         end
        
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
  
        A1 = getOptData(rb(idA1),A1);
        A1.pSC.U = [A1.pPos.X(4);A1.pPos.X(5);A1.pPos.X(9);A1.pPos.X(12)]; % populates control signal actual
        
        A2 = getOptData(rb(idA2),A2);
        A2.pSC.U = [A2.pPos.X(4);A2.pPos.X(5);A2.pPos.X(9);A2.pPos.X(12)]; % populates control signal actual                             
        
       
        %% Control
        % Formation Members Position
        LF.pPos.X = [A1.pPos.X(1:3); A2.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired position ...........................................
        LF.mInvTrans;
        % Drone 1
        A2.pPos.Xda = A2.pPos.Xd;              % save previous posture
        A1.pPos.Xd(1:3) = LF.pPos.Xd(4:6);    % desired position
        A1.pPos.Xd(7:9) = LF.pPos.dXr(4:6);   % desired velocities
        A1.pPos.Xd(6) = atan2(A1.pPos.Xd(8),A1.pPos.Xd(7)); % desired Psi
        
        % Derivative (dPsi)
        if abs(A1.pPos.Xd(6) - A1.pPos.Xda(6)) > pi
            if A1.pPos.Xda(6) < 0
                A1.pPos.Xda(6) =  2*pi + A1.pPos.Xda(6);
            else
                A1.pPos.Xda(6) = -2*pi + A1.pPos.Xda(6);
            end
        end
        A1.pPos.Xd(12) = (A1.pPos.Xd(6) - A1.pPos.Xda(6))/(1/30);
        
        % Drone 2
        A2.pPos.Xda = A2.pPos.Xd;              % save previous posture
        A2.pPos.Xd(1:3) = LF.pPos.Xd(4:6);    % desired position
        A2.pPos.Xd(7:9) = LF.pPos.dXr(4:6);   % desired velocities
        A2.pPos.Xd(6) = atan2(A2.pPos.Xd(8),A2.pPos.Xd(7)); % desired Psi
        
        % Derivative (dPsi)
        if abs(A2.pPos.Xd(6) - A2.pPos.Xda(6)) > pi
            if A2.pPos.Xda(6) < 0
                A2.pPos.Xda(6) =  2*pi + A2.pPos.Xda(6);
            else
                A2.pPos.Xda(6) = -2*pi + A2.pPos.Xda(6);
            end
        end
        A2.pPos.Xd(12) = (A2.pPos.Xd(6) - A2.pPos.Xda(6))/(1/30);
        
        % ............................................................
        % Dynamic compensantion
        A1 = cUnderActuatedControllerMexido(A1,gains);  % ArDrone
        A2 = cUnderActuatedControllerMexido(A2,gains);  % ArDrone
        A2 = J.mControl(A2);                      % joystick command (priority)
         
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[A1.pPos.Xd' A1.pPos.X' A1.pSC.Ud' A1.pSC.U' ...
            A2.pPos.Xd' A2.pPos.X' A2.pSC.Ud' A2.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
        %         %
        %         % Variable to feed plotResults function
        %         data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
        %             A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)];
        %% Send control signals to robots
        A2.rSendControlSignals;
        Rede.mSendMsg(A1);  % send data to network
        
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
    
end

%% Close files
fclose(Arq);

%% Send control signals to stop robots
A1.pSC.Ud = [0; 0; 0; 0];
% Send to network (a few times to be sure)
for ii = 1:5
    Rede.mSendMsg(A1);
end

% Land drone
if A2.pFlag.Connected == 1
    A2.rLand;
end

%% Plot results
% figure;
% plotResults(P,A,data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
