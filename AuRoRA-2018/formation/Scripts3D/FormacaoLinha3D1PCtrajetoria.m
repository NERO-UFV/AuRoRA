%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [xf yf zf rhof alfaf betaf]

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
P = Pioneer3DX(1);
A = ArDrone(2);
A.pPar.Ts = 1/30;
% A.pPar.Cgains = [0.5 2 0.5 2 5 2; 1 15 1 20 1 2.5];
A.pPar.Cgains = [0.5 1 0.5 1 5 2; 1 15 1 15 1 2.5];

% A.pPar.Cgains = [1.5 3.2 1.2 2 2 17;
%     2 15 2 15 1 5];

% Formation 3D
LF = LineFormation3D;

% Joystick
J = JoyControl;

%% Open file to save data
% NomeArq = datestr(now,30);
% cd('DataFiles')
% cd('Log_FormacaoLinha3D')
% Arq = fopen(['FL3d_expTraj' NomeArq '.txt'],'w');
% cd(PastaAtual)
%
% Teste método do Igor
% DroneVar = CreateDataFile(A,'Exp','Log_Optitrack');
% OptData = CreateOptDataFile(A,'30Hz','Log_Optitrack');



%% Robot/Simulator conection
P.rConnect;
% A.rConnect;

%% Robots initial pose
% Pioneer 3DX
Xo = [-1 0 0 0];
P.rSetPose(Xo');

% ArDrone
% A.rTakeOff;
% A.rGetSensorCalibration;  % not sure what it does
A.pPos.X(3) = 0.75;

% shg              % show graphic window
% pause(5)
disp('Start..............');

%% Trajectory variables
a  = 1.2;               % x distance
b  = 1;                 % y distance
w = 0.05;               % angular velocity for pioneer

%% Variable initialization
data = [];
LF.pPos.Qd = [1 1 0 2 deg2rad(90) deg2rad(60)]';

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
fig = figure(1);
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

axis([-4 4 -4 4 0 4]);
% Draw robot
P.mCADplot(1,'k');             % Pioneer
A.mCADplot;                     % ArDrone
grid on
drawnow
% % pause

%% Simulation
% Time variables initialization
timeout = 150;   % maximum simulation duration
tsim = 130;
t  = tic;
tc = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle

% Loop while error > erroMax
while toc(t) < tsim
    
    if toc(tc) > 1/30
        tc = tic;
        %% Trajectory
        % Lemniscata (8')
        % Positions
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
        
        % Acquire sensors data
        P.rGetSensorData;    % Pioneer
        A.rGetSensorData;
        
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired position ...........................................
        LF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);         % desired position
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);        % desired position
%         P.sInvKinematicModel(LF.pPos.dXr(1:3));  % calculate control signals
        
        % Drone
        A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        % ............................................................
        % Dynamic compensantion
        A = cUnderActuatedController(A,A.pPar.Cgains);  % ArDrone
        A = J.mControl(A);           % joystick command (priority)
        P = fDynamicController(P);
%         P = fCompensadorDinamico(P);      % Pioneer Dynamic Compensator
        
        %         % Save data (.txt file)
        %         fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
        %             A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
        %         fprintf(Arq,'\n\r');
        %         %         %
        %         % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)];
        
%         SaveData(DroneVar,A,toc(t));
%         SaveOptData(OptData,A,toc(t));
        
        % Send control signals to robots
        A.rSendControlSignals;
        P.rSendControlSignals;
        
    end
    
    % Draw robot
    if toc(tp) > 0.5
        tp = tic;
        
        P.mCADdel                     % Pioneer
        P.mCADplot(1,'k');             % Pioneer modelo bonitão
        A.mCADplot;                    % ArDrone
        grid on
        drawnow
    end
    %
    %     % Simulation timeout
    %     if toc(t)> timeout
    %         disp('Timeout man!');
    %         break
    %     end
    
end

%% Close file and stop robot
% fclose(Arq);
% CloseDataFile(DroneVar)
% CloseOptDataFile(OptData)

% Send control signals
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer
% Land drone
if A.pFlag.Connected == 1
    A.rLand;
end

%% Plot results
% figure;
plotResults(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
