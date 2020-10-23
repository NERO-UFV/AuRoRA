%% 3D Line Formation Drone-Drone
% Drone 1 is the reference of the formation
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
A1 = ArDrone(1);
A2 = ArDrone(2);
% A.pPar.Ts = 1/30;
gains = [1 2 4 2 5 2; 15 30 15 30 1 2.5];
% gains = [0.4 1 0.4 1 0.8 3; 4 12 4 12 4 2.5];

% A.pPar.Cgains = [1.5 3.2 1.2 2 2 17;
%     2 15 2 15 1 5];

% Formation 3D
LF = LineFormation3D;

% Joystick
J = JoyControl;

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_FormacaoLinha3D')
Arq = fopen(['FL3d_2DronesSimTraj' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Robot/Simulator conection
% A1.rConnect;
% A2.rConnect;

%% Robots initial pose
% ArDrone
% A1.rTakeOff;
% A2.rTakeOff;

% A1.rGetSensorCalibration;  % not sure what it does
% A2.rGetSensorCalibration;  % not sure what it does
A2.pPos.X([1 2]) = [1; 1];


% shg              % show graphic window
% pause(5)
disp('Start..............');

%% Trajectory variables
a  = 3;               % x distance
b  = 2;               % y distance
c  = 2;               % altitude
w = 0.05;              % angular velocity for pioneer

%% Variable initialization
data = [];
LF.pPos.Qd = [1 1 1 1.5 pi 0]';

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

axis([-6 6 -6 6 0 6]);
% Draw robot
A1.mCADplot               % Ardrone 1
A2.mCADplot;              % ArDrone 2
grid on
drawnow
% % pause

%% Simulation
% Time variables initialization
timeout = 150;   % maximum simulation duration
tsim    = 150;
t       = tic;
tc      = tic;
tp      = tic;
t1      = tic;        % ardrone cycle
t2      = tic;        % ardrone cycle

% Loop while error > erroMax
while toc(t) < tsim
    
    if toc(tc) > 1/30
        tc = tic;
        %% Trajectory
        % Elipse
        % Positions
        ta = toc(t);
        LF.pPos.Qd(1)  = a*cos(w*ta);         % x position
        LF.pPos.Qd(2)  = b*sin(w*ta);         % y position
%         LF.pPos.Qd(3)  = 3;                   % z position
        LF.pPos.Qd(3) = 3 + c*sin(w*ta);      % z 
        LF.pPos.Qd(5) = w*ta;                 % alpha 
        LF.pPos.Qd(6) = pi/4;                 % beta
        %         LF.pPos.Qd(4)  = 2;                   % rho position
        %         LF.pPos.Qd(5)  = deg2rad(0);          % alpha position
        %         LF.pPos.Qd(6)  = deg2rad(90);         % beta position
        % Velocities
        LF.pPos.dQd(1)  = -a*w*sin(w*ta);      % x velocities
        LF.pPos.dQd(2)  = b*w*cos(2*w*ta);  % y velocities
        LF.pPos.dQd(3)  = 0;                  % z velocities
        LF.pPos.dQd(4)  = 0;                  % rho velocities
        LF.pPos.dQd(5)  = w;                  % alpha velocities
        LF.pPos.dQd(6)  = 0;                  % beta velocities
        
        % Acquire sensors data
        A1.rGetSensorData;    % Pioneer
        A2.rGetSensorData;
        
        % Formation Members Position
        LF.pPos.X = [A1.pPos.X(1:3); A2.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired position ...........................................
        LF.mInvTrans;
        % ArDrone 1
        A1.pPos.Xd(1:3) = LF.pPos.Xd(1:3);
        A1.pPos.Xd(7:9) = LF.pPos.dXr(1:3);
        
        % ArDrone 2
        A2.pPos.Xd(1:3) = LF.pPos.Xd(4:6);
        A2.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        
        % ............................................................
        % Dynamic compensantion
        A1 = cUnderActuatedController(A1,gains);  % ArDrone
%         A1 = J.mControl(A1);           % joystick command (priority)
        
        A2 = cUnderActuatedController(A2,gains);  % ArDrone
%         A2 = J.mControl(A2);           % joystick command (priority)
               
        lim = (15*pi/180);
        % U = [phi theta dz dpsi]
        A1.pSC.U = [A1.pPos.X(4)/lim; A1.pPos.X(5)/lim; A1.pPos.X(9); A1.pPos.X(12)/(100*pi/180)];
        A2.pSC.U = [A2.pPos.X(4)/lim; A2.pPos.X(5)/lim; A2.pPos.X(9); A2.pPos.X(12)/(100*pi/180)];

        % Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[A1.pPos.Xd' A1.pPos.X' A1.pSC.Ud' A1.pSC.U' ...
            A2.pPos.Xd' A2.pPos.X' A2.pSC.Ud' A2.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
      
        % Variable to feed plotResults function
        data = [data; A1.pPos.Xd' A1.pPos.X' A1.pSC.Ud' A1.pSC.U' ...
            A2.pPos.Xd' A2.pPos.X' A2.pSC.Ud' A2.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)];
          
        % Send control signals to robots
        A1.rSendControlSignals;
        A2.rSendControlSignals;
        
    end
    
    % Draw robot
    if toc(tp) > 0.2
        tp = tic;
        A1.mCADplot;                    % ArDrone 1
        A2.mCADplot;                    % ArDrone 2
        
        
%         plot3(LF.pPos.Qd(1),LF.pPos.Qd(2),LF.pPos.Qd(3)), hold on; % desired formation
%         plot3(LF.pPos.Q(1),LF.pPos.Q(2),LF.pPos.Q(3)); % real formation

        grid on
        drawnow
    end

end

%% Close file and stop robot
fclose(Arq);

% Land drone 1
if A1.pFlag.Connected == 1
    A1.rLand;
end
% Land drone 2
if A2.pFlag.Connected == 1
    A2.rLand;
end

%% Plot results
% Percourse made
figure;
p1 = plot3(data(:,13),data(:,14),data(:,15),'r-','LineWidth',1); % real drone 1
hold on;
p2 = plot3(data(:,45),data(:,46),data(:,47),'b-','LineWidth',1); % real drone 2

pd1 = plot3(data(:,1),data(:,2),data(:,3),'r--','LineWidth',0.5); % real drone 1
hold on;
pd2 = plot3(data(:,33),data(:,34),data(:,35),'b--','LineWidth',0.5); % real drone 2


xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','interpreter','Latex');
grid on

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
