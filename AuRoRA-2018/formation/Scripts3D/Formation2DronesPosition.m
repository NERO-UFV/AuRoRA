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
% A.pPar.Cgains = [0.5 2 0.5 2 5 2; 1 15 1 20 1 2.5];
gains = [0.5 1 0.5 1 5 2; 1 15 1 15 1 2.5];

% A.pPar.Cgains = [1.5 3.2 1.2 2 2 17;
%     2 15 2 15 1 5];

% Formation 3D
LF = LineFormation3D;

% Joystick
J = JoyControl;

P = Pioneer3DX;

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
b  = 2;                 % y distance
w = 0.08;               % angular velocity for pioneer

%% Variable initialization
data = [];
    Qd = [ 1  1   1  1  0  pi/2;
           1  -1  1  1  0  pi/2;
          -1 -1   1  1  0  pi/2;
          -1  1   1  1  0  pi/2;
           0  0   1  1  0  pi/2 ];
cont = 0;     % counter to change desired position through simulation
time = 10;    % time to change desired positions [s]
LF.pPos.Qd = Qd(1,:)';
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

axis([-6 6 -6 6 0 4]);
% Draw robot
A1.mCADplot               % Ardrone 1
A2.mCADplot;              % ArDrone 2
grid on
drawnow
% % pause

%% Simulation
% Time variables initialization
timeout = 150;   % maximum simulation duration
tsim    = 60;
t       = tic;
tc      = tic;
tp      = tic;
t1      = tic;        % ardrone cycle
t2      = tic;        % ardrone cycle

% Loop while error > erroMax
while toc(t) < tsim
    
    if toc(tc) > 1/30
        tc = tic;
         %% Desired positions
        if toc(t)> cont*time
            cont = cont + 1;
        end
        if cont <= size(Qd,1)
           LF.pPos.Qd = Qd(cont,:)';
        end         % beta velocities
        
        %% Acquire sensors data
        A1.rGetSensorData;    % Pioneer
        A2.rGetSensorData;
        
        %% Control
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
        A1 = J.mControl(A1);           % joystick command (priority)
        
        A2 = cUnderActuatedController(A2,gains);  % ArDrone
        A2 = J.mControl(A2);           % joystick command (priority)
        
        %% Save data (.txt file)
        %         fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
        %             A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
        %         fprintf(Arq,'\n\r');
        %         %         %
        %         % Variable to feed plotResults function
        data = [data; A1.pPos.Xd' A1.pPos.X' A1.pSC.Ud' A1.pSC.U' ...
            A2.pPos.Xd' A2.pPos.X' A2.pSC.Ud' A2.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)];
        
        %         SaveData(DroneVar,A,toc(t));
        %         SaveOptData(OptData,A,toc(t));
        
        %% Send control signals to robots
        A1.rSendControlSignals;
        A2.rSendControlSignals;
        
    end
    
    % Draw robot
    if toc(tp) > 0.2
        tp = tic;
%         P.mCADdel;
        A1.mCADplot;                    % ArDrone 1
        A2.mCADplot;                    % ArDrone 2
%         P.pPos.Xc = A2.pPos.X;
%         P.mCADplot(0.5,'k');
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
p1 = plot3(data(:,13),data(:,14),data(:,15),'r--','LineWidth',2);
hold on;
p1d = plot3(data(:,1),data(:,2),data(:,3),'ko','LineWidth',1);

p2 = plot3(data(:,45),data(:,46),data(:,47),'b-','LineWidth',1);
p2d = plot3(data(:,33),data(:,34),data(:,35),'ko','LineWidth',1);

% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','interpreter','Latex');
grid on
hold off

% % Desired positions
% pd1 = plot3(Xd(:,1),Xd(:,2),Xd(:,3)+h,'ro','LineWidth',sl);hold on % pioneer
% pd2 = plot3(AXd(:,1),AXd(:,2),AXd(:,3)+h,'b--','LineWidth',sl);     % drone

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
