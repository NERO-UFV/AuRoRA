%% NSB 3D Line Formation Pioneer-Drone 
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [xf yf zf rhof aNSBFaf betaf]

%
% Developer : Mauro Mafra - 06/02/2019
% Revision: R00 - Inicial Version
% 

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
try
    P = Pioneer3DX(1);
    A = ArDrone(2);
    NSBF = NullSpace3D;  % Null Space Object Instance
catch
    disp(' ####   Load Class Issues   ####');
    disp('');
    disp(ME);
end

% Inicialize Variables
xinc = 0.5;
yinc = 0.5;
xp = P.pPos.X(1)+xinc;
yp = P.pPos.X(2)+yinc;
zp = 0;
t = tic;
tt_IAE  = text(xp,yp+yinc*2,['IAE = ',num2str(NSBF.IAE,'%3.1f')],'FontWeight','bold');
tt_ITAE = text(xp,yp+yinc,  ['ITAE = ',num2str(NSBF.ITAE,'%3.1f')],'FontWeight','bold');
tt_T    = text(xp,yp+yinc*2,['t = ',num2str(toc(t),'%3.1f'),' [s]'],'FontWeight','bold');

% Performace Score Comparison
ISEc = zeros(7,3);
IAEc = zeros(7,3);

data = [];
NSBF.pPos.Qd = [1 1 0 2 deg2rad(60) deg2rad(60)]';

%% Controller Gains
A.pPar.Ts = 1/30;
%gains = [0.5 1 0.5 1 5 2; 1 15 1 15 1 2.5];
gains = [0.7 0.8 0.7 0.8 5 2; 1 12 1 12 1 4];   % teste  simulacao Referencia
pGains = [0.75 0.75 0.12 0.035];                % Ganhos do Compensador do Paioneer

index = 1;

% ganhos originais
NSBF.pPar.K1 = 1*diag([2 3 2 3 0.25 0.5]);            % kinematic control gain  - controls amplitude
NSBF.pPar.K2 = 1*diag([0.1 0.1 0.1 0.1 0.1 0.1]);     % kinematic control gain - control saturation

% Joystick
J = JoyControl;

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_NSB')
Arq = fopen(['NSB_BLem8_' NomeArq '.txt'],'w');
cd(PastaAtual)


%% Robot/Simulator conection
% P.rConnect;
% A.rConnect;

%% Robots initial pose
% Pioneer 3DX
Xo = [-1 0 0 0];
P.rSetPose(Xo');

% ArDrone
% A.rTakeOff;
% A.rGetSensorCalibration;  % not sure what it does
A.pPos.X(3) = 0.75;

% show graphic window
% pause(5)
disp('Start..............');

%% Trajectory variables
a  = 2;               % x distance
b  = 2;               % y distance
w = 0.1;              % angular velocity for pioneer

%% Formation initial error

NSBF.mInvTrans;

% Formation initial pose
NSBF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
NSBF.mDirTrans;

% Formation Error
NSBF.mFormationError;

% Formation Weighted Error 
NSBF.mFormationWeightedError;

%% Configure simulation window
fig = figure(1);
% plot robots desired position
% plot3(NSBF.pPos.Xd(1),NSBF.pPos.Xd(2),NSBF.pPos.Xd(3),'rsq','LineWidth',2),hold on;
% plot3(NSBF.pPos.Xd(4),NSBF.pPos.Xd(5),NSBF.pPos.Xd(6),'bsq','LineWidth',2);
% % plot  formation line
% xl = [NSBF.pPos.Xd(1)   NSBF.pPos.Xd(4)];
% yl = [NSBF.pPos.Xd(2)   NSBF.pPos.Xd(5)];
% zl = [NSBF.pPos.Xd(3)   NSBF.pPos.Xd(6)];
%
% pl = line(xl,yl,zl);
% pl.Color = 'g';
% pl.LineStyle = '-';
% pl.LineWidth = 1;

axis([-4 4 -4 4 0 3]);
% Draw robot

P.mCADplot(1,'k');             % Pioneer
A.mCADplot;                     % ArDrone
grid on
drawnow
% % pause

%% Simulation
% Time variables initialization
cSwitch    = 4;
sampleTime = 1/30;
tsim       = 60;                                 % Maximum Time Simulation 

t  = tic;
tc = tic;
ts = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle

% Loop while error > erroMax
while toc(t) < tsim
    
    if toc(tc) > sampleTime
        tc = tic;        
        %% Trajectory          
        ta = toc(t);
        
        % NSBF.mTrajectoryLemniscata(a,b,w,ta); % Lemniscata
        NSBF.mTrajectoryEllipse(a,b,w,ta);      % Ellipse/Circle
                
        % Acquire sensors data
        P.rGetSensorData;                       % Pioneer
        A.rGetSensorData;                       % Drone        
        
        %% Control
        % Formation Members Position
        NSBF.mSetPose(P,A);              
                
        % Formation Control
        switch cSwitch
            case 1
                NSBF.mFormationControl;            % Conventional Control
                ISEc(:,cSwitch) = [NSBF.ISEv;NSBF.ISE];
                IAEc(:,cSwitch) = [NSBF.IAEv;NSBF.IAE];
            case 2
                NSBF.mFormationPriorityControl;    % Formation Priority Control 
                ISEc(:,cSwitch) = [NSBF.ISEv;NSBF.ISE];
                IAEc(:,cSwitch) = [NSBF.IAEv;NSBF.IAE];
            case 3
                NSBF.mPositionPriorityControl;     % Prosition Priority Control
                ISEc(:,cSwitch) = [NSBF.ISEv;NSBF.ISE];
                IAEc(:,cSwitch) = [NSBF.IAEv;NSBF.IAE];
            case 4
                NSBF.mAuxPriorityControl;     % Prosition Priority Control
                ISEc(:,cSwitch) = [NSBF.ISEv;NSBF.ISE];
                IAEc(:,cSwitch) = [NSBF.IAEv;NSBF.IAE];
        end
        
        % Desired position ...........................................
        NSBF.mInvTrans;
                                
        % Pioneer
        P.pPos.Xd(1:3) = NSBF.pPos.Xd(1:3);         % desired position
        P.pPos.Xd(7:9) = NSBF.pPos.dXr(1:3);        % desired position
       
        % Drone
        A.pPos.Xd(1:3) = NSBF.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = NSBF.pPos.dXr(4:6);
        
        % ............................................................
        % Dynamic compensantion
        A = cUnderActuatedController(A,gains);  % ArDrone
        A = J.mControl(A);           % joystick command (priority)
        %P = fDynamicController(P);

        % Verificar para usar um compesador dinâmico
        P.sInvKinematicModel(NSBF.pPos.dXr(1:3));  % calculate control signals
        P = fCompensadorDinamico(P);      % Pioneer Dynamic Compensator

        % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)];
        
        
        % Send control signals to robots
        A.rSendControlSignals;
        P.rSendControlSignals;
        
        % Get Formation Performance Scores        
        NSBF.mFormationPerformace(sampleTime,toc(t),index);                      
        
        xp = P.pPos.X(1)+xinc;
        yp = P.pPos.X(2)+yinc;
        zp = 0;
       
        index = index + 1;
        
    end
    
    % Draw robot
    if toc(tp) > 0.5
    %if toc(tp) > inf
        tp = tic;   
%         try
%             delete(fig1);
%             delete(fig2);     
%         catch
%         end
                
        P.mCADdel                     % Pioneer
        P.mCADplot(1,'k');             % Pioneer modelo bonitão
        A.mCADplot;                    % ArDrone
        grid on
        
%         % Percourse made
%         fig1 = plot3(data(:,13),data(:,14),data(:,15),'r-','LineWidth',0.8); hold on;
%         fig2 = plot3(data(:,41),data(:,42),data(:,43),'b-','LineWidth',0.8);

        tt_IAE.String = ['IAE = ',num2str(NSBF.IAE,'%3.1f')];
        tt_ITAE.String = ['ITAE = ',num2str(NSBF.ITAE,'%3.1f')];        
        tt_T.String = ['t = ',num2str(toc(t),'%3.1f'),' [s]'];
      
        aXYZ = axis;
        tt_IAE.Position  = [(aXYZ(1,1)); aXYZ(1,4) ; (aXYZ(1,6)+1.8)];                        
        tt_ITAE.Position = [(aXYZ(1,1)); aXYZ(1,4) ; (aXYZ(1,6)+1.4)];             
        tt_T.Position    = [(aXYZ(1,1)); aXYZ(1,4) ; (aXYZ(1,6)+1)];

        
        grid on
        drawnow
        view(-21,30)

    end
    
end

%% Close file and stop robot
 fclose(Arq);
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

% Display Formation Performace on Command Windows
NSBF.mDisplayFormationPerformace;


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
