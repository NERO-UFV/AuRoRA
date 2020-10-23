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

% Set Start Parameters
IAE  = 0;
ITAE = 0;
iPer = 1;  % Performace index
ISEfv = zeros(size(NSBF.pPos.Qtil));
IAEfv = zeros(size(NSBF.pPos.Qtil));


xinc = 0.5;
yinc = 0.5;
xp = P.pPos.X(1)+xinc;
yp = P.pPos.X(2)+yinc;
zp = 0;
t = tic;
tt_IAE = text(xp,yp+yinc*2,['IAE = ',num2str(IAE,'%3.1f')],'FontWeight','bold');
tt_ITAE = text(xp,yp+yinc,['ITAE = ',num2str(ITAE,'%3.1f')],'FontWeight','bold');
tt_T = text(xp,yp+yinc*2,['t = ',num2str(toc(t),'%3.1f'),' [s]'],'FontWeight','bold');


A.pPar.Ts = 1/30;
% A.pPar.Cgains = [0.5 2 0.5 2 5 2; 1 15 1 20 1 2.5];
A.pPar.Cgains = [0.5 1 0.5 1 5 2; 1 15 1 15 1 2.5];

% A.pPar.Cgains = [1.5 3.2 1.2 2 2 17;
%     2 15 2 15 1 5];

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

% shg              % show graphic window
% pause(5)
disp('Start..............');

%% Trajectory variables
a  = 1.2;               % x distance
b  = 1;                 % y distance
w = 0.05;               % angular velocity for pioneer

%% Variable initialization
data = [];
NSBF.pPos.Qd = [1 1 0 2 deg2rad(90) deg2rad(60)]';

% Robots desired pose
NSBF.mInvTrans;

%% Formation initial error
% Formation initial pose
NSBF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
NSBF.mDirTrans;

% Formation Error
NSBF.mFormationError;

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
        NSBF.pPos.Qd(1)  = a*sin(w*ta);         % x position
        NSBF.pPos.Qd(2)  = b*sin(2*w*ta);       % y position
        %         NSBF.pPos.Qd(3)  = 0;                   % z position
        %         NSBF.pPos.Qd(4)  = 2;                   % rho position
        %         NSBF.pPos.Qd(5)  = deg2rad(0);          % alpha position
        %         NSBF.pPos.Qd(6)  = deg2rad(90);         % beta position
        % Velocities
        NSBF.pPos.dQd(1)  = a*w*cos(w*ta);      % x velocities
        NSBF.pPos.dQd(2)  = b*2*w*cos(2*w*ta);  % y velocities
        NSBF.pPos.dQd(3)  = 0;                  % z velocities
        NSBF.pPos.dQd(4)  = 0;                  % rho velocities
        NSBF.pPos.dQd(5)  = 0;                  % alpha velocities
        NSBF.pPos.dQd(6)  = 0;                  % beta velocities
        
        % Acquire sensors data
        P.rGetSensorData;    % Pioneer
        A.rGetSensorData;
        
        % Formation Members Position
        % NSBF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        
        %% Control
        % Formation Members Position
        NSBF.mSetPose(P,A);
                
        % Formation Control
        % NSBF.mFormationControl;  % Formação Convencional 
        NSBF.mFormationPriorityControl;
        % NSBF.mPositionPriorityControl;
                     
        % Formation Controller
        NSBF.mFormationControl;
              
        % Desired position ...........................................
        NSBF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = NSBF.pPos.Xd(1:3);         % desired position
        P.pPos.Xd(7:9) = NSBF.pPos.dXr(1:3);        % desired position
%         P.sInvKinematicModel(NSBF.pPos.dXr(1:3));  % calculate control signals
        
        % Drone
        A.pPos.Xd(1:3) = NSBF.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = NSBF.pPos.dXr(4:6);
        % ............................................................
        % Dynamic compensantion
        A = cUnderActuatedController(A,A.pPar.Cgains);  % ArDrone
        A = J.mControl(A);           % joystick command (priority)
        P = fDynamicController(P);
%         P = fCompensadorDinamico(P);      % Pioneer Dynamic Compensator
        
        %         % Save data (.txt file)
        %         fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
        %             A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)]);
        %         fprintf(Arq,'\n\r');
        %         %         %
        %         % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' NSBF.pPos.Qd' NSBF.pPos.Qtil' toc(t)];
        
%         SaveData(DroneVar,A,toc(t));
%         SaveOptData(OptData,A,toc(t));
        
        % Send control signals to robots
        A.rSendControlSignals;
        P.rSendControlSignals;
        
        % Get Formation Performance
        % NSBF.mFormationPerformace;
                
        % Calculate Total Scores
        IAE = IAE + .1*norm(NSBF.pPos.Qtil);
        ITAE = ITAE + toc(t)*.1*norm(NSBF.pPos.Qtil);    
        
        % Calculate Individual Scores
        IAEfv(:,iPer+1) = IAEfv(:,iPer) + abs(NSBF.pPos.Qtil);        
        ISEfv(:,iPer+1) = ISEfv(:,iPer) + toc(t)*.1*((NSBF.pPos.Qtil).^2);    
        iPer = iPer + 1;                
        
        xp = P.pPos.X(1)+xinc;
        yp = P.pPos.X(2)+yinc;
        zp = 0;

        
        
    end
    
    % Draw robot
    if toc(tp) > 0.5
        tp = tic;
        
        P.mCADdel                     % Pioneer
        P.mCADplot(1,'k');             % Pioneer modelo bonitão
        A.mCADplot;                    % ArDrone
        grid on

        tt_IAE.String = ['IAE = ',num2str(IAE,'%3.1f')];
        tt_ITAE.String = ['ITAE = ',num2str(ITAE,'%3.1f')];        
        tt_T.String = ['t = ',num2str(toc(t),'%3.1f'),' [s]'];
      
%         tt_T.Position = [xp ; yp+yinc*2 ; zp];
%         tt_V.Position = [xp ; yp+yinc ; zp];                
        aXYZ = axis;
        tt_IAE.Position  = [(aXYZ(1,1)); aXYZ(1,4) ; (aXYZ(1,6)+1.8)];                        
        tt_ITAE.Position = [(aXYZ(1,1)); aXYZ(1,4) ; (aXYZ(1,6)+1.4)];             
        tt_T.Position    = [(aXYZ(1,1)); aXYZ(1,4) ; (aXYZ(1,6)+1)];

        drawnow
    end
    %
    %     % Simulation timeout
    if toc(t)> timeout
        disp('Timeout man!');
        break
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


% Performace Plots
disp(" ");
disp(" ");
disp(strcat(" IAExf: ",toStringJSON(IAEfv(1,iPer))));
disp(strcat(" IAEyf: ",toStringJSON(IAEfv(2,iPer))));
disp(strcat(" IAEzf: ",toStringJSON(IAEfv(3,iPer))));
disp(strcat(" IAErf: ",toStringJSON(IAEfv(4,iPer))));
disp(strcat(" IAEbf: ",toStringJSON(IAEfv(5,iPer))));
disp(strcat(" IAEaf: ",toStringJSON(IAEfv(6,iPer))));
disp(strcat(" IAEf : ",toStringJSON(norm(IAEfv))));
disp(" ");
disp(IAEfv(:,iPer));
disp(norm(IAEfv));


disp(" ");
disp(" ");
disp(strcat(" ISExf: ",toStringJSON(ISEfv(1,iPer))));
disp(strcat(" ISEyf: ",toStringJSON(ISEfv(2,iPer))));
disp(strcat(" ISEzf: ",toStringJSON(ISEfv(3,iPer))));
disp(strcat(" ISErf: ",toStringJSON(ISEfv(4,iPer))));
disp(strcat(" ISEbf: ",toStringJSON(ISEfv(5,iPer))));
disp(strcat(" ISEaf: ",toStringJSON(ISEfv(6,iPer))));
disp(strcat(" ISEf : ",toStringJSON(norm(ISEfv))));
disp(" ");
disp(IAEfv(:,iPer));
disp(norm(IAEfv));



vIndex = (1:1:iPer);
figure()
hold on
    plot(vIndex,IAEfv);
    title("IAE Formation Variables Error");
    xlabel('Index Progress')
    ylabel('Error')
    legend('X','Y','Z','Rho','Alpha','Beta');
    grid on;
hold off


figure()
hold on
    plot(vIndex,ISEfv);
    title("ISE Formation Variables Error");
    xlabel('Index Progress');
    ylabel('Error');
    legend('X','Y','Z','Rho','Alpha','Beta');
    grid on;
hold off





% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
