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
% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
% gains = [.5 2 .5 2 5 2; 1 20 1 15 1 2.5];
gains = [0.7 0.8 0.7 0.8 5 2; 1 12 1 12 1 4];   % teste  simulacao

% Formation 3D
LF = LineFormation3D;

% Formation gains
LF.pPar.K1 = 1*diag([0.75 0.75 0.5 0.5 0.015 0.5]);     % kinematic control gain  - controls amplitude
% LF.pPar.K2 = 1*diag([0.18 0.18 0.5 0.5 0.5 0.5]);       % kinematic control gain - control saturation
LF.pPar.K2 = 1*diag([0.1 0.1 0.5 2 0.5 0.3]);       % kinematic control gain - control saturation


% ganhos originais
% LF.pPar.K1 = 1*diag([0.8 0.8 0.5 0.02 0.015 0.03]);  % kinematic control gain  - controls amplitude
% LF.pPar.K2 = 1*diag([0.1 0.1 0.5 0.5 0.5 0.5]);      % kinematic control gain - control saturation


% Joystick
J = JoyControl;

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_FormacaoLinha3D')
Arq = fopen(['FL3dSIMU_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Robot/Simulator conection
P.rConnect;
A.rConnect;

%% Robot initial pose
% Pioneer 3DX
Xo = [1 -1 0 0];
P.rSetPose(Xo');

% ArDrone
A.rTakeOff;
% A.rGetSensorCalibration;
A.pPos.X(3) = 0.75;

% shg              % show graphic window
pause(5)


disp('Start..............');

%% Variable initialization
data = [];
% Desired formation [xf yf zf rhof alfaf betaf]
% LF.pPos.Qd = [2 -1 0 2 deg2rad(0) deg2rad(90)]';
Qd = [ 1.2   1.2   0   1.5   0   pi/2;
       1.2  -1.2   0   2     0   pi/2;
      -1.2  -1.2   0   1.5   0   pi/2;
      -1.2   1.2   0   2     0   pi/2
       0      0    0   1.5   0   pi/2];
cont = 1;     % counter to change desired position through simulation
time = 15;    % time to change desired positions [s]

%% Configure simulation window
fig = figure(1);
% plot robots initial positions
plot3(P.pPos.X(1),P.pPos.X(2),P.pPos.X(3),'r^','LineWidth',0.8); hold on;
plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'b^','LineWidth',0.8);

for kk = 1:size(Qd,1)
    LF.pPos.Qd = Qd(kk,:)';
    
    LF.mInvTrans;
    plot3(Qd(:,1),Qd(:,2),Qd(:,3),'r.','MarkerSize',20,'LineWidth',2),hold on;
    plot3(LF.pPos.Xd(4),LF.pPos.Xd(5),LF.pPos.Xd(6),'b.','MarkerSize',20,'LineWidth',2);
    % plot  formation line
    xl = [LF.pPos.Xd(1)   LF.pPos.Xd(4)];
    yl = [LF.pPos.Xd(2)   LF.pPos.Xd(5)];
    zl = [LF.pPos.Xd(3)   LF.pPos.Xd(6)];
    
    pl = line(xl,yl,zl);
    pl.Color = 'k';
    pl.LineStyle = '--';
    pl.LineWidth = 1;
    
    axis([-4 4 -4 4 0 4]);
    view(75,15)
    % Draw robot
    % P.mCADplot(1,'k');             % Pioneer
    % A.mCADplot                     % ArDrone
    grid on
    drawnow
end
pause(3)

%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
LF.mDirTrans;

% Formation Error
LF.mFormationError;
%% First desired position
LF.pPos.Qd = Qd(1,:)';

% Robots desired pose
LF.mInvTrans;

% Formation Error
LF.pPos.Qtil = LF.pPos.Qd - LF.pPos.Q;

%% Simulation
% Maximum error permitted
erroMax = [.1 .1 0 .1 deg2rad(5) deg2rad(5)];

% Time variables initialization
timeout = 30;   % maximum simulation duration
% timeout = size(Qd,1)*time + 30;
t  = tic;
tc = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle


% Loop while error > erroMax
while abs(LF.pPos.Qtil(1))>erroMax(1) || abs(LF.pPos.Qtil(2))>erroMax(2) || ...
        abs(LF.pPos.Qtil(4))>erroMax(4)|| abs(LF.pPos.Qtil(5))>erroMax(5) ...
        || abs(LF.pPos.Qtil(6))>erroMax(6) % formation errors
    
    if toc(tc) > 1/30
        
        tc = tic;
        %% Desired positions  / time
        if toc(t)> cont*time && cont <= size(Qd,1)
            
            LF.pPos.Qd = Qd(cont,:)';
            
            % Formation Error
            LF.pPos.Qtil = LF.pPos.Qd - LF.pPos.Q;
            
            cont = cont + 1;
            
        end
        
        %% Acquire sensors data
        P.rGetSensorData;    % Pioneer
        A.rGetSensorData;    % ArDrone
        
        
        %% Control
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        
        % Formation Control
        LF.mFormationControl;
        
        % Desired position ...........................................
        LF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position
        P.sInvKinematicModel(LF.pPos.dXr(1:3));  % calculate control signals
        
        % Drone
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        %             A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7)); % psi
        
        %             % Derivative (dPsi)
        %             if abs(A.pPos.Xd(6) - A.pPos.Xda(6)) > pi
        %                 if A.pPos.Xda(6) < 0
        %                     A.pPos.Xda(6) =  2*pi + A.pPos.Xda(6);
        %                 else
        %                     A.pPos.Xda(6) = -2*pi + A.pPos.Xda(6);
        %                 end
        %             end
        %             A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/(1/30);  % dPsi
        % ............................................................
        % Dynamic compensantion - robot individual control        
        A = cUnderActuatedControllerMexido(A,gains);  % ArDrone
        A = J.mControl(A);                            % joystick command (priority)
        P = fCompensadorDinamico(P);                  % Pioneer Dynamic Compensator
        
        % Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)];
        
        % Send control signals to robots
        P.rSendControlSignals;
        A.rSendControlSignals;
               
    end
    
    
    % Draw robot
    if toc(tp) > 0.2
        tp = tic;
        try
            delete(fig1);
            delete(fig2);
        catch
        end
        P.mCADdel                      % Pioneer
        P.mCADplot(1,'k');             % Pioneer modelo bonitão
        A.mCADplot;                    % ArDrone
        
        % Percourse made
        fig1 = plot3(data(:,13),data(:,14),data(:,15),'r-','LineWidth',0.8); hold on;
        fig2 = plot3(data(:,41),data(:,42),data(:,43),'b-','LineWidth',0.8);
        
%         view(75,30)
        axis([-4 4 -4 4 0 4]);
        grid on
        drawnow
    end
    
    % Simulation timeout
    if toc(t)> size(Qd,1)*timeout
        disp('Timeout man!');
        break
    end
    
end

%% Close file and stop robot
fclose(Arq);

% Send control signals
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer

if A.pFlag.Connected == 1
    A.rLand;
end

%% Plot results
% figure;
plotResults(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
