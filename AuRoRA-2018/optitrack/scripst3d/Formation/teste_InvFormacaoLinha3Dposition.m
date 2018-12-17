%% 3D Line Formation Pioneer-Drone: trajetory control using optitrack
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [xf yf zf rhof alfaf betaf]

%% Initialization
clear
close all
clc

try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_Optitrack')
Arq = fopen(['FL3d_' NomeArq '.txt'],'w');
cd(PastaAtual)


%% Load Classes
% Robot
P = Pioneer3DX(1);
A = ArDrone(2);

% 3D Formation
LF = LineFormation3D;

% Optitrack
OPT = OptiTrack;
OPT.Initialize;

% Network data sharing
Rede = NetDataShare;

% Connect Joystick
J = JoyControl;


%% Robot/Simulator conection
P.rConnect;       % Pioneer 3dx
A.rConnect;       % ArDrone
A.rTakeOff;       % ArDrone take off
% A.rGetSensorCalibration;
pause(8);

%% Robots initial pose  (OptiTrack)
rb = OPT.RigidBody;   % get current rigid body information from optitrack
n = length(rb);       % number of rigid bodies tracked
if ~isempty(n)
    for ii = 1:n
        switch rb(ii).Name(1)
            case 'P'
                P = getOptData(rb(ii),P);   % get Pioneer pose
            case 'A'
                A = getOptData(rb(ii),A);   % get ArDrone pose
            otherwise
                disp('Unknown rigid body type. (Known types [case sensitive]: "A" or "P")');
        end
    end
    
else
    disp('No rigid body tracked');
end

pause(1)
disp('Start..............');

%% Network communication check - for Pioneer's machine
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
disp('Data received. Segue o jogo...');

%% Constant Variables initialization
data = [];      % save variables to draw graphics
a = 1.5;        % x radius
b = 1.5;        % y radius
w = 0.007;      % angular velocity for trajectory
dt = 1/30;      % ArDrone initial refresh rate

% Desired formation [xf yf zf rhof alfaf betaf]
LF.pPos.Qd = [1 -1 0 1 deg2rad(90) deg2rad(45)]';

% % Robots desired pose
% LF.mInvTrans;
%% Formation initial error
% Formation members initial pose [pioneer(x,y,z) drone(x,y,z)]
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

%Formation Error
LF.mDirTrans;            % initial pose
LF.mFormationError;

% Robot Error
LF.mInvFormationError;

%% Configure simulation window
fig = figure(1);
% plot robots desired position
h1 = plot3(LF.pPos.Xd(1),LF.pPos.Xd(2),LF.pPos.Xd(3),'rsq','LineWidth',2);
hold on;
h2 = plot3(LF.pPos.Xd(4),LF.pPos.Xd(5),LF.pPos.Xd(6),'bsq','LineWidth',2);
% plot  desired formation line
xl = [LF.pPos.Xd(1)   LF.pPos.Xd(4)];
yl = [LF.pPos.Xd(2)   LF.pPos.Xd(5)];
zl = [LF.pPos.Xd(3)   LF.pPos.Xd(6)];

pl = line(xl,yl,zl);
pl.Color = 'g';
pl.LineStyle = '-';
pl.LineWidth = 1;

axis([-8 8 -8 8 0 4])
% Draw robot
% P.mCADplot2D('b')            % Pioneer
P.mCADplot(1,'k');             % Pioneer
A.mCADplot                     % ArDrone
grid on
drawnow
% pause(2)

%% Simulation
% Maximum error permitted
erroMax = [.2 .2 0 .2 deg2rad(10) deg2rad(10)];

% Time variables
timeout = 360;   % maximum simulation duration
t      = tic;    % current time
tc     = tic;    % formation refresh rate
tp     = tic;    % graphic refresh rate
t1     = tic;    % pioneer cycle
t2     = tic;    % ardrone cycle
tdelay = tic;    % time to start trajectory
td     = tic;    % sampling time

% Loop while error > erroMax
while abs(LF.pPos.Qtil(1))>erroMax(1) || abs(LF.pPos.Qtil(2))>erroMax(2) || ...
        abs(LF.pPos.Qtil(4))>erroMax(4)|| abs(LF.pPos.Qtil(5))>erroMax(5) ...
        || abs(LF.pPos.Qtil(6))>erroMax(6) || toc(t)> timeout % formation errors
    
    if toc(tc) > 0.1
        
        tc = tic;
        
        LF.pPos.dXr(1) = 0;  % dx
        LF.pPos.dXr(2) = 0;  % dy
        LF.pPos.dXr(3) = 0;  % dz
        
        %% Data aquisition
        % Get network data from Pioneer 3DX
        Rede.mReceiveMsg;
        % If there is data from optitrack, overwrite position values
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30); % current velocities (robot sensors)
            X  = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        else
            k = k+1;
            disp('No data received from network......')
        end
        
        % get positions/angles from optitrack
        rb = OPT.RigidBody;
        n = length(rb);
        
        % sampling time (variable)
        dt = toc(td);
        if dt > 2/30
            dt = 1/30;
        end
        
        if ~isempty(n)
            for ii = 1:n
                switch rb(ii).Name(1)
                    
                    % Pioneer ..........................................
                    case 'P'
                        P = getOptData(rb(ii),P);
                        
                        P.pPos.X(7:9) = (P.pPos.X(1:3)-P.pPos.Xa(1:3))/dt;
                        P.pPos.X(10:11) = (P.pPos.X(4:5)-P.pPos.Xa(4:5))/dt;
                        if abs(P.pPos.X(6) - P.pPos.Xa(6)) > pi
                            if P.pPos.Xa(6) < 0
                                P.pPos.Xa(6) =  2*pi + P.pPos.Xa(6);
                            else
                                P.pPos.Xa(6) = -2*pi + P.pPos.Xa(6);
                            end
                        end
                        P.pPos.X(12) = (P.pPos.X(6)-P.pPos.Xa(6))/dt; 
                        
                        P.pPos.Xa = P.pPos.X;   % saves previous pose   
                        td = tic;               % reset time
                        % ArDrone .......................................
                    case 'A'
                        A = getOptData(rb(ii),A);
                        A.pPos.X(7:9) = (A.pPos.X(1:3)-A.pPos.Xa(1:3))/dt;
                        A.pPos.X(10:11) = (A.pPos.X(4:5)-A.pPos.Xa(4:5))/dt;
                        if abs(A.pPos.X(6) - A.pPos.Xa(6)) > pi
                            if A.pPos.Xa(6) < 0
                                A.pPos.Xa(6) =  2*pi + A.pPos.Xa(6);
                            else
                                A.pPos.Xa(6) = -2*pi + A.pPos.Xa(6);
                            end
                        end
                        A.pPos.X(12) = (A.pPos.X(6)-A.pPos.Xa(6))/dt;
                        
                        A.pPos.Xa = A.pPos.X;    % saves previous pose
                        td = tic;                % reset time 
                    otherwise
                        disp('Unknown rigid body type. (Known types: "A" or "P")');
                end
            end
            
        else
            disp('No rigid body tracked');
        end
        
        A.rGetAngles;      % get roll/pitch angles from drone sensors
        
      
        
        %% Formation Control (Kinematics)
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3);];
        
        % Formation Controller
        LF.mInvFormationControl;
        
        %Formation Error
        LF.mDirTrans;
        LF.mFormationError;
        
        
        % Desired position ...........................................
        % Pioneer
        %         LF.mInvTrans;
        %         P.pPos.Xd(1:3) = LF.pPos.Xr(1:3);
        
        % Robot Control Velocity
        P.sInvKinematicModel(LF.pPos.dXr(1:3));   % Calculate control signals
        
        % Drone
        % Position Integration for ArDrone
        %       A.pPos.Xd(1:3) = A.pPos.Xd(1:3) + LF.pPos.dXr(4:6)*0.1;
        %       A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        %
        % ............................................................
        
        
        
    end
    
    %% Robot Control (Dynamics)
    
    % Pioneer ...........................................................
    if toc(t1) > P.pPar.Ts
        t1 = tic;
        P = fCompensadorDinamico(P);      % Pioneer Dynamic Compensator
        % ##################################################
        %     % If not using dynamic compensation
        %     P.pSC.Ud = P.pSC.Ur;              % Pioneer
        % ##################################################
      
        % Send Control Signal
          Rede.mSendMsg(P);      % send data to network           
          
    end
    
    % ArDrone ...........................................................
    if toc(t2) > A.pPar.Ts
        t2 = tic;
        A = cUnderActuatedController(A);  % ArDrone
        A = J.mControl(A);           % joystick command (priority)
        % ##################################################
        % If not using dynamic compensation
        %               A.pSC.Ud = A.pSC.Ur;              % ArDrone
        % ##################################################
        % Tentativa de ajuste de psi
        %           A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7));
        
        % Send Control Signal
        A.rSendControlSignals;            % ArDrone
        
        
    end
    
    %% Save data
    %         fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
    %             A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
    %         fprintf(Arq,'\n\r');   % .txt file
    
    % Variable to feed plotResults function
    data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
        A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)];
    
    
    %% Draw robot
    if toc(tp) > 0.1
        tp = tic;
        
        P.mCADdel                     % delete previous Pioneer
        P.mCADplot(1,'k');            % plot new Pioneer
        A.mCADplot                    % move ArDrone model
        
        grid on
        drawnow
        %testando
        delete(h1);
        delete(h2);
        delete(pl);
        % plot robots desired position
        h1 = plot3(LF.pPos.Xd(1),LF.pPos.Xd(2),LF.pPos.Xd(3),'rsq','LineWidth',2);
        hold on;
        h2 = plot3(LF.pPos.Xd(4),LF.pPos.Xd(5),LF.pPos.Xd(6),'bsq','LineWidth',2);
        % plot  formation line
        xl = [LF.pPos.Xd(1)   LF.pPos.Xd(4)];
        yl = [LF.pPos.Xd(2)   LF.pPos.Xd(5)];
        zl = [LF.pPos.Xd(3)   LF.pPos.Xd(6)];
        
        pl = line(xl,yl,zl);
        pl.Color = 'g';
        pl.LineStyle = '-';
        pl.LineWidth = 1;
    end
    
    %% Simulation timeout
    if toc(t)> timeout
        disp('Timeout man!');
        break
    end
    
    
end

%% Close file 
% fclose(Arq);

%%  Stop/Land robot
% Pioneer /////////////////////////////////////////////

% define control signal null
P.pSC.Ud = [0 ; 0];
% Send to network (a few times to be sure)
for ii = 1:5
    Rede.mSendMsg(P);
end

% ArDrone ///////////////////////////////////////////
% Land drone
if A.pFlag.Connected == 1
    A.rLand;
end


%% Plot results
% figure;
plotResults(P,A,data);

% End Of Code x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x