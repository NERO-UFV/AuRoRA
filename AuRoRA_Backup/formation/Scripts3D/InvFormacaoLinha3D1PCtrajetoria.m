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
% Robot class
P = Pioneer3DX(1);
A = ArDrone(2);

% Formation class
LF = LineFormation3D;

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_FormacaoLinha3D')
Arq = fopen(['FL3d_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Robot/Simulator conection
% P.rConnect;
% A.rConnect;

%% Robots initial pose
% Pioneer 3DX
Xo = [-4 6 0 0];
P.rSetPose(Xo');

% ArDrone
% A.rTakeOff;
% A.pPos.X(3) = 0.8;
A.pPos.X(1:3) = [5 -4 .75];

shg              % show graphic window
% pause(1)
disp('Start..............');

%% Variable initialization
data = [];
% Desired formation [xf yf zf rhof alfaf betaf]
% xsq = 1.2;
% ysq = 1.24;
% zsq = 0;
% LF.pPos.Qd = [3*xsq 3*ysq xsq 0]';
LF.pPos.Qd = [0 0 0 3 deg2rad(30) deg2rad(60)]'; 

%% Formation initial error
% Formation members initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

%Formation Error
LF.mDirTrans;
LF.mFormationError;

% Robot Error
LF.mInvFormationError;

%% Configure simulation window
fig = figure(1);
% plot robots desired position
h1 = plot3(LF.pPos.Xd(1),LF.pPos.Xd(2),LF.pPos.Xd(3),'rsq','LineWidth',2);,hold on;
h2 = plot3(LF.pPos.Xd(4),LF.pPos.Xd(5),LF.pPos.Xd(6),'bsq','LineWidth',2);
  % plot  formation line 
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
pause

%% Simulation
% Maximum error permitted
erroMax = [.1 .1 0 .1 deg2rad(.1) deg2rad(.1)];

% Time variables initialization
timeout = 360;   % maximum simulation duration
t  = tic;
tc = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle
ttt = tic;
tdelay = tic;


% Loop while error > erroMax
while abs(LF.pPos.Qtil(1))>erroMax(1) || abs(LF.pPos.Qtil(2))>erroMax(2) || ...
        abs(LF.pPos.Qtil(4))>erroMax(4)|| abs(LF.pPos.Qtil(5))>erroMax(5) ...
        || abs(LF.pPos.Qtil(6))>erroMax(6) || toc(t)> timeout % formation errors
    
    if toc(tc) > 0.1
        
        tc = tic;
        %Abri Parênteses
        %         % Trajetória desejada
        if toc(tdelay) > 60
        tt = toc(ttt);
        a = 1.5;
        b = 1.5;
        w = 0.007;
        LF.pPos.Qd(1) = a*sin(2*pi*w*tt) ;           % x
        LF.pPos.Qd(2) = b*sin(2*pi*2*w*tt);          % y
        LF.pPos.Qd(3) = 0;    % z
        LF.pPos.dXr(1) = a*2*pi*w*cos(2*pi*w*tt);     % dx
        LF.pPos.dXr(2) = b*2*pi*2*w*cos(2*pi*2*w*tt); % dy
        LF.pPos.dXr(3) = 0;  % dz
        else
        LF.pPos.Qd = [0 0 0 3 deg2rad(30) deg2rad(60)]';  
        LF.pPos.dXr(1) = 0;     % dx
        LF.pPos.dXr(2) = 0; % dy
        LF.pPos.dXr(3) = 0;  % dz
        ttt = tic;
        end
        %Fechei Parênteses
        
        
        % Acquire sensors data (pPos.Xs = pPos.X)
        P.rGetSensorData;    % Pioneer
        A.rGetSensorData;    % ArDrone
        
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3);];
        
        % Formation Controller 
        LF.mInvFormationControl;
        
        %Formation Error
        LF.mDirTrans;
        LF.mFormationError;
        
%         display(LF.pPos.Qtil)
        
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
        
        % Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Variable to feed plotResults function
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)];
        
    end
      
    %% Dynamic Compensation/Control ....................................................
    % Pioneer 
    if toc(t1) > P.pPar.Ts
        t1 = tic;
        P = fCompensadorDinamico(P);      % Pioneer Dynamic Compensator
          % ------------------------------------------------
          %     % If not using dynamic compensation
          %     P.pSC.Ud = P.pSC.Ur;              % Pioneer
          % -------------------------------------------------
     % Send Control Signal
     
    P.rSendControlSignals;            % Pioneer
         end
    % ArDrone 
    if toc(t2) > A.pPar.Ts
        t2 = tic;
        A = cUnderActuatedController(A);  % ArDrone
          % ------------------------------------------------
          % If not using dynamic compensation
%               A.pSC.Ud = A.pSC.Ur;              % ArDrone
          % --------------------------------------
          % Tentativa de ajuste de psi
%           A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7));
          
           % Send Control Signal
            A.rSendControlSignals;            % ArDrone   
           
           
    end
  
    % Draw robot
    if toc(tp) > 0.1
        tp = tic;
        
        P.mCADdel                     % Pioneer
%         P.mCADplot2D('b')             % Pioneer trianglinho
        P.mCADplot(1,'k');             % Pioneer modelo bonitão
        A.mCADplot                    % ArDrone
        
        grid on
        drawnow
        %testando
        delete(h1);
        delete(h2);
        delete(pl);
% plot robots desired position
h1 = plot3(LF.pPos.Xd(1),LF.pPos.Xd(2),LF.pPos.Xd(3),'rsq','LineWidth',2);,hold on;
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
    
    % Simulation timeout
    if toc(t)> timeout
        disp('Timeout man!');
        break
    end
    
    
end

%% Close file and stop robot

fclose(Arq);
% Send control signals
P.pSC.Ud = [0; 0];
P.rSendControlSignals;    % Pioneer
% A.rLand;                  % ArDrone

% P.rDisconnect;          % Pioneer
% A.rDisconnect;          % ArDrone

% figure;
plotResults(P,A,data);

