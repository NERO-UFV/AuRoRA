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

A = ArDrone(2);     % load ardrone class

% The Gains must be given in the folowing order
% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
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
Arq = fopen(['FL3d_TrajElipseUltimate' NomeArq '.txt'],'w');
cd(PastaAtual)

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
A.rConnect;                 % ardrone

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
a  = 1;             % x distance
b  = 0.5;           % y distance
w  = 0.15;          % angular velocity for pioneer

%% Variable initialization
data = [];
% Desired formation [xf yf zf rhof alfaf betaf]
Qd = [1 1 0 1.5 -pi/2 pi/3]';
LF.pPos.Qd = Qd;                % initial desired position

% Robots desired pose
LF.mInvTrans;

% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];  % robos variables  
LF.mDirTrans;                                % calculate formation variables

% Formation Error
LF.mFormationError;

%% Simulation
% Time variables initialization
flyflag = 1;             % flying = 1 
nlap    = 3;             % number of laps 
tsim    = 2*pi*nlap/w;   % trajectory duration time
t       = tic;           % current time
tc      = tic;           % robots sample time  
tp      = tic;           % plot update time

% Loop controlled by time
while toc(t) < tsim 
    if toc(tc) > 1/30
        tc = tic;
        %% Trajectory changes
        % Changes the desired variables 
        if toc(t) > 2*tsim/nlap 
            LF.pPos.Qd(4) = 1;     % rho
            Qd(5) = -pi/2;         % alpha angle                       
            LF.pPos.Qd(6) = Qd(6); % beta angle            
        elseif toc(t) > tsim/nlap  
            LF.pPos.Qd(4) = 1;     % rho
            Qd(5) = 0;             % alpha angle                       
            LF.pPos.Qd(6) = pi/2;  % beta angle            
        end
        % Landing and taking off
        if toc(t) > 2*tsim/nlap
            if flyflag == 0
                A.rTakeOff;               % takeoff again 
                flyflag = 1;              % acreditando que irá decolar de prima
            end
        elseif toc(t) > tsim/nlap + 8     % landing
           if flyflag ==1
                A.rLand;
                flyflag = 0; 
            end
        end
        
        %% Trajectory
        % Elipse
        ta = toc(t);
        % Positions
        LF.pPos.Qd(1)  = a*cos(w*ta);          % x position
        LF.pPos.Qd(2)  = b*sin(w*ta);          % y position
        LF.pPos.Qd(5)  = Qd(5) + P.pPos.X(6);  % alpha angle
        % Quadrant correction
        if abs(LF.pPos.Qd(5)) > pi
            if LF.pPos.Qd(5) > 0
                LF.pPos.Qd(5) = LF.pPos.Qd(5) - 2*pi;
            else
                LF.pPos.Qd(5) = LF.pPos.Qd(5) + 2*pi;
            end
        end
        % Velocities
        LF.pPos.dQd(1) = -a*w*sin(w*ta);  % x velocity
        LF.pPos.dQd(2) = b*w*cos(w*ta);   % y velocity
        LF.pPos.dQd(5) = 0;               % alpha velocity
        
        %% Acquire sensors data
        % Get network data
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);     % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12)); % current position (robot sensors)
        end
        
        % Get optitrack data
        rb = OPT.RigidBody;            % read optitrack
        % Ardrone
        A = getOptData(rb(idA),A);
        A.pSC.U = [A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]; % populates actual control signal to save data
        P = getOptData(rb(idP),P);
        
        %% Control
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired positions 
        LF.mInvTrans;                            
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);       % desired velocities              
        % Drone
        A.pPos.Xda     = A.pPos.Xd;              % save previous posture
        A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);        % desired position
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);       % desired velocities
        A.pPos.Xd(6)   = 0; %atan2(A.pPos.Xd(8),A.pPos.Xd(7)); % desired Psi
        
%         % Derivative (dPsi)
%         if abs(A.pPos.Xd(6) - A.pPos.Xda(6)) > pi
%             if A.pPos.Xda(6) < 0
%                 A.pPos.Xda(6) =  2*pi + A.pPos.Xda(6);
%             else
%                 A.pPos.Xda(6) = -2*pi + A.pPos.Xda(6);
%             end
%         end
%         A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/(1/30);
        
        % Dynamic controllers
        P = fDynamicController(P);               % Pioneer Dynamic controller
        A = cUnderActuatedControllerMexido(A,gains);   % ArDrone
        A = J.mControl(A);                       % joystick command (priority)
        
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
       
end

%% Loop Landing
tland = 15;
while toc(t) < tsim+tland      
    if toc(tc) > 1/30
        tc = tic;
        %% Landing position
        ta = toc(t);
        % Positions
        LF.pPos.Qd(1)  = 0;                             % x position
        LF.pPos.Qd(2)  = 0;                             % y position
        LF.pPos.Qd(4)  = Qd(4) - (Qd(4)-0.5)*(ta-tsim)/(tland); % rho 
        LF.pPos.Qd(5)  = Qd(5) + P.pPos.X(6);           % alpha 
        LF.pPos.Qd(6)  = pi/2;                          % beta 
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
        % Ardrone
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
        P = fDynamicController(P);              % Pioneer Dynamic controller
        A = cUnderActuatedController(A,gains);  % ArDrone
        A = J.mControl(A);                      % joystick command (priority)
        
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');

        %% Send control signals to robots
        Rede.mSendMsg(P);            % send data to network
        A.rSendControlSignals;
    end    
end

%% Close files
fclose(Arq);

%% Send control signals to stop robots
% Land drone
if A.pFlag.Connected == 1
    A.rEmergency;
    A.rLand;
end

P.pSC.Ud = [0; 0];
% Send to network (a few times to be sure)
for ii = 1:5
    Rede.mSendMsg(P);
end

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx