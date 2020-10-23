%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [ xf yf zf rho alfa beta ]
% Initial Comands

clear; close all; clc;
try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes
% Robots
P = Pioneer3DX(1);
A = ArDrone(61);

A.pPar.Ts = 0.030;
P.pPar.Ts = 0.030;

% Formation 3D
LF = LineFormationControl;

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;

% ArDrone
% A.rConnect;
% A.rTakeOff;
% t = tic;
% t_control = tic;
% disp('Start Take Off Timming....');
% while toc(t) < inf
%     if toc(t_control) > 0.030
%         A = J.mControl(A);
%         A.rSendControlSignals;
%     end
% end
disp('Taking Off End Time....');

%% Robots initial pose
idP = getID(OPT,P);            % pioneer ID on optitrack
idA = getID(OPT,A);            % drone ID on optitrack

rb = OPT.RigidBody;            % read optitrack data

A = getOptData(rb(idA),A);     % get ardrone data
P = getOptData(rb(idP),P);     % get pioneer data

%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
LF.pPos.Xr = LF.pPos.X;
LF.mDirTrans;

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 0.030;

rX = 1.0;      % [m]
rY = 1.0;      % [m]
rho = 1.5;      % [m]
T = 45;         % [s]
Tf = 90;        % [s]
w = 2*pi/T;     % [rad/s]

T1 = 85;             % Lemniscata
T2 =  3.0 + T1;         % Aproximação + Emergency
T3 =  0.1 + T2;         % Andando com o Drone pousado
T4 =  2.0 + T3;         % Parando o Pioneer

caso = 1;

fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_integA = tic;
t  = tic;
t_control = tic;

while toc(t) < T4

    if toc(t_control) > T_CONTROL             
        
        t_control = tic;
        
        %% Trajectory Planner        
        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf; 
        X_traj = rX*sin(w*tp);
        Y_traj = rY*sin(2*w*tp);
        dX_traj = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);
        dY_traj = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);
        
        if toc(t) < T1     
            if caso == 1
                fprintf('\n\nCaso 1: Leminiscata\n');
                caso = 2;
            end
            LF.pPos.Qd(1) = X_traj;       % xF
            LF.pPos.Qd(2) = Y_traj;       % yF
            LF.pPos.Qd(4) = rho;          % rho
            LF.pPos.dQd(1) = dX_traj;     % dxF
            LF.pPos.dQd(2) = dY_traj;     % dyF

        elseif toc(t) > T1 && toc(t) < T2       
            if caso == 2
                fprintf('\n\nCaso 2: Aproximação + Emergency\n');
                caso = 3;
            end      
            LF.pPos.Qd(1) = X_traj;       % xF
            LF.pPos.Qd(2) = Y_traj;       % yF
            LF.pPos.Qd(4) = rho/2;        % rho
            LF.pPos.dQd(1) = dX_traj;     % dxF
            LF.pPos.dQd(2) = dY_traj;     % dyF
            A.rEmergency;
            
        elseif toc(t) > T2 && toc(t) < T3  
            if caso == 3
                fprintf('\n\nCaso 3: Andando com o Drone pousado\n');
                caso = 4;           
                A.rLand;
            end
            LF.pPos.Qd(1) = X_traj;       % xF
            LF.pPos.Qd(2) = Y_traj;       % yF
            LF.pPos.Qd(4) = rho/2;        % rho
            LF.pPos.dQd(1) = dX_traj;     % dxF
            LF.pPos.dQd(2) = dY_traj;     % dyF
        end
        
        %% Get Data
        
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        
        % Pioneer
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);  % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        P = getOptData(rb(idP),P); 
        
        % Ardrone
        A = getOptData(rb(idA),A);
		A.pPos.X(7:12) = (A.pPos.X(1:6)-A.pPos.Xa(1:6))/toc(t_integA);
        A.pSC.U = [1 ; -1 ; 1; -1].*[A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]./A.pPar.uSat; % populates actual control signal to save data
        t_integA = tic;
        
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];   % Posição dos membros da formação
        LF.mDirTrans;                                 % Transformada Direta X --> Q

        %% Control
%         LF.mFormationControl(toc(t_integF));
        LF.mFormationControl_NullSpace('P',toc(t_integF)); % 'P' ---> Position    'F' ---> Form
        t_integF = tic;
        
        % Ganhos
        
        % Ganhos NP (VALIDADOS)
%         LF.pPar.K1 = diag([   1.0    1.0    1.0    1.0   1.0    1.0   ]);     % FORA
%         LF.pPar.K2 = diag([   0.4    0.4    0.4    0.2   0.2    0.2   ]);     % DENTRO
        
        % Ganhos F (VALIDADOS)
        LF.pPar.K1 = diag([   20.0  20.0    20.0   2.0   2.0    2.0   ]);     % kinematic control gain  - controls amplitude
        LF.pPar.K2 = diag([    0.1   0.1     0.1   0.1   0.1    0.1   ]);     % kinematic control gain - control saturation
   
        % Ganhos P
%         LF.pPar.K1 = diag([   1.0   1.0     1.0    5.0   5.0    5.0   ]);     % kinematic control gain  - controls amplitude
%         LF.pPar.K2 = diag([   0.4   0.4     0.4    0.1   0.1    0.1   ]);     % kinematic control gain - control saturation
 
        LF.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        
        P.pPos.Xda = P.pPos.Xd;    % save previous posture
        P.pPos.Xd(1:2) = LF.pPos.Xr(1:2);             % Posição desejada
        P.pPos.Xd(7:8) = LF.pPos.dXr(1:2);            % Velocidade desejada
        
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        A.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);

% % %         A.pPos.Xd(6) = P.pPos.X(6);
% % %         A.pPos.Xd(12) = P.pPos.X(12);  % dPsi
            
            %% Save data

        % Variable to feed plotResults function
        data = [  data  ; P.pPos.Xd'     P.pPos.X'        P.pSC.Ud(1:2)'    P.pSC.U(1:2)'...
                          A.pPos.Xd'     A.pPos.X'        A.pSC.Ud'         A.pSC.U' ...
                          LF.pPos.Qd'    LF.pPos.Qtil'    LF.pPos.Xd'...
                          toc(t)];
           
% %         %   1 -- 12      13 -- 24     25 -- 26          27 -- 28 
% %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
% %             
% %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60    
% %             A.pPos.Xd'   A.pPos.X'    A.pSC.Ud'         A.pSC.U'
% %             
% %         %   61 -- 66     67 -- 72       73 -- 78       79
% %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];
   
        %% Send control signals to robots
        % Pioneer
        cgains = [ 0.1  0.1  0.75  0.75  0.75  0.75  0.10  0.03 ];
        P = fDynamicController(P,cgains);     % Pioneer Dynamic Compensator
        if toc(t) > T3
            if caso == 4
                fprintf('\n\nCaso 4: Parando Pioneer\n');
                caso = 5;
            end
            LF.pPos.Qd = LF.pPos.Q;
            LF.pPos.dQd = zeros(6,1);
            P.pSC.Ud = [0; 0];
        end
      
        Rede.mSendMsg(P);
        
        % Drone
        % The Gains must be given in the folowing order
        % Rolagem Arfagem e Guinada (cabeceo)
        % kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
        Agains =   [   0.3   3.00    0.3   3.00   10.00    2.00 ;  1   20  1   20   1   3.0]; % GANHOS NP
        
        A = cUnderActuatedController(A,Agains);  % ArDrone
        A = J.mControl(A);                       % joystick command (priority)        
        A.rSendControlSignals;       
    end
end
%% Send control signals
% P.pSC.Ud = [0.05  ;  0];
% 
% for ii = 1:50
%     Rede.mSendMsg(P);
% end
%% Send control signals
P.pSC.Ud = [0  ;  0];
for ii = 1:5        
    Rede.mSendMsg(P);
end
%% Land drone
if A.pFlag.Connected == 1
    A.rLand;                % Commando to Land Drone 
end
%% Plot results
% % figure;
plotResultsLineControl(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
