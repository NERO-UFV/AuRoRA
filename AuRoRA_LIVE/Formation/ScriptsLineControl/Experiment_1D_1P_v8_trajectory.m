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
A = ArDrone(40);
A.pPar.Ts = 0.030;
% Formation 3D
LF = LineFormationControl;
% Joystick
J = JoyControl;
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
% Network
Rede = NetDataShare;

%% Network communication check
% % % % tm = tic;
% % % % while true
% % % %     
% % % %     if isempty(Rede.pMSG.getFrom)
% % % %         Rede.mSendMsg(P);
% % % %         if toc(tm) > 0.0150
% % % %             tm = tic;
% % % %             Rede.mReceiveMsg;
% % % %             disp('Waiting for message......')
% % % %         end
% % % %     elseif length(Rede.pMSG.getFrom) > 1
% % % %         if isempty(Rede.pMSG.getFrom{3})
% % % %             Rede.mSendMsg(P);
% % % %             tm = tic;
% % % %             Rede.mReceiveMsg;
% % % %             disp('Waiting for message......')
% % % %             
% % % %         else
% % % %             break
% % % %         end
% % % %     end
% % % % end
% % % % clc
% % % % disp('Data received. Continuing program...');

%% Robots initial pose
% detect rigid body ID from optitrack
idP = getID(OPT,P);          % pioneer ID on optitrack
idA = getID(OPT,A);            % drone ID on optitrack

rb = OPT.RigidBody;            % read optitrack data
A = getOptData(rb(idA),A);     % get ardrone data
P = getOptData(rb(idP),P);   % get pioneer data

% ArDrone
A.rConnect;

% Send Comand do Take off     
A.rTakeOff
tp = tic;
tc = tic;
disp('Start Take Off Timming....');
while toc(tp) < 10
    if toc(tc) > 0.030
        tc = tic;
        A = J.mControl(A);
        A.rSendControlSignals;
    end
end
disp('Taking Off End Time....');

%% Variable initialization
data = [];

%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
LF.pPos.Xr = LF.pPos.X;
% Formation initial pose
LF.mDirTrans;

%% Simulation

fprintf('\nStart..............\n\n');
% Time variables initialization
T_FORMATION = 0.030;
T_PIONEER = 0.030;
T_ARDRONE = 0.030;

rX = 1.0;       % [m]
rY = 1.0;       % [m]
rho = 1.5;      % [m]
T = 100;         % [s]
Tf = 200;       % [s]
w = 2*pi/T;     % [rad/s]

T1 = 180.0;             % Lemniscata
T2 =  10.0 + T1;        % Aproximação + Emergency
T3 =  0.1 + T2;         % Andando com o Drone pousado
T4 =  5.0 + T3;         % Parando o Pioneer

% % % rX = 1.0;       % [m]
% % % rY = 1.0;       % [m]
% % % rho = 1.5;      % [m]
% % % T = 75;         % [s]
% % % Tf = 150;       % [s]
% % % w = 2*pi/T;     % [rad/s]
% % % 
% % % T1 = 135.0;             % Lemniscata
% % % T2 =  15.0 + T1;        % Aproximação + Emergency
% % % T3 =  0.1 + T2;         % Andando com o Drone pousado
% % % T4 =  5.0 + T3;         % Parando o Pioneer

T_FP = 40;      % [s]
T_FA = 40;      % [s]

caso = 1;

t_Formation = tic;      % Formation cycle
t_Pioneer = tic;        % Pioneer cycle
t_ArDrone = tic;        % Ardrone cycle
t_integF = tic;
t_integA = tic;
t  = tic;

while toc(t) < T4

    if toc(t_Formation) > T_FORMATION             
        t_Formation = tic;
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        % Pioneer
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{3}(29:30);  % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{3}(14+(1:12));   % current position (robot sensors)
        end
        P = getOptData(rb(idP),P); 
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];   % Posição dos membros da formação
        LF.mDirTrans;                                 % Transformada Direta X --> Q
        % Ardrone
        A = getOptData(rb(idA),A);
		A.pPos.X(7:12) = (A.pPos.X(1:6)-A.pPos.Xa(1:6))/toc(t_integF);
        A.pSC.U = [1 ; -1 ; 1; -1].*[A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]./A.pPar.uSat; % populates actual control signal to save data
        
        %% Trajectory      
        if toc(t) < T1
            
            if caso == 1
                fprintf('\n\nCaso 1: Leminiscata\n');
                caso = 2;
            end
            t_traj = toc(t);
            a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
            tp = a*Tf;
       
            LF.pPos.Qd(1) = rX*sin(w*tp);                   % xF
            LF.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
            LF.pPos.Qd(3) = 0.00;                           % zF
            LF.pPos.Qd(4) = rho;                            % rho
            LF.pPos.Qd(5) = 0.00;                           % alpha (frente/trás)
            LF.pPos.Qd(6) = 0.00;                           % beta  (lateral)
            
            LF.pPos.dQd(1) = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);     % dxF
            LF.pPos.dQd(2) = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);      % dyF
            LF.pPos.dQd(3) = 0.00;                          % dzF
            LF.pPos.dQd(4) = 0.00;                          % drho
            LF.pPos.dQd(5) = 0.00;                          % dalpha
            LF.pPos.dQd(6) = 0.00;                          % dbeta
       
        elseif toc(t) > T1 && toc(t) < T2
            
            if caso == 2
                fprintf('\n\nCaso 2: Aproximação + Emergency\n');
                caso = 3;
            end
            
            t_traj = toc(t);
            a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
            tp = a*Tf;
            
            LF.pPos.Qd(1) = rX*sin(w*tp);                   % xF
            LF.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
            LF.pPos.Qd(3) = 0.00;                           % zF
            LF.pPos.Qd(4) = 0.50;                           % rho
            LF.pPos.Qd(5) = 0.00;                           % alpha (frente/trás)
            LF.pPos.Qd(6) = 0.00;                           % beta  (lateral)
            
            LF.pPos.dQd(1) = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);     % dxF
            LF.pPos.dQd(2) = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);      % dyF
            LF.pPos.dQd(3) = 0.00;                          % dzF
            LF.pPos.dQd(4) = 0.00;                          % drho
            LF.pPos.dQd(5) = 0.00;                          % dalpha
            LF.pPos.dQd(6) = 0.00;                          % dbeta
            
            A.rEmergency;
            
        elseif toc(t) > T2 && toc(t) < T3
            
            if caso == 3
                fprintf('\n\nCaso 3: Andando com o Drone pousado\n');
                caso = 4;           
                A.rLand;
            end
            
            t_traj = toc(t) - Tf;
            a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
            tp = a*Tf;
            
            LF.pPos.Qd(1) = rX*sin(w*tp);                   % xF
            LF.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
            LF.pPos.Qd(3) = 0.00;                           % zF
            LF.pPos.Qd(4) = 0.50;                           % rho
            LF.pPos.Qd(5) = 0.00;                           % alpha (frente/trás)
            LF.pPos.Qd(6) = 0.00;                           % beta  (lateral)
            
            LF.pPos.dQd(1) = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);     % dxF
            LF.pPos.dQd(2) = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);      % dyF
            LF.pPos.dQd(3) = 0.00;                          % dzF
            LF.pPos.dQd(4) = 0.00;                          % drho
            LF.pPos.dQd(5) = 0.00;                          % dalpha
            LF.pPos.dQd(6) = 0.00;                          % dbeta
          
        end
        
        LF.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        
        P.pPos.Xd(1:2) = LF.pPos.Xr(1:2);             % Posição desejada
        P.pPos.Xd(7:8) = LF.pPos.dXr(1:2);            % Velocidade desejada
        
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        A.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
        A.pPos.Xd(7:9) = LF.pPos.dXr(4:6);
        
        A.pPos.Xd(1:2) = LF.pPos.Qd(1:2);
        A.pPos.Xd(3) = 1.5;
        A.pPos.Xd(7:9) = LF.pPos.dQd(1:3);

% % %         A.pPos.Xd(6) = P.pPos.X(6);
% % %         A.pPos.Xd(12) = P.pPos.X(12);  % dPsi
        
        
        %% Control
        LF.mFormationControl(toc(t_integF));
%         LF.mFormationControl_NullSpace('F',toc(t_integF)); % 'P' ---> Position    'F' ---> Form
        t_integF = tic;
        
        % Ganhos
        LF.pPar.K1 = diag([   5.0    5.0    5.0    2.0   1.0    1.0   ]);     % kinematic control gain  - controls amplitude
        LF.pPar.K2 = diag([   0.1    0.1    0.1    0.1   0.1    0.1   ]);     % kinematic control gain - control saturation
        
%         LF.pPar.K1 = diag([   15.0   15.0   15.0   15.0  10.0  10.0   ]);     % kinematic control gain  - controls amplitude
%         LF.pPar.K2 = diag([   0.1    0.1    0.1    0.1   0.1   0.1   ]);     % kinematic control gain - control saturation
     
        %% Simulando Falhas
        
        % Pioneer
%         if toc(t) > T_FP && toc(t) < T_FP + 40
% 
%             P.pSC.Ud = [0; 0];
%             
%         end
        
        % Drone
        
% % %         if toc(t) > T_FA && toc(t) < T_FA + 0.1
% % %             
% % % %              
% % %               fprintf('\n\nCaso 4: Parando o Drone\n');
% % % 
% % %             
% % %         end
    
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
   
    end
    %% Send control signals to robots
    if toc(t_Pioneer) > T_PIONEER
        t_Pioneer = tic;
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{3}(29:30);  % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{3}(14+(1:12));   % current position (robot sensors)
        end
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        P = getOptData(rb(idP),P); 
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];   % Posição dos membros da formação
        LF.mDirTrans;                                 % Transformada Direta X --> Q
%         cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];
        cgains = [ 0.1  0.05  0.75  0.75  0.75  0.75  0.05  0.005 ];
        P = fDynamicController(P,cgains);     % Pioneer Dynamic Compensator

        if toc(t) < 2
            P.pSC.Ud = [0; 0];
        elseif toc(t) > T3
            if caso == 4
                fprintf('\n\nCaso 4: Parando Pioneer\n');
                caso = 5;
            end
            LF.pPos.Qd = LF.pPos.Q;
            LF.pPos.dQd = zeros(6,1);
            P.pSC.Ud = [0; 0];
        end
        Rede.mSendMsg(P);
    end
        
    if toc(t_ArDrone) > T_ARDRONE
        t_ArDrone = tic;
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        A = getOptData(rb(idA),A);
        A.pPos.X(7:12) = (A.pPos.X(1:6)-A.pPos.Xa(1:6))/toc(t_integA);
        A.pSC.U = [1 ; -1 ; 1; -1].*[A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]./A.pPar.uSat; % populates actual control signal to save data
        t_integA = tic;
        % The Gains must be given in the folowing order
        % Rolagem Arfagem e Guinada (cabeceo)
        % kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
%         Agains = [   0.50    2.00    0.50   2.00   5.00    2.00 ;   1   20   1   15   1   2.5];% Default
%         Agains =   [   0.05    1.00    0.05   1.00   2.50    1.00 ;  .5   10   .5   7.5   .5   1.5]; % GANHOS QUENTE PELANDO
        Agains =   [  0.3   2.0      0.3   2.0      5.0   2.0 ;  1   20   1   15   1   2.5]; % GANHOS QUENTE PELANDO
               
        A = cUnderActuatedController(A,Agains);  % ArDrone
        A = J.mControl(A);                       % joystick command (priority)
        A.rSendControlSignals;
    end

end

%% Send control signals
% % P.pSC.Ud = [-0.05  ;  0];
% % 
% % for ii = 1:50
% %     Rede.mSendMsg(P);
% % end

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
