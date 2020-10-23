%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [xf yf zf rhof alfaf betaf]

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

T_CONTROL = 0.030; % colocar 0.40 talvez melhore

%% Load Classes
% Robots
for ii = 1:3
    P{ii} = Pioneer3DX(ii);
    P{ii}.pPar.Ts = T_CONTROL;
end
Pgains = [ 0.10  0.10  0.8  0.8  0.75  0.75  0.10  0.03 ];

A = ArDrone(61);
A.pPar.Ts = T_CONTROL;
Agains =   [   0.3   3.00    0.3   3.00   10.00    3.00 ;  1   20  1   20   1   3.0]; % GANHOS NP

% Joystick
J = JoyControl;

% Formation 3D
LF{1} = LineFormationControl;
LF{2} = LineFormationControl;

LF{1}.pPar.K1 = diag([   20.0  20.0    20.0   3.0   3.0    3.0   ]);     % kinematic control gain  - controls amplitude
LF{1}.pPar.K2 = diag([    0.1   0.1     0.1   0.1   0.1    0.1   ]);     % kinematic control gain - control saturation

LF{2}.pPar.K1 = diag([     1.0    1.0    1.0    40.0  40.0  40.0   ]);     % kinematic control gain  - controls amplitude
LF{2}.pPar.K2 = diag([     0.1    0.1    0.1    0.1   0.1   0.1   ]);


% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;

A.rConnect;
A.rTakeOff;
t = tic;
t_control = tic;
disp('Start Take Off Timming....');
while toc(t) < 10
    if toc(t_control) > T_CONTROL
        A = J.mControl(A);
        A.rSendControlSignals;
    end
end
disp('Taking Off End Time....');

%% Robots initial pose
% detect rigid body ID from optitrack
idP{1} = getID(OPT,P{1});            % pioneer ID on optitrack
idP{2} = getID(OPT,P{2})+1;          % pioneer ID on optitrack
idA = getID(OPT,A);                  % drone ID on optitrack
rb = OPT.RigidBody;                  % read optitrack data
P{1} = getOptData(rb(idP{1}),P{1});  % get pioneer data
P{2} = getOptData(rb(idP{2}),P{2});  % get pioneer data
A = getOptData(rb(idA),A);           % get ardrone data

%% Formation initial error
% Formation initial pose
LF{1}.pPos.X = [P{1}.pPos.X(1:3); A.pPos.X(1:3)];
LF{1}.pPos.Xr = LF{1}.pPos.X;

LF{2}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];
LF{2}.pPos.Xr = LF{2}.pPos.X;

% Formation initial pose
LF{1}.mDirTrans;
LF{2}.mDirTrans;

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization

rX = 1.0;           % [m]
rY = 1.0;           % [m]
rho = 1.00;         % [m]
rhoArDrone = 1.5;   % [m]
T = 50;             % [s]
Tf = 100;            % [s]
w = 2*pi/T;         % [rad/s]

T_FP = 50;          % [s]

T1 = 95;             % Lemniscata
T2 =  3.0 + T1;      % Aproximação + Emergency
T3 =  0.1 + T2;      % Andando com o Drone pousado
T4 =  2.0 + T3;      % Parando o Pioneer
T_LIDER = 70;

caso = 1;
lider = 1;
print = 0;

data = zeros(round(Tf/T_CONTROL),189);
inc = 0;
data1 = [];
data2 = [];

t_control = tic;

t_integ{1} = tic;
t_integ{2} = tic;
t_integA = tic;
t_lider = tic;

t  = tic;

while toc(t) < T4
    if toc(t_control) > T_CONTROL               
        t_control = tic;
        if toc(t_lider) > T_LIDER
            lider = 2;
            P{1}.pSC.Ud = [0; 0];
            if print == 1
                fprintf('\nTroca de Líder..............\n\n');
                print = 1;
            end
        end
            
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>2
            P{1}.pSC.U  = Rede.pMSG.getFrom{4}(29:30);  % current velocities (robot sensors)
            P1X       = Rede.pMSG.getFrom{4}(14+(1:12));   % current position (robot sensors)
            
            P{2}.pSC.U  = Rede.pMSG.getFrom{5}(29:30);  % current velocities (robot sensors)
            P2X       = Rede.pMSG.getFrom{5}(14+(1:12));   % current position (robot sensors)

        end

        % Get optitrack data
        rb = OPT.RigidBody;                   % read optitrack data
        P{1} = getOptData(rb(idP{1}),P{1});   % get pioneer data
        P{2} = getOptData(rb(idP{2}),P{2});   % get pioneer data
        
        A = getOptData(rb(idA),A);
		A.pPos.X(7:12) = (A.pPos.X(1:6)-A.pPos.Xa(1:6))/toc(t_integA);
        A.pSC.U = [1 ; -1 ; 1; -1].*[A.pPos.X(4);A.pPos.X(5);A.pPos.X(9);A.pPos.X(12)]./A.pPar.uSat; % populates actual control signal to save data
        t_integA = tic;
        
        if lider == 1
            LF{1}.pPos.X = [P{1}.pPos.X(1:3); A.pPos.X(1:3)];
        else
            LF{1}.pPos.X = [P{2}.pPos.X(1:3); A.pPos.X(1:3)];
        end
        LF{1}.mDirTrans;
        
        LF{2}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];  % Posição dos membros da formação
        LF{2}.mDirTrans;                                      % Transformada Direta X --> Q       

        %% Trajectory

        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf; 
        X_traj = rX*sin(w*tp);
        Y_traj = rY*sin(2*w*tp);
        dX_traj = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);
        dY_traj = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);
        
        LF{2}.pPos.Qd(1) = X_traj;                        % xF
        LF{2}.pPos.Qd(2) = Y_traj;                        % yF
        LF{2}.pPos.Qd(4) = rho;                           % rho
        LF{2}.pPos.Qd(5) = pi/4;                          % alpha (+frente/-trás)
        LF{2}.pPos.Qd(6) = pi/4;                          % beta  (lateral  -dir/+ esq)

        LF{2}.pPos.dQd(1) = dX_traj;                      % dxF
        LF{2}.pPos.dQd(2) = dY_traj;                      % dyF
        
        LF{1}.pPos.dQd(1) = dX_traj;                      % dxF
        LF{1}.pPos.dQd(2) = dY_traj;                      % dyF
       
        if toc(t) < T1     
            if caso == 1
                fprintf('\n\nCaso 1: Leminiscata\n');
                caso = 2;
            end
            LF{1}.pPos.Qd(1) = X_traj;       % xF
            LF{1}.pPos.Qd(2) = Y_traj;       % yF
            LF{1}.pPos.Qd(4) = rhoArDrone;   % rho

        elseif toc(t) > T1 && toc(t) < T2       
            if caso == 2
                fprintf('\n\nCaso 2: Aproximação + Emergency\n');
                caso = 3;
            end      
            LF{1}.pPos.Qd(1) = X_traj;       % xF
            LF{1}.pPos.Qd(2) = Y_traj;       % yF
            LF{1}.pPos.Qd(4) = rhoArDrone/2; % rho
            A.rEmergency;
            
        elseif toc(t) > T2 && toc(t) < T3  
            if caso == 3
                fprintf('\n\nCaso 3: Andando com o Drone pousado\n');
                caso = 4;           
                A.rLand;
            end
            LF{1}.pPos.Qd(1) = X_traj;       % xF
            LF{1}.pPos.Qd(2) = Y_traj;       % yF
            LF{1}.pPos.Qd(4) = rhoArDrone/2; % rho
        end

        LF{1}.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        LF{2}.mInvTrans;                                 % Transformada Inversa Qd --> Xd

        %% Control
        
        % Formation Control      
        LF{1}.mFormationControl_NullSpace('F',toc(t_integ{1})); % 'P' ---> Position    'F' ---> Form
        t_integ{1} = tic;
        
        LF{2}.mFormationControl_NullSpace('P',toc(t_integ{2})); % 'P' ---> Position    'F' ---> Form
        t_integ = tic;
           
        % Pioneer       
        
        P{1}.pPos.Xda = P{1}.pPos.Xd;                       % save previous posture
        P{1}.pPos.Xd(1:2) = LF{1}.pPos.Xr(1:2);             % Posição desejada
        P{1}.pPos.Xd(7:8) = LF{1}.pPos.dXr(1:2);            % Velocidade desejada
        
        P{2}.pPos.Xda = P{2}.pPos.Xd;                       % save previous posture
        
        if lider == 1
            P{2}.pPos.Xd(1:2) = LF{2}.pPos.Xr(4:5);             % Posição desejada
            P{2}.pPos.Xd(7:8) = LF{2}.pPos.dXr(4:5);            % Velocidade desejada
        else
            P{2}.pPos.Xd(1:2) = LF{1}.pPos.Xr(1:2);             % Posição desejada
            P{2}.pPos.Xd(7:8) = LF{1}.pPos.dXr(1:2);            % Velocidade desejada
            LF{2}.pPos.Q = zeros(6,1);
            LF{2}.pPos.Qd = zeros(6,1);
            LF{2}.pPos.Xd = zeros(6,1);
            LF{2}.pPos.dQd = zeros(6,1);
        end
              
        A.pPos.Xda = A.pPos.Xd;    % save previous posture
        A.pPos.Xd(1:3) = LF{1}.pPos.Xr(4:6);
        A.pPos.Xd(7:9) = LF{1}.pPos.dXr(4:6);
        
        % Controlador Dinâmico       
        P{1} = fDynamicController(P{1},Pgains);
        P{2} = fDynamicController(P{2},Pgains);
  
        %% Save data (.txt file)        
        % Variable to feed plotResults function
        inc = inc + 1;
        
        data1 = [  data1  ; P{1}.pPos.Xd'     P{1}.pPos.X'        P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)'...
                            A.pPos.Xd'        A.pPos.X'           A.pSC.Ud'            A.pSC.U' ...
                            LF{1}.pPos.Qd'    LF{1}.pPos.Qtil'    LF{1}.pPos.Xd'...
                            toc(t)];
        
        data2 = [  data2  ; P{2}.pPos.Xd'     P{2}.pPos.X'        P{2}.pSC.Ud(1:2)'    P{2}.pSC.U(1:2)'...
                            A.pPos.Xd'        A.pPos.X'           A.pSC.Ud'            A.pSC.U' ...
                            LF{2}.pPos.Qd'    LF{2}.pPos.Qtil'    LF{2}.pPos.Xd'...
                    toc(t)];
        
%         data(inc,:) = [   P{1}.pPos.Xd'     P{1}.pPos.X'        P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)' ...
%                           P{2}.pPos.Xd'     P{2}.pPos.X'        P{2}.pSC.Ud(1:2)'    P{2}.pSC.U(1:2)' ...
%                           P{3}.pPos.Xd'     P{3}.pPos.X'        P{3}.pSC.Ud(1:2)'    P{3}.pSC.U(1:2)' ...
%                           A.pPos.Xd'        A.pPos.X'           A.pSC.Ud'            A.pSC.U' ...
%                           LF{1}.pPos.Qd'    LF{1}.pPos.Q'       LF{1}.pPos.Qtil'     LF{1}.pPos.Xd' ...
%                           LF{2}.pPos.Qd'    LF{2}.pPos.Q'       LF{2}.pPos.Qtil'     LF{2}.pPos.Xd' ...
%                           LF{3}.pPos.Qd'    LF{3}.pPos.Q'       LF{3}.pPos.Qtil'     LF{3}.pPos.Xd' ...
%                           toc(t)];
                      
        if toc(t) > T3
            if caso == 4
                fprintf('\n\nCaso 4: Parando Pioneer\n');
                caso = 5;
            end

            for ii=1:2
                P{ii}.pSC.Ud = [0; 0];
                LF{ii}.pPos.Qd = LF{ii}.pPos.Q;
                LF{ii}.pPos.dQd = zeros(6,1);
            end
        end
     
        % Send control signals to robots

        for ii=1:2
            Rede.mSendMsg(P{ii});
        end
        A = cUnderActuatedController(A,Agains);  % ArDrone
        A = J.mControl(A);                       % joystick command (priority)        
        A.rSendControlSignals;  
        
%         if toc(t_control) > T_CONTROL 
%             disp(['Estourou o tempo: ',num2str(1000*(toc(t_control)-T_CONTROL))]); 
%         end
    end
end

fprintf('\nFim..............\n\n');

%% Land drone
if A.pFlag.Connected == 1
    A.rLand;                % Commando to Land Drone 
end

%% Send control signals
% % P{1}.pSC.Ud = [.05  ;  0];
% % P{2}.pSC.Ud = [.05  ;  0];
% % 
% % for ii = 1:100
% %     Rede.mSendMsg(P{1});
% %     Rede.mSendMsg(P{2});
% % end

% P{1}.pSC.Ud = -[.1  ;  0];
% P{2}.pSC.Ud = -[.1  ;  0];
% 
% for ii = 1:100
%     Rede.mSendMsg(P{1});
%     Rede.mSendMsg(P{2});
% end

%% Send control signals
P{1}.pSC.Ud = [0  ;  0];
P{2}.pSC.Ud = [0  ;  0];
    
for ii = 1:100
    Rede.mSendMsg(P{1});    
        Rede.mSendMsg(P{2});
    Rede.mSendMsg(P{3});   
end

%% Plot results
% % figure;
% plotResultsLineControl_v2(data);
% plotResultsLineControl(data2);

% plotResultsLineControl(data1);
% plotResultsLineControl(data2);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
