%% 3D Line Formation Pioneer-Drone

% Pioneer is the reference of the formation
% The formation variables are:
% Q = [xf yf zf rhof alfaf betaf]

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  %

% Initial Comands

clear; close all; clc;
try
    fclose(instrfindall);
catch
end

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Look for root folder

PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Emergency settings

% Emmergency button
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'Emergency!', ...
    'Callback', 'btnEmergencia = 1;', ...
    'Position', [50 50 400 300]);

nLandMsg = 3; % Number of redundant landing messages.

%% Load Classes

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Robots
P{1} = Pioneer3DX(1);
P{2} = Pioneer3DX(2);

A{1} = ArDrone(3); % Laranja
A{2} = ArDrone(4); % Laranja

A{1}.pPar.ip = '192.168.1.61';
A{1}.rConnect;
A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216  ]';

% 
A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
A{2}.pPar.ip = '192.168.1.40';
A{2}.rConnect;
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216  ]';


% A{1} = ArDrone(61); % Laranja
% A{2} = ArDrone(50); % Azul
% A{2} = ArDrone(40); % Branco
% A{2} = ArDrone(1); % Kibon

% Network
Rede = NetDataShare;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Formation 3D
LF{1} = LineFormationControl;   % P1 + D1 | FORMA
LF{2} = LineFormationControl;   % P1 + P2 | FORMA
LF{3} = LineFormationControl;   % P2 + D2 | FORMA
LF{4} = LineFormationControl;   % D1 + D2 | FORMA

LF{2}.pPar.K1 = diag([  1.0   1.0   0.0   1.0    2.0   2.0  ]);     % kinematic control gain  - controls amplitude
LF{2}.pPar.K2 = diag([  0.5   0.5   0.0   0.5    0.3   0.3    ]);   % kinematic control gain - control saturation

for ii = [1 3]
    LF{ii}.pPar.K1 = diag([  1.0   1.0   0.0   1.0    2.0   2.0  ]);     % kinematic control gain  - controls amplitude
    LF{ii}.pPar.K2 = diag([  0.5   0.5   0.0   0.5    0.15   0.15  ]);   % kinematic control gain - control saturation
end

% LF{4}.pPar.K1 = diag([  1.0   1.0   0.0   1.0    2.0   2.0  ]);    % kinematic control gain  - controls amplitude
% LF{4}.pPar.K2 = diag([  0.5   0.5   0.0   0.5    0.15   0.15  ]);  % kinematic control gain - control saturation

LF{4}.pPar.K1 = diag([  1.0   1.0   0.0   0.5    2.0    2.0  ]);       % kinematic control gain  - controls amplitude
LF{4}.pPar.K2 = diag([  0.5   0.5   0.0   0.5    0.15   0.15  ]);     % kinematic control gain - control saturation

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

A{1}.rTakeOff;
A{2}.rTakeOff;

% A{1}.rLand;
A{2}.rLand;

t = tic;
t_control = tic;
T_CONTROL = 1/30;
disp('Start Take Off Timming....');

try
    while toc(t) < 1
        if toc(t_control) > T_CONTROL
            A{1} = J.mControl(A{1});
            A{1}.rSendControlSignals;

            A{2} = J.mControl(A{2});
            A{2}.rSendControlSignals;
        end
    end
    disp('Taking Off End Time....');
catch M
   
    fprintf('\n----------------------------   Code Error   ----------------------------\n');
    disp('');
    disp(M);
    disp('');
    
    fprintf('\n   ----> Bebop Landing through Try/Catch Loop Command\n');
    for i=1:nLandMsg
        A{1}.rLand;
        A{2}.rLand;
    end
end


%% Robots initial pose
% Pioneer P3DX
% detect rigid body ID from optitrack

idP{1} = getID(OPT,P{1});            % pioneer ID on optitrack
idP{2} = getID(OPT,P{2})+1;          % pioneer ID on optitrack
idA{1} = getID(OPT,A{1});            % drone ID on optitrack
idA{2} = getID(OPT,A{2})+1;          % drone ID on optitrack
rb = OPT.RigidBody;                  % read optitrack data
P{1} = getOptData(rb(idP{1}),P{1});  % get pioneer data
P{2} = getOptData(rb(idP{2}),P{2});  % get pioneer data
A{1} = getOptData(rb(idA{1}),A{1});  % get ardrone data
A{2} = getOptData(rb(idA{2}),A{2});  % get ardrone data

% Atribuindo posições desejadas   
Xd = zeros(6,1);
Xda = zeros(6,1);
dXd = zeros(6,1);
XTs = LF{1}.pPar.Ts;

% P1
P{1}.pPos.Xd(1:3) = Xd(1:3);         % Posição desejada
P{1}.pPos.Xd(7:9) = dXd(1:3);        % Velocidade desejada
% A1
A{1}.pPos.Xda = A{1}.pPos.Xd;   % save previous posture
A{1}.pPos.Xd(1:3) = Xd(1:3) + [ 0.0 ; 0.0 ; 1.00001 ]; % Posição desejada
A{1}.pPos.Xd(7:9) = dXd(1:3);

% P2
P{2}.pPos.Xd(1:3) = Xd(1:3) + [ 1.0 ; 0.0 ; 0.000001 ]; % Posição desejada
P{2}.pPos.Xd(7:9) = dXd(1:3);        % Velocidade desejada
% A2
A{2}.pPos.Xda = A{2}.pPos.Xd;   % save previous posture
A{2}.pPos.Xd(1:3) = Xd(1:3) + [ 1.0 ; 0.0 ; 1.0 ];
A{2}.pPos.Xd(7:9) = dXd(1:3);        % Velocidade desejada

%% Formation initial error
% Formation initial pose
LF{1}.pPos.X = [P{1}.pPos.X(1:3); A{1}.pPos.X(1:3)];
LF{1}.pPos.Xr = LF{1}.pPos.X;
LF{1}.mDirTrans;
LF{1}.pPos.Qda = LF{1}.pPos.Qd;

LF{2}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];
LF{2}.pPos.Xr = LF{2}.pPos.X;
LF{2}.mDirTrans;
LF{2}.pPos.Qda = LF{2}.pPos.Qd;

LF{3}.pPos.X = [P{2}.pPos.X(1:3); A{2}.pPos.X(1:3)];
LF{3}.pPos.Xr = LF{3}.pPos.X;
LF{3}.mDirTrans;
LF{3}.pPos.Qda = LF{3}.pPos.Qd;

LF{4}.pPos.X = [A{2}.pPos.X(1:3); A{1}.pPos.X(1:3)];
LF{4}.pPos.Xr = LF{4}.pPos.X;
LF{4}.mDirTrans;
LF{4}.pPos.Qda = LF{4}.pPos.Qd;

%% Configure simulation window

fig = figure();
axis([-3 3 -3 3 0 3]);
view(-21,30);
hold on;
grid on;

P{1}.mCADdel;
P{2}.mCADdel;
try
    delete(fig1);
    delete(fig2);
    delete(fig3);
    delete(fig4);
    delete(fig5);
    delete(square);
catch
end

P{1}.mCADplot(0.75,'k');
P{2}.mCADplot(0.75,'g');
A{1}.mCADcolor([0 0 1]);
A{1}.mCADplot;
A{2}.mCADcolor([0 0 0]);
A{2}.mCADplot;
drawnow;

%% Simulation
pause(3);

fprintf('\nStart..............\n\n');

% Time variables initialization

T_PLOT = 10;       % Período de plotagem em tempo real

T_FORMATION = LF{1}.pPar.Ts; % 200ms
T_PIONEER = P{1}.pPar.Ts; %0.100;
T_ARDRONE = A{1}.pPar.Ts; %1/30

rX = 1.0; % [m]
rY = 1.0; % [m]
T = 75;   % [s]
Tf = 150; % [s]
w = 2*pi/T; % [rad/s]

T1 = 115.0;             % Lemniscata
T2 =  15.0 + T1;        % Aproximação + Emergency
T3 =  15.1 + T2;        % Andando com o Drone pousado
T4 =  5.0 + T3;         % Parando o Pioneer

caso = 1;

% Data variables
kk = 1;
data = zeros(round(T4/T_FORMATION),217); % Data matrix

t  = tic;
t_plot = tic;

t_Formation = tic;      % Formation control

t_Pioneer_1 = tic;
t_Pioneer_2 = tic;

t_ArDrone_1 = tic;
t_ArDrone_2 = tic;

try
    while toc(t)< T4
        % =====================================================================
        % Pioneer

        if toc(t_Pioneer_1) > T_PIONEER
            t_Pioneer_1 = tic;
    %         disp('P1');
            rb = OPT.RigidBody;                   % read optitrack data
            P{1} = getOptData(rb(idP{1}),P{1});   % get pioneer data
            Rede.mReceiveMsg;
            try
                if length(Rede.pMSG.getFrom)>2
                    P{1}.pSC.U  = Rede.pMSG.getFrom{3}(29:30);  % current velocities (robot sensors)
                    P1X       = Rede.pMSG.getFrom{3}(14+(1:12));   % current position (robot sensors)
                end
            catch
                P{1}.pSC.U = P{1}.pSC.Ud;
            end
            P{1} = fDynamicController(P{1});    % Controlador        
            Rede.mSendMsg(P{1});           % Enviar sinal de controle para o robô
        end

        if toc(t_Pioneer_2) > T_PIONEER
            t_Pioneer_2 = tic;
    %         disp('P2');
            rb = OPT.RigidBody;                   % read optitrack data
            P{2} = getOptData(rb(idP{2}),P{2});   % get pioneer data
            Rede.mReceiveMsg;
            if length(Rede.pMSG.getFrom)>2
                P{2}.pSC.U  = Rede.pMSG.getFrom{4}(29:30);  % current velocities (robot sensors)
                P2X       = Rede.pMSG.getFrom{4}(14+(1:12));   % current position (robot sensors)
            end
            P{2} = fDynamicController(P{2});    % Pioneer Dynamic Compensator
            P{2}.pSC.Ud = [0;0];
            Rede.mSendMsg(P{2});           % Enviar sinal de controle para o robô
        end

        % =====================================================================
        % ArDrone   
        if toc(t_ArDrone_1) > T_ARDRONE
            t_ArDrone_1 = tic;
    %         disp('D1');
            A{1} = getOptData(rb(idA{1}),A{1});
            A{1}.pPos.X(7:12) = (A{1}.pPos.X(1:6)-A{1}.pPos.Xa(1:6))/A{1}.pPar.Ts;
            A{1}.pSC.U = [1 ; -1 ; 1; -1].*[A{1}.pPos.X(4);A{1}.pPos.X(5);A{1}.pPos.X(9);A{1}.pPos.X(12)]./A{1}.pPar.uSat;
            Agains =   [   0.1    2.00    0.1   2.00   5.00    2.00 ;  1   20   1   15   1   2.5]; % GANHOS QUENTE PELANDO
            A{1} = cUnderActuatedController(A{1},Agains);  % Controlador 
            A{1}.rSendControlSignals;               % Enviar sinal de controle para o robô
        end

        if toc(t_ArDrone_2) > T_ARDRONE
            t_ArDrone_2 = tic;
    %         disp('D2');
            A{2} = getOptData(rb(idA{2}),A{2});
            A{2}.pPos.X(7:12) = (A{2}.pPos.X(1:6)-A{2}.pPos.Xa(1:6))/A{2}.pPar.Ts;
            A{2}.pSC.U = [1 ; -1 ; 1; -1].*[A{2}.pPos.X(4);A{2}.pPos.X(5);A{2}.pPos.X(9);A{2}.pPos.X(12)]./A{2}.pPar.uSat;
            Agains =   [   0.1    2.00    0.1   2.00   5.00    2.00 ;  1   20   1   15   1   2.5]; % GANHOS QUENTE PELANDO
            A{2} = cUnderActuatedController(A{2},Agains);  % Controlador
            A{2}.rSendControlSignals;               % Enviar sinal de controle para o robô
        end

        % =====================================================================
        % Laço de controle de formação
        if toc(t_Formation) > T_FORMATION
            t_Formation = tic;

            % Trajectory Planner
            t_traj = toc(t);
            a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
            tp = a*Tf;
            Xda = Xd;
            Xd(1) = rX*sin(w*tp);                   % xF
            Xd(2) = rY*sin(2*w*tp);                 % yF
            Xd(3) = 0.00;                           % zF
            Xd(4) = 0.00;                           % rho
            Xd(5) = 0.00;                           % alpha (frente/trás)
            Xd(6) = 0.00;                           % beta  (lateral)
            dXd = (Xd - Xda)/XTs;

            % Formation instant variables and references
            LF{1}.pPos.X = [P{1}.pPos.X(1:3); A{1}.pPos.X(1:3)];    % P1 + A1 | POSIÇÃO
            LF{2}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];    % P1 + P2 | POSIÇÃO
            LF{3}.pPos.X = [P{2}.pPos.X(1:3); A{2}.pPos.X(1:3)];    % P2 + A2 | POSIÇÃO
            LF{4}.pPos.X = [A{2}.pPos.X(1:3); A{1}.pPos.X(1:3)];    % A2 + A1 | POSIÇÃO

            for ii = 1:4
                LF{ii}.pPos.Qda = LF{ii}.pPos.Qd;
            end

            LF{1}.pPos.Qd = Xd + [ 0.0 ; 0.0 ; 0.0 ;   1.0   ;  0.0  ; 0.0  ];        
            LF{2}.pPos.Qd = Xd + [ 0.0 ; 0.0 ; 0.0 ;   1.0   ;  pi/2 ; 0.0  ];
            LF{3}.pPos.Qd = Xd + [ 1.0 ; 0.0 ; 0.0 ;   1.0  ;  0.0  ; 0.0  ];
            LF{4}.pPos.Qd = Xd + [ 1.0 ; 0.0 ; 1.0 ;   1.0  ;  -pi/2 ; 0.0 ];


            for ii = 1:4
                LF{ii}.pPos.dQd = (LF{ii}.pPos.Qd - LF{ii}.pPos.Qda)/LF{ii}.pPar.Ts;
    %             LF{ii}.mFormationControl;
                LF{ii}.mFormationControlNSB('F');
                LF{ii}.mInvTrans;
            end

            % Atribuindo posições desejadas       
            % P1
            P{1}.pPos.Xd(1:3) = Xd(1:3);         % Posição desejada
            P{1}.pPos.Xd(7:9) = dXd(1:3);        % Velocidade desejada

            % A1
            A{1}.pPos.Xda = A{1}.pPos.Xd;   % save previous posture
            A{1}.pPos.Xd(1:3) = LF{1}.pPos.Xr(4:6); % Posição desejada
            A{1}.pPos.Xd(7:9) = LF{1}.pPos.dXr(4:6);
            
            A{1}.pPos.Xd(1:3) = Xd(1:3) + [0 ; 0 ; 1]; % Posição desejada
            A{1}.pPos.Xd(7:9) = dXd(1:3);

            % P2
            P{2}.pPos.Xd(1:3) = LF{2}.pPos.Xr(4:6); % Posição desejada
            P{2}.pPos.Xd(7:9) = LF{2}.pPos.dXr(4:6);        % Velocidade desejada

            % A2
            A{2}.pPos.Xda = A{2}.pPos.Xd;   % save previous posture
            A{2}.pPos.Xd(1:3) = LF{3}.pPos.Xr(4:6); %(LF{3}.pPos.Xr(4:6)  + LF{4}.pPos.Xr(4:6))/2;
            A{2}.pPos.Xd(7:9) = LF{3}.pPos.dXr(4:6); %(LF{3}.pPos.dXr(4:6) + LF{4}.pPos.dXr(4:6))/2;        % Velocidade desejada

            if toc(t) < T1     
                if caso == 1
                    fprintf('\n\nCaso 1: Leminiscata\n');
                    caso = 2;
                end
            end
            
            if toc(t) > T1 && toc(t) < T2
                if caso == 2
                    fprintf('\n\nCaso 2: Aproximação + Emergency\n');
                    caso = 3;
                end 
                if toc(t_ArDrone_1) > T_ARDRONE
                    LF{3}.pPos.Qd = Xd + [ 1.0 ; 0.0 ; 0.0 ;   0.5  ;  0.0  ; 0.0  ];
                    LF{4}.pPos.Qd = Xd + [ 1.0 ; 0.0 ; 0.5 ;   1.0  ;  -pi/2 ; 0.0 ];
                    A{1}.rEmergency;
                    A{2}.rEmergency;
                end    
            end

            if toc(t) > T2 && toc(t) < T3
                if caso == 3
                    fprintf('\n\nCaso 3: Andando com o Drone pousado\n');
                    caso = 4;
                    for i=1:nLandMsg
                        A{1}.rLand;
                        A{2}.rLand;
                    end
                end 
                if toc(t_ArDrone_1) > T_ARDRONE
                    LF{3}.pPos.Qd = Xd + [ 1.0 ; 0.0 ; 0.0 ;   0.5  ;  0.0  ; 0.0  ];
                    LF{4}.pPos.Qd = Xd + [ 1.0 ; 0.0 ; 0.5 ;   1.0  ;  -pi/2 ; 0.0 ];
                end    
            end
            
            if toc(t) > T3
                if caso == 4
                    fprintf('\n\nCaso 4: Parando Pioneer\n');
                    caso = 5;
                end

                for ii=1:3
                    LF{ii}.pPos.Qd = LF{ii}.pPos.Q;
                    LF{ii}.pPos.dQd = zeros(6,1);
                end
                
                for i=1:nLandMsg
                    for ii=1:2
                        P{ii}.pSC.Ud = [0  ;  0];
                        Rede.mSendMsg(P{ii});
                    end
                end
            end
            

            % Variable to feed plotResults function    
            data(kk,:) = [  P{1}.pPos.Xd'     P{1}.pPos.X'        P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)'...
                            P{2}.pPos.Xd'     P{2}.pPos.X'        P{2}.pSC.Ud(1:2)'    P{2}.pSC.U(1:2)'...
                            A{1}.pPos.Xd'     A{1}.pPos.X'        A{1}.pSC.Ud'         A{1}.pSC.U' ...
                            A{2}.pPos.Xd'     A{2}.pPos.X'        A{2}.pSC.Ud'         A{2}.pSC.U' ...
                            LF{1}.pPos.Qd'    LF{1}.pPos.Qtil'    LF{1}.pPos.Xr'       LF{1}.pPos.Xd'...
                            LF{2}.pPos.Qd'    LF{2}.pPos.Qtil'    LF{2}.pPos.Xr'       LF{2}.pPos.Xd'...
                            LF{3}.pPos.Qd'    LF{3}.pPos.Qtil'    LF{3}.pPos.Xr'       LF{3}.pPos.Xd'...
                            LF{4}.pPos.Qd'    LF{4}.pPos.Qtil'    LF{4}.pPos.Xr'       LF{4}.pPos.Xd'...
                            toc(t)];

                            kk = kk + 1;
            try
            catch
            % %         %   1 -- 12             13 -- 24             25 -- 26              27 -- 28
            % %             P{1}.pPos.Xd'       P{1}.pPos.X'         P{1}.pSC.Ud(1:2)'     P{1}.pSC.U(1:2)'
            % %         
            % %         %   29 -- 40            41 -- 52             53 -- 54              55 -- 56 
            % %             P{2}.pPos.Xd'       P{2}.pPos.X'         P{2}.pSC.Ud(1:2)'     P{2}.pSC.U(1:2)'
            % %         
            % %         %   57 -- 68            69 -- 80             81 -- 84              85 -- 88
            % %             A{1}.pPos.Xd'       A{1}.pPos.X'         A{1}.pSC.Ud'          A{1}.pSC.U'
            % %
            % %         %   89 -- 100           101 -- 112           113 -- 116            117 -- 120
            % %             A{2}.pPos.Xd'       A{2}.pPos.X'         A{2}.pSC.Ud'          A{2}.pSC.U'
            % %        
            % %         %   121 -- 126          127 -- 132           133 -- 138            139 -- 144
            % %             LF{1}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
            % %         
            % %         %   145 --          
            % %             LF{2}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
            % %        
            % %         %   169 --        
            % %             LF{3}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
            % %
            % %         %   193 --         
            % %             LF{4}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
            % %
            % %         %   217
            % %         %   toc(t)  ]
            end
        end

        
        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        if btnEmergencia ~= 0
            fprintf('\n----------------------------   Emergency   -----------------------------\n');
            fprintf('\n   ----> Bebop Landing through Emergency Command\n');
            fprintf('\n   ----> Pioneer Stopping through Emergency Command\n');

            % Send 3 times Commands 1 second delay to Drone Land
            fprintf('\n   ----> Bebop Landing through Try/Catch Loop Command\n');
            for i=1:nLandMsg
                A{1}.rLand;
                A{2}.rLand;
            end
            fprintf('\n   ----> Pioneer Stopping through Try/Catch Loop Command\n');
            for i=1:nLandMsg
                for ii=1:2
                    P{ii}.pSC.Ud = [0  ;  0];
                    Rede.mSendMsg(P{ii});
                end
            end
            break;
        end
        
        
        
        %% Draw robots
        if toc(t_plot) > T_PLOT
            t_plot = tic;
            % Pioneer
            P{1}.mCADdel;
            P{2}.mCADdel;

            try
                delete(fig1);
                delete(fig2);
                delete(fig3);
                delete(fig4);
                delete(fig5);
                delete(square);
                delete(squared);
            catch
            end

            P{1}.mCADplot(0.75,'k');
            P{2}.mCADplot(0.75,'g');
            A{1}.mCADplot;
            A{2}.mCADplot([0;0;0]);

            % Percourse made
            fig1 = plot3(data(2:kk-1,1),data(2:kk-1,2),data(2:kk-1,3),'k--','LineWidth',1.0);
            fig2 = plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
            fig3 = plot3(data(2:kk-1,41),data(2:kk-1,42),data(2:kk-1,43),'g-','LineWidth',0.5);
            fig4 = plot3(data(2:kk-1,69),data(2:kk-1,70),data(2:kk-1,71),'b-','LineWidth',0.5);
            fig5 = plot3(data(2:kk-1,101),data(2:kk-1,102),data(2:kk-1,103),'k-','LineWidth',0.5);
            square = plot3( data(kk-2,[13 41 101 69 13]),...
                            data(kk-2,[14 42 102 70 14]),...
                            data(kk-2,[15 43 103 71 15]), 'Color',[1 0.5 0],'LineWidth',1);

            squared = plot3( data(kk-2,[139 166 190 142 139]),...
                            data(kk-2,[140 167 191 143 140]),...
                            data(kk-2,[141 168 192 144 141]), 'Color',[1 0.5 0],'LineWidth',1,'LineStyle',':');                    
            drawnow;

            
            
        end
    end
catch M
    
    fprintf('\n----------------------------   Code Error   ----------------------------\n');
    disp('');
    disp(M);
    disp('');
    
    fprintf('\n   ----> Bebop Landing through Try/Catch Loop Command\n');
    for i=1:nLandMsg
        A{1}.rLand;
        A{2}.rLand;
    end
    fprintf('\n   ----> Pioneer Stopping through Try/Catch Loop Command\n');
    for i=1:nLandMsg
        for ii=1:2
            P{ii}.pSC.Ud = [0  ;  0];
            Rede.mSendMsg(P{ii});
        end
    end

end

axis equal;
set(gca,'Box','on');

%% Parando os robôs

for i=1:nLandMsg
    A{1}.rLand;
    A{2}.rLand;
end


%%
for i=1:nLandMsg
    for ii=1:2
        P{ii}.pSC.Ud = [0  ;  0];
        Rede.mSendMsg(P{ii});
    end
end

%%
% % % % for i=1:100
% % % %     for ii=1:2
% % % %         P{ii}.pSC.Ud = [0  ;  0.25];
% % % %         Rede.mSendMsg(P{ii});
% % % %     end
% % % % end
