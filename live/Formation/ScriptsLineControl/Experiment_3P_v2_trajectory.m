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

%% Load Classes
% Robots
P{1} = Pioneer3DX(1);
P{2} = Pioneer3DX(2);
P{3} = Pioneer3DX(3);

Pgains = [ 0.20  0.20  0.8  0.8  0.75  0.75  0.10  0.03 ];

% Formation 3D
LF{1} = LineFormationControl;
LF{2} = LineFormationControl;

% Prioridade Forma   
% LF1.pPar.K1 = 100*diag([   10.0   10.0   10.0   10.0  10.0  10.0   ]);     % kinematic control gain  - controls amplitude
% LF1.pPar.K2 = 100*diag([    0.1    0.1    0.1    0.1   0.1   0.1   ]);     % kinematic control gain - control saturation

% Prioridade Posição 
LF{1}.pPar.K1 = diag([     1.0   1.0   1.0     50.0   20.0  20.0   ]);     % kinematic control gain  - controls amplitude
LF{1}.pPar.K2 = diag([     0.1    0.1    0.1    0.1   0.1   0.1   ]);

LF{2}.pPar.K1 = LF{1}.pPar.K1;     % kinematic control gain  - controls amplitude
LF{2}.pPar.K2 = LF{1}.pPar.K2;     % kinematic control gain - control saturation

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;

%% Robots initial pose
% detect rigid body ID from optitrack
idP{1} = getID(OPT,P{1});            % pioneer ID on optitrack
idP{2} = getID(OPT,P{2})+1;          % pioneer ID on optitrack
idP{3} = getID(OPT,P{3})+2;          % pioneer ID on optitrack
rb = OPT.RigidBody;                   % read optitrack data
P{1} = getOptData(rb(idP{1}),P{1});   % get pioneer data
P{2} = getOptData(rb(idP{2}),P{2});   % get pioneer data
P{3} = getOptData(rb(idP{3}),P{3});   % get pioneer data

%% Variable initialization
data = [];

%% Formation initial error
% Formation initial pose
LF{1}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];
LF{1}.pPos.Xr = LF{1}.pPos.X;

LF{2}.pPos.X = [P{1}.pPos.X(1:3); P{3}.pPos.X(1:3)];
LF{2}.pPos.Xr = LF{2}.pPos.X;

% Formation initial pose
LF{1}.mDirTrans;
LF{2}.mDirTrans;

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization

T_CONTROL = 0.030;

rX = 1.0;       % [m]
rY = 1.0;       % [m]
rho = 1.00;     % [m]
T = 45;         % [s]
Tf = 90;        % [s]
w = 2*pi/T;     % [rad/s]

T_FP = 50;      % [s]

caso = 1;

t_control = tic;
t_Pioneer = tic;        % pioneer cycle

t_integ{1} = tic;
t_integ{2} = tic;

t  = tic;

while toc(t) < Tf
    if toc(t_control) > T_CONTROL               
        t_control = tic;
             
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>2
            P{1}.pSC.U  = Rede.pMSG.getFrom{4}(29:30);  % current velocities (robot sensors)
            P1X       = Rede.pMSG.getFrom{4}(14+(1:12));   % current position (robot sensors)
            
            P{2}.pSC.U  = Rede.pMSG.getFrom{5}(29:30);  % current velocities (robot sensors)
            P2X       = Rede.pMSG.getFrom{5}(14+(1:12));   % current position (robot sensors)
            
            P{3}.pSC.U  = Rede.pMSG.getFrom{6}(29:30);  % current velocities (robot sensors)
            P3X       = Rede.pMSG.getFrom{6}(14+(1:12));   % current position (robot sensors)
        end

        % Get optitrack data
        rb = OPT.RigidBody;                   % read optitrack data
        P{1} = getOptData(rb(idP{1}),P{1});   % get pioneer data
        P{2} = getOptData(rb(idP{2}),P{2});   % get pioneer data
        P{3} = getOptData(rb(idP{3}),P{3});   % get pioneer data
        
        LF{1}.pPos.X = [P{1}.pPos.X(1:3); P{2}.pPos.X(1:3)];  % Posição dos membros da formação
        LF{1}.mDirTrans;                                      % Transformada Direta X --> Q
        
        LF{2}.pPos.X = [P{1}.pPos.X(1:3); P{3}.pPos.X(1:3)];  % Posição dos membros da formação
        LF{2}.mDirTrans;                                      % Transformada Direta X --> Q
        
        %% Trajectory

        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf; 
        X_traj = rX*sin(w*tp);
        Y_traj = rY*sin(2*w*tp);
        dX_traj = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);
        dY_traj = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);
        
        LF{1}.pPos.Qd(1) = X_traj;                           % xF
        LF{1}.pPos.Qd(2) = Y_traj;                           % yF
        LF{1}.pPos.Qd(4) = rho;                           % rho
        LF{1}.pPos.Qd(5) = pi/4;                          % alpha (+frente/-trás)
        LF{1}.pPos.Qd(6) = pi/4;                          % beta  (lateral  -dir/+ esq)

        LF{1}.pPos.dQd(1) = dX_traj;                         % dxF
        LF{1}.pPos.dQd(2) = dY_traj;                         % dyF
        
        LF{2}.pPos.Qd(1:3) = LF{1}.pPos.Qd(1:3);          % xF , yF , zF
        LF{2}.pPos.Qd(4) = rho;                           % rho
        LF{2}.pPos.Qd(5) = pi/4;                          % alpha (+frente/-trás)
        LF{2}.pPos.Qd(6) = -pi/4;                         % beta  (lateral  -dir/+ esq)
        
        LF{2}.pPos.dQd = LF{1}.pPos.dQd;
        
        LF{1}.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        LF{2}.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        
        %% Control
        
        % Formation Control
        
%         LF{1}.mFormationControl(toc(t_integ{1}));
        LF{1}.mFormationControl_NullSpace('P',toc(t_integ{1})); % 'P' ---> Position    'F' ---> Form
%         LF{1}.mFormationControl_NullSpace('F',toc(t_integ{1})); % 'F' ---> Position    'F' ---> Form
        t_integ{1} = tic;
        
%         LF{2}.mFormationControl(toc(t_integ{2}));
        LF{2}.mFormationControl_NullSpace('P',toc(t_integ{2})); % 'P' ---> Position    'F' ---> Form
%         LF{2}.mFormationControl_NullSpace('F',toc(t_integ{2})); % 'F' ---> Position    'F' ---> Form
        t_integ{2} = tic;
           
        % Pioneer       
        
%         P{1}.pPos.Xd(1:2) = [ X_traj ; Y_traj];             % Posição desejada
%         P{1}.pPos.Xd(7:8) = [ dX_traj ; dY_traj];           % Velocidade desejada
        
        P{1}.pPos.Xd(1:2) = LF{1}.pPos.Xr(1:2);             % Posição desejada
        P{1}.pPos.Xd(7:8) = LF{1}.pPos.dXr(1:2);            % Velocidade desejada
        
        P{2}.pPos.Xd(1:2) = LF{1}.pPos.Xr(4:5);             % Posição desejada
        P{2}.pPos.Xd(7:8) = LF{1}.pPos.dXr(4:5);            % Velocidade desejada
        
        P{3}.pPos.Xd(1:2) = LF{2}.pPos.Xr(4:5);             % Posição desejada
        P{3}.pPos.Xd(7:8) = LF{2}.pPos.dXr(4:5);            % Velocidade desejada
        
        % Controlador Dinâmico       
        P{1} = fDynamicController(P{1},Pgains);
        P{2} = fDynamicController(P{2},Pgains);
        P{3} = fDynamicController(P{3},Pgains);
       
% %         %% Simulando Falhas
% %         
% %         % Pioneer
% %         if toc(t) > T_FP && toc(t) < T_FP + 30
% % 
% %             P1.pSC.Ud = [0; 0];
% %             
% %         end
  
        %% Save data (.txt file)        
        % Variable to feed plotResults function
        data = [  data  ; P{1}.pPos.Xd'     P{1}.pPos.X'        P{1}.pSC.Ud(1:2)'    P{1}.pSC.U(1:2)' ...
                          P{2}.pPos.Xd'     P{2}.pPos.X'        P{2}.pSC.Ud(1:2)'    P{2}.pSC.U(1:2)' ...
                          P{3}.pPos.Xd'     P{3}.pPos.X'        P{3}.pSC.Ud(1:2)'    P{3}.pSC.U(1:2)' ...
                          LF{1}.pPos.Qd'    LF{1}.pPos.Q'       LF{1}.pPos.Qtil'     LF{1}.pPos.Xd' ...
                          LF{2}.pPos.Qd'    LF{1}.pPos.Q'       LF{2}.pPos.Qtil'     LF{2}.pPos.Xd' ...
                          toc(t)];
           
            % %       %   1 -- 12         13 -- 24          25 -- 26           27 -- 28 
            % %           P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud'         P1.pSC.U'
            % %           
            % %       %   29 -- 40        41 -- 52          53 -- 54           55 -- 56    
            % %           P2.pPos.Xd'     P2.pPos.X'        P2.pSC.Ud'         P2.pSC.U'
            % %
            % %       %   57 -- 68        69 -- 80          81 -- 82           83 -- 84    
            % %           P3.pPos.Xd'     P3.pPos.X'        P3.pSC.Ud'         P3.pSC.U'
            % %             
            % %       %   85 -- 90        91 -- 96          97 -- 102          
            % %           LF1.pPos.Qd'    LF1.pPos.Qtil'    LF1.pPos.Xd'    
            % %
            % %       %   103 -- 108      109 -- 114        115 -- 120          
            % %           LF2.pPos.Qd'    LF2.pPos.Qtil'    LF2.pPos.Xd'  
            % %
            % %       %   121
            % %       %   toc(t)  ];
    
     
        % Send control signals to robots        
        Rede.mSendMsg(P{1});    
        Rede.mSendMsg(P{2});
        Rede.mSendMsg(P{3});
        
    end
end

%% Send control signals
% P{1}.pSC.Ud = [.05  ;  0];
% P{2}.pSC.Ud = [.05  ;  0];
% P{3}.pSC.Ud = [.05  ;  0];
% 
% for ii = 1:100
%     Rede.mSendMsg(P{1});
%     Rede.mSendMsg(P{2});
%     Rede.mSendMsg(P{3});
% end
% 
% P{1}.pSC.Ud = -[.1  ;  0];
% P{2}.pSC.Ud = -[.1  ;  0];
% P{3}.pSC.Ud = -[.1  ;  0];
% 
% for ii = 1:100
%     Rede.mSendMsg(P{1});
%     Rede.mSendMsg(P{2});
%     Rede.mSendMsg(P{3});
% end

%% Send control signals
P{1}.pSC.Ud = [0  ;  0];
P{2}.pSC.Ud = [0  ;  0];
P{3}.pSC.Ud = [0  ;  0];
    
for ii = 1:100
    Rede.mSendMsg(P{1});    
    Rede.mSendMsg(P{2});
    Rede.mSendMsg(P{3});   
end

%% Plot results
% % figure;
% plotResultsLineControl_v2(data);
% plotResultsLineControl(data2);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
