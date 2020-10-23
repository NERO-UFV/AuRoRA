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

%% Load Classes

% Robots

P1 = Pioneer3DX(1);
P2 = Pioneer3DX(2);
P3 = Pioneer3DX(3);

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Formation 3D
LF1 = LineFormationControl;
LF2 = LineFormationControl;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;

%% Network communication check
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P1);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 4
        if isempty(Rede.pMSG.getFrom{4})
            Rede.mSendMsg(P1);
            
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message 1......')
            
        else
            break
        end
    end
end

tm = tic;
while true
    
    if length(Rede.pMSG.getFrom) > 4
        if isempty(Rede.pMSG.getFrom{5})
            for ii = 1:100
                Rede.mSendMsg(P2);
            end
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message 2......')
            
        else
            for ii = 1:100
                Rede.mSendMsg(P2);
            end
            break
        end
    end
end

tm = tic;
while true
    
    if length(Rede.pMSG.getFrom) > 4
        if isempty(Rede.pMSG.getFrom{6})
            for ii = 1:10
                Rede.mSendMsg(P3);
            end
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message 3......')
            
        else
            for ii = 1:10
                Rede.mSendMsg(P3);
            end
            break
        end
    end
end
clc
disp('Data received. Continuing program...');

%% Robots initial pose
% detect rigid body ID from optitrack
idP1 = getID(OPT,P1);            % pioneer ID on optitrack
idP2 = getID(OPT,P2)+1;          % pioneer ID on optitrack
idP3 = getID(OPT,P3)+2;          % pioneer ID on optitrack

rb = OPT.RigidBody;            % read optitrack data
P1 = getOptData(rb(idP1),P1);   % get pioneer data
P2 = getOptData(rb(idP2),P2);   % get pioneer data
P3 = getOptData(rb(idP3),P3);   % get pioneer data

%% Variable initialization

data = [];

%% Formation initial error

% Formation initial pose
LF1.pPos.X = [P1.pPos.X(1:3); P2.pPos.X(1:3)];
LF1.pPos.Xr = LF1.pPos.X;

LF2.pPos.X = [P1.pPos.X(1:3); P3.pPos.X(1:3)];
LF2.pPos.Xr = LF2.pPos.X;

% Formation initial pose
LF1.mDirTrans;
LF2.mDirTrans;

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization

T_CONTROL = 0.030;
T_PIONEER = 0.030;

% rX = 1.0;       % [m]
% rY = 1.0;       % [m]
% rho = 1.0;      % [m]
% T = 45;         % [s]
% Tf = 90;       % [s]
% w = 2*pi/T;     % [rad/s]

rX = 1.0;       % [m]
rY = 1.0;       % [m]
rho = 1.0;      % [m]
T = 50;         % [s]
Tf = 100;       % [s]
w = 2*pi/T;     % [rad/s]

T_FP = 50;      % [s]

caso = 1;

t_control = tic;
t_Pioneer = tic;        % pioneer cycle

t_integ_1 = tic;
t_integ_2 = tic;

t  = tic;

while toc(t) < Tf

    if toc(t_control) > T_CONTROL
               
        t_control = tic;
             
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>2
            P1.pSC.U  = Rede.pMSG.getFrom{4}(29:30);  % current velocities (robot sensors)
            P1X       = Rede.pMSG.getFrom{4}(14+(1:12));   % current position (robot sensors)
            
            P2.pSC.U  = Rede.pMSG.getFrom{5}(29:30);  % current velocities (robot sensors)
            P2X       = Rede.pMSG.getFrom{5}(14+(1:12));   % current position (robot sensors)
            
            P3.pSC.U  = Rede.pMSG.getFrom{6}(29:30);  % current velocities (robot sensors)
            P3X       = Rede.pMSG.getFrom{6}(14+(1:12));   % current position (robot sensors)
        end

        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        
        P1 = getOptData(rb(idP1),P1);
        P2 = getOptData(rb(idP2),P2);
        P3 = getOptData(rb(idP3),P3); 
        
        LF1.pPos.X = [P1.pPos.X(1:3); P2.pPos.X(1:3)];  % Posição dos membros da formação
        LF1.mDirTrans;                                  % Transformada Direta X --> Q
        
        LF2.pPos.X = [P1.pPos.X(1:3); P3.pPos.X(1:3)];  % Posição dos membros da formação
        LF2.mDirTrans;                                  % Transformada Direta X --> Q
        
        %% Trajectory

        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf;

        LF1.pPos.Qd(1) = rX*sin(w*tp);                   % xF
        LF1.pPos.Qd(2) = rY*sin(2*w*tp);                 % yF
        LF1.pPos.Qd(3) = 0.00;                           % zF
        LF1.pPos.Qd(4) = rho;                            % rho
        LF1.pPos.Qd(5) = pi/4;                           % alpha (frente/trás)
        LF1.pPos.Qd(6) = pi/4;                           % beta  (lateral)

        LF1.pPos.dQd(1) = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);     % dxF
        LF1.pPos.dQd(2) = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);      % dyF
        LF1.pPos.dQd(3) = 0.00;                          % dzF
        LF1.pPos.dQd(4) = 0.00;                          % drho
        LF1.pPos.dQd(5) = 0.00;                          % dalpha
        LF1.pPos.dQd(6) = 0.00;                          % dbeta           
     
        LF2.pPos.Qd(1:3) = LF1.pPos.Qd(1:3);            % xF , yF , zF
        LF2.pPos.Qd(4) = rho;                           % rho
        LF2.pPos.Qd(5) = pi/4;                          % alpha (+frente/-trás)
        LF2.pPos.Qd(6) = -pi/4;                         % beta  (lateral  -dir/+ esq)
        
        LF2.pPos.dQd = LF1.pPos.dQd;
        
        LF1.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        LF2.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        
        %% Control
        
        % Formation Control
        
%         LF1.mFormationControl(toc(t_integ_1));
        LF1.mFormationControl_NullSpace('P',toc(t_integ_1)); % 'P' ---> Position    'F' ---> Form
%         LF1.mFormationControl_NullSpace('F',toc(t_integ_1)); % 'F' ---> Position    'F' ---> Form

        t_integ_1 = tic;
        
%         LF2.mFormationControl(toc(t_integ_2));
        LF2.mFormationControl_NullSpace('P',toc(t_integ_2)); % 'P' ---> Position    'F' ---> Form
%         LF2.mFormationControl_NullSpace('F',toc(t_integ_2)); % 'F' ---> Position    'F' ---> Form

        t_integ_2 = tic;
        
        % Ganhos
              
%         LF1.pPar.K1 = diag([   15.0   15.0   15.0   15.0  10.0  10.0   ]);     % kinematic control gain  - controls amplitude
%         LF1.pPar.K2 = diag([    0.1    0.1    0.1    0.1   0.1   0.1   ]);     % kinematic control gain - control saturation
        
        LF1.pPar.K1 = diag([    30.0    30.0   30.0   30.0  30.0  30.0   ]);     % kinematic control gain  - controls amplitude
        LF1.pPar.K2 = diag([     0.1    0.1    0.1    0.1   0.1   0.1   ]);
        
        LF2.pPar.K1 = LF1.pPar.K1;     % kinematic control gain  - controls amplitude
        LF2.pPar.K2 = LF1.pPar.K2;     % kinematic control gain - control saturation
      
        % Pioneer       
        
        P1.pPos.Xd(1:2) = LF1.pPos.Xr(1:2);             % Posição desejada
        P1.pPos.Xd(7:8) = LF1.pPos.dXr(1:2);            % Velocidade desejada
        
        P2.pPos.Xd(1:2) = LF1.pPos.Xr(4:5);             % Posição desejada
        P2.pPos.Xd(7:8) = LF1.pPos.dXr(4:5);            % Velocidade desejada
        
        P3.pPos.Xd(1:2) = LF2.pPos.Xr(4:5);             % Posição desejada
        P3.pPos.Xd(7:8) = LF2.pPos.dXr(4:5);            % Velocidade desejada
        
        % Controlador Dinâmico
        
        % Ganhos pré-definidos
%         cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];
        cgains = [ 0.10  0.10  0.75  0.75  0.75  0.75  0.10  0.05 ];
        
        P1 = fDynamicController(P1,cgains);     % Pioneer Dynamic Compensator
        P2 = fDynamicController(P2,cgains);     % Pioneer Dynamic Compensator
        P3 = fDynamicController(P3,cgains);     % Pioneer Dynamic Compensator
       
        %% Simulando Falhas
        
        % Pioneer
        if toc(t) > T_FP && toc(t) < T_FP + 30

            P1.pSC.Ud = [0; 0];
            
        end
  
        %% Save data (.txt file)
%         disp([P1.pSC.U   P2.pSC.U   P3.pSC.U]);
        
        % Variable to feed plotResults function
        data = [  data  ; P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud(1:2)'    P1.pSC.U(1:2)' ...
                          P2.pPos.Xd'     P2.pPos.X'        P2.pSC.Ud(1:2)'    P2.pSC.U(1:2)' ...
                          P3.pPos.Xd'     P3.pPos.X'        P3.pSC.Ud(1:2)'    P3.pSC.U(1:2)' ...
                          LF1.pPos.Qd'    LF1.pPos.Qtil'    LF1.pPos.Xd' ...
                          LF2.pPos.Qd'    LF2.pPos.Qtil'    LF2.pPos.Xd' ...
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
    
     
        %% Send control signals to robots
        
        Rede.mSendMsg(P1);
        Rede.mSendMsg(P2);
        Rede.mSendMsg(P3);
    
        if toc(t_control) > 0.030
            disp(['Estourou o tempo: ',num2str(1000*(toc(t_control)-0.030))]); 
        end
    end

end

%% Send control signals
P1.pSC.Ud = -[.05  ;  0];
P2.pSC.Ud = -[.05  ;  0];
P3.pSC.Ud = -[.05  ;  0];

for ii = 1:100
    Rede.mSendMsg(P1);
    Rede.mSendMsg(P2);
    Rede.mSendMsg(P3);
end

%% Send control signals
P1.pSC.Ud = [0  ;  0];
P2.pSC.Ud = [0  ;  0];
P3.pSC.Ud = [0  ;  0];
    
for ii = 1:100
    Rede.mSendMsg(P1);    
    Rede.mSendMsg(P2);
    Rede.mSendMsg(P3);   
end

%% Plot results
% % figure;
% plotResultsLineControl_v2(data);
% plotResultsLineControl(data2);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
