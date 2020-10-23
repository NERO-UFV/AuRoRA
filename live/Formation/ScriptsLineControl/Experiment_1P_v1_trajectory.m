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
    elseif length(Rede.pMSG.getFrom) > 1
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

%% Robots initial pose
% detect rigid body ID from optitrack
idP1 = getID(OPT,P1);            % pioneer ID on optitrack
rb = OPT.RigidBody;            % read optitrack data
P1 = getOptData(rb(idP1),P1);   % get pioneer data

%% Variable initialization

data = [];

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization

T_CONTROL = 0.030;
T_PIONEER = 0.030;

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

t  = tic;

while toc(t) < Tf

    if toc(t_control) > T_CONTROL
               
        t_control = tic;
             
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>2
            P1.pSC.U  = Rede.pMSG.getFrom{4}(29:30);  % current velocities (robot sensors)
            P1X       = Rede.pMSG.getFrom{4}(14+(1:12));   % current position (robot sensors)
        end

        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        
        P1 = getOptData(rb(idP1),P1);
        
        %% Trajectory

        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf;

        P1.pPos.Xd(1) = rX*sin(w*tp);                   % xF
        P1.pPos.Xd(2) = rY*sin(2*w*tp);                 % yF

        P1.pPos.Xd(7) = w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp);     % dxF
        P1.pPos.Xd(8) = 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp);      % dyF

        %% Control
        % Controlador Dinâmico
        
        % Ganhos pré-definidos
%         cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];
        cgains = [ 0.10  0.10  0.75  0.75  0.75  0.75  0.10  0.05 ];
        
        P1 = fDynamicController(P1,cgains);     % Pioneer Dynamic Compensator
       
 
        %% Save data (.txt file)
%         disp([P1.pSC.U   P2.pSC.U   P3.pSC.U]);
        
        % Variable to feed plotResults function
        data = [  data  ; P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud(1:2)'    P1.pSC.U(1:2)' ...
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
            % %           LF1.pPos.Qd'    LF1.pPos.Qtil'    LF1.pPos.Xd'  
            % %
            % %       %   121
            % %       %   toc(t)  ];
    
     
        %% Send control signals to robots
        
        Rede.mSendMsg(P1);
    
        if toc(t_control) > 0.030
            disp(['Estourou o tempo: ',num2str(1000*(toc(t_control)-0.030))]); 
        end
    end

end

%% Send control signals
P1.pSC.Ud = -[.05  ;  0];

for ii = 1:100
    Rede.mSendMsg(P1);
end

%% Send control signals
P1.pSC.Ud = [0  ;  0];
    
for ii = 1:100
    Rede.mSendMsg(P1);    
end

%% Plot results
% % figure;
% plotResultsLineControl(data);

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
