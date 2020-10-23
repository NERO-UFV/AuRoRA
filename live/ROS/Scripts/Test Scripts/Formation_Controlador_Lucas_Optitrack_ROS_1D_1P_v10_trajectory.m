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

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.166');
    B = Bebop(1,'B');
    P = PioneerROS;
    
    %P = Pioneer3DX(1);  % Pioneer Instance
    
    % Joystick
    J = JoyControl;

    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
       
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end



%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);

% Formation 3D
LF = LineFormationControl;

% Beboop
%disp('Start Take Off Timming....');
% B.rTakeOff;
% pause(3);
disp('Taking Off End Time....');

%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];
LF.pPos.Xr = LF.pPos.X;
LF.mDirTrans;

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 0.2;

rX = 1.0;      % [m]
rY = 1.0;      % [m]
rho = 1.5;      % [m]
T = 40;         % [s]
Tf = 80;        % [s]
w = 2*pi/T;     % [rad/s]

T1 = 75;             % Lemniscata
T2 =  3.0 + T1;         % Aproximação + Emergency
T3 =  0.1 + T2;         % Andando com o Drone pousado
T4 =  2.0 + T3;         % Parando o Pioneer

caso = 1;

fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

try    
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
            
        elseif toc(t) > T2 && toc(t) < T3  
            if caso == 3
                fprintf('\n\nCaso 3: Andando com o Drone pousado\n');
                caso = 4;           
                disp("Land Command");
                B.rCmdStop; 
                B.rLand;
                B.rCmdStop; 
            end
            LF.pPos.Qd(1) = X_traj;       % xF
            LF.pPos.Qd(2) = Y_traj;       % yF
            LF.pPos.Qd(4) = rho/2;        % rho
            LF.pPos.dQd(1) = dX_traj;     % dxF
            LF.pPos.dQd(2) = dY_traj;     % dyF
        end
             
        % Ardrone
        B.rGetSensorDataOpt;
        
        LF.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];   % Posição dos membros da formação
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
%         LF.pPar.K1 = diag([   10.0  10.0    10.0   5.0   5.0    5.0   ]);     % kinematic control gain  - controls amplitude
%         LF.pPar.K2 = diag([    0.1   0.1     0.1   0.1   0.1    0.1   ]);     % kinematic control gain - control saturation
%    
        % Ganhos P
        LF.pPar.K1 = diag([   1.0   1.0     1.0    5.0   5.0    5.0   ]);     % kinematic control gain  - controls amplitude
        LF.pPar.K2 = diag([   0.4   0.4     0.4    0.1   0.1    0.1   ]);     % kinematic control gain - control saturation
 
        LF.mInvTrans;                                 % Transformada Inversa Qd --> Xd
        
        P.pPos.Xda = P.pPos.Xd;    % save previous posture
        P.pPos.Xd(1:2) = LF.pPos.Xr(1:2);             % Posição desejada
        P.pPos.Xd(7:8) = LF.pPos.dXr(1:2);            % Velocidade desejada
        
        B.pPos.dXd(7:12) = (B.pPos.Xd(7:12) - B.pPos.Xda(7:12))/toc(t_incB);
        B.pPos.X(12) = (B.pPos.X(6) - B.pPos.Xa(6))/toc(t_incB);
        t_incB = tic;
        
        B.pPos.Xda = B.pPos.Xd;    % save previous posture
        B.pPos.Xd(1:3) = LF.pPos.Xr(4:6);
        B.pPos.Xd(7:9) = LF.pPos.dXr(4:6);

        B.pPos.Xd(6) = P.pPos.X(6);
        B.pPos.Xd(12) = P.pPos.X(12);  % dPsi
            
            %% Save data

        % Variable to feed plotResults function
        data = [  data  ; P.pPos.Xd'     P.pPos.X'        P.pSC.Ud(1:2)'    P.pSC.U(1:2)'...
                          B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
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

        % Test send commands
        P.rCommand;   
               
        % Beboop

%         A = cUnderActuatedController(A,Agains);  % ArDrone
        % Joystick Command Priority
        B = J.mControl(B);                    % joystick command (priority)   
        B.rCommand;
        
        
        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
            disp('Bebop Landing through Emergency Command ');
            B.rCmdStop;
            B.rLand;
            
            P.rCmdStop;
            break;
        end  
        
        
    end
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B.rCmdStop;       
    
    disp('');
    disp(ME);
    disp('');
    
    % Send Land Command
    P.rCmdStop;     
end


% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B.rCmdStop;    
    B.rLand
    
    P.rCmdStop;         
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%% Plot results
figure;
plotResultsLineControl(data);
