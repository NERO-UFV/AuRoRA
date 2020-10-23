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
    RI.rConnect('192.168.0.146');
    B{1} = Bebop(1,'B1');
    
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
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);
% Beboop
disp('Start Take Off Timming....');
% B.rTakeOff;
% pause(3);
disp('Taking Off End Time....');

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 0.2; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 60;

T = 30;
w = 2*pi/T;

fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

try    
    while toc(t) < T_MAX

    if toc(t_control) > T_CONTROL             
        
        t_control = tic;
        
        % Trajectory Planner        
        Xd = [cos(w*toc(t)) sin(w*toc(t)) 1.5 0]';
        dXd = [-w*sin(w*toc(t)) w*cos(w*toc(t)) 0 0]';
        ddXd = [-w^2*cos(w*toc(t)) -w^2*sin(w*toc(t)) 0 0]';
        
        % Ardrone
        B.rGetSensorDataOpt;
       
        % Encontrando velocidade angular
        B.pPos.X(12) = (B.pPos.X(6) - B.pPos.Xa(6))/toc(t_incB);
        t_incB = tic;
        
        B.pPos.Xd(1:3) = Xd(1:3);
        B.pPos.Xd(6) = Xd(4);
        
        B.pPos.Xd(7:9) = dXd(1:3);
        B.pPos.Xd(12) = dXd(4);
        
        B.pPos.dXd(7:9) = ddXd(1:3);
        B.pPos.dXd(12) = ddXd(4);

        B.cInverseDynamicController;
        
            %% Save data

        % Variable to feed plotResults function
        data = [  data  ; B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
                          toc(t)];
           
% %         %   1 -- 12      13 -- 24     25 -- 26          27 -- 28 
% %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
% %             
% %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60    
% %             B.pPos.Xd'   B.pPos.X'    B.pSC.Ud'         B.pSC.U'
% %             
% %         %   61 -- 66     67 -- 72       73 -- 78       79
% %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];

               
        % Beboop
        % Joystick Command Priority
        B = J.mControl(B);                    % joystick command (priority)   
        B.rCommand;
        B.pSC.Ud

        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
            disp('Bebop Landing through Emergency Command ');
            B.rCmdStop;
            B.rLand;
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
   
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B.rCmdStop;    
    B.rLand      
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%% Plot results
% figure;
% plotResultsLineControl(data);
