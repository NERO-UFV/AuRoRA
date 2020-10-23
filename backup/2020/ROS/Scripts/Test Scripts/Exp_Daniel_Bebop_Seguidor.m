clear; close all; clc;
try
    fclose(instrfindall);
catch
end


%% Load Class
try
    % Load Classes
    RI = RosInterface;
    
    % INSERIR IP DO SERVIDOR ROS EM USO
    setenv('ROS_IP','192.168.0.102')
    setenv('ROS_MASTER_URI','http://192.168.0.102:11311')
    RI.rConnect('192.168.0.102');
    B{1} = Bebop(1,'B1');    
    
    % Joystick
    J = JoyControl;   

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



%%%%%%%%%%%%%%%%%%%%%% Bot�o de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

% Bebop
disp('Start Take Off Timming....');
% B{1}.rTakeOff;
disp('Taking Off End Time....');

%% Variable initialization
data = [];
B{1}.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295  ]';
B{1}.pPar.Ts = 1/30;

% Time variables initialization
t  = tic;
T_exp = 480; % tempo de experimento
T_amos = 1/30; % per�odo de amostragem do experimento
t_amos = tic;
t_exp = tic;

cont = 0;

pause(3);
B{1}.pPos.X(1:3) = [0 0 0];
disp('While....');
try
    while toc(t) < T_exp
              
        if toc(t_amos) > T_amos

            t_amos = tic;
            t_atual = toc(t);

%% TRAJET�RIA
                B{1}.pPos.Xd([1 2 3 6]) = [-1;0;-.2;0];
                B{1}.pPos.Xd([7 8 9 12]) = [0;0;0;0];
%                           
%% ODOMETRIA POR WHYCON
              B{1}.pPos.Xa(1:6) = B{1}.pPos.X(1:6);
              B{1}.rGetMarker;
              B{1}.pPos.X(7:9) = (B{1}.pPos.X(1:3) - B{1}.pPos.Xa(1:3))/toc(B{1}.pSC.tcontrole);
                       
%% CONTROLE            

            B{1}.cInverseDynamicController_Compensador;
            
% Filtro evitando envio de sinal de controle caso o Bebop tenha perdido o marcador WHYCON
            
            if sum(B{1}.pPos.Xa(1:6) == B{1}.pPos.X(1:6)) == 6
                cont = cont + 1;
                B{1}.cInverseDynamicController_Compensador;                
                B{1}.pSC.Ud(6) = 0;  % remoção controle yaw (opcional)
            else
                cont = 0;
                B{1}.cInverseDynamicController_Compensador;                
                B{1}.pSC.Ud(6) = 0;  % remoção controle yaw (opcional)
            end
            
            if cont >= 10
                B{1}.pSC.Ud([1:3 6]) = [0 0 0 0];
            end
%             
            disp(B{1}.pSC.Ud([1:3 6]));

            % Joystick Command Priority
            B{1} = J.mControl(B{1}); 
            B{1}.rCommand;
            
%% DATA      

        data = [];
        
%% EMERG�NCIA

        drawnow
        if btnEmergencia ~= 0 || B{1}.pFlag.EmergencyStop ~= 0 || B{1}.pFlag.isTracked ~= 1
            disp('Bebop Landing through Emergency Command ');

            % Send 3 times Commands 1 second delay to Drone Land
            for i=1:nLandMsg
                disp("End Land Command");
                B{1}.rCmdStop;
                B{1}.rLand;
            end
            break;
        end
                end
            end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B{1}.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    B{1}.rLand
    
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B{1}.rCmdStop;
    B{1}.rLand
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");
