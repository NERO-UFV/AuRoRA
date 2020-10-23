%% 3D Line Formation Pioneer-Drone

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

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.101');
    B = Bebop(1,'B1');
    P = RPioneer(1);
    
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

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);
% Bebop
disp('Start Take Off Timming....');
% B.rTakeOff;
pause(3);
disp('Taking Off End Time....');

%% Variable initialization
data = [];
B.rGetSensorDataLocal;
P.rGetSensorData;

% Time variables initialization
t_amostragem = 0.2; % 200 ms de Amostragem | 5 Hz de Frequência
t_max = 20;

T = 30;
w = 2*pi/T;

fprintf('\nStart..............\n\n');

% Formação
L1 = diag([.5 .5 1 .5 .5 .5]);
L2 = diag([1 1 1 1 1 1]);

% Pioneer
a_P = 0.1;
Vref_P_A = 0;

% Drone
Ku = diag([0.8417 0.8354 3.966 9.8524]);
Kv = diag([0.18227 0.17095 4.001 4.7295]);
K_bebop = diag([.8 .8 .8 .8]);
Vref_B_A = 0;

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
t_der = tic;

try
    while toc(t) < t_max
        if toc(t_control) > t_amostragem
            t_control = tic;
            
            B.rGetSensorDataLocal;
            P.rGetSensorData;
            
            X_P = P.pPos.X;
            X_B = B.pPos.X;
            
            Qd = [2 0 0 0.8 0 pi/3]';
            dQd = [0 0 0 0 0 0]';
            
            Rho = norm(X_B(1:3) - X_P(1:3));
            Alpha = atan2(X_B(2) - X_P(2),X_B(1) - X_P(1));
            Beta = atan2(X_B(3) - X_P(3),norm(X_B(1:2) - X_P(1:2)));
            Q = [X_P(1) X_P(2) X_P(3) Rho Alpha Beta]';
            
            Qtil = Qd - Q;
            
            dQref = dQd + L1*tanh(L2*Qtil);
            
            Jinv = [1 0 0       0                   0                           0;
                0 1 0           0                   0                           0;
                0 0 1           0                   0                           0;
                1 0 0 cos(Alpha)*cos(Beta) -Rho*sin(Alpha)*cos(Beta) -Rho*cos(Alpha)*sin(Beta);
                0 1 0 sin(Alpha)*cos(Beta)  Rho*cos(Alpha)*cos(Beta) -Rho*sin(Alpha)*sin(Beta);
                0 0 1       sin(Beta)               0                       Rho*cos(Beta)    ];
            
            dXref = Jinv*dQref;
            
            F_P = [cos(X_P(6)) -a_P*sin(X_P(6));
                sin(X_P(6)) a_P*cos(X_P(6))];
            
            F_B = [cos(X_B(6)) -sin(X_B(6)) 0 0;
                sin(X_B(6))  cos(X_B(6)) 0 0;
                0            0      1 0;
                0            0      0 1];
            
            Vref_P = F_P\dXref(1:2);
            Vref_B = F_B\[dXref(4:6);0];
            
            %Derivando a velocidade desejada
            dVref_P = (Vref_P - Vref_P_A)/toc(t_der);
            dVref_B = (Vref_B - Vref_B_A)/toc(t_der);
            t_der = tic;
            
            Vref_P_A = Vref_P;
            Vref_B_A = Vref_B;
            
            %Velocidade do robô em seu próprio eixo
            V_P = [X_P(7) X_P(12)];
            V_B = F_B\([X_B(7:9);X_B(12)]);

            Ud_P = Vref_P;
            Ud_B = Ku\(dVref_B + K_bebop*(Vref_B - V_B) + Kv*V_B);
            
            P.pSC.Ud = Ud_P(1:2);
            B.pSC.Ud = [Ud_B(1:3)' 0 0 Ud_B(4)]';
            
            B = J.mControl(B);
            
            B.pSC.Ud
            
            P.rCommand;
%             B.rCommand;
            
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

