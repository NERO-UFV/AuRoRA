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
    setenv('ROS_IP','192.168.0.153')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    B = Bebop(1,'B1');
    P = RPioneer(1);
    P = Pioneer3DX(1);
    
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
B.rTakeOff;
pause(3);
disp('Taking Off End Time....');

%% Variable initialization
X_P_hist = [];
X_B_hist = [];
Qd_hist = [];
Q_hist = [];
t_hist = [];
B.rGetSensorDataLocal;
P.rGetSensorData;

% Time variables initialization
t_amostragem = 0.2; % 200 ms de Amostragem | 5 Hz de Frequência
t_max = 20;

T = 30;
w = 2*pi/T;

fprintf('\nStart..............\n\n');

% Formação
L1 = diag([.5 .5 1 .5 1 1]);
L2 = diag([1 1 1 1 1 1]);

% Pioneer
a_P = 0.1;
Vref_P_A = 0;

% Drone
Ku = diag([0.8417 0.8354 3.966 9.8524]);
Kv = diag([0.18227 0.17095 4.001 4.7295]);
K_bebop = diag([.8 .8 .8 .8]);
Vref_B_A = 0;
K1_ori = 2;
K2_ori = 2;

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
t_der = tic;

tol_caminho = 0.1;

%Caminho
Ve = 0.05;
Vd = 0;
Vdmax = 0.3;

inc = 0.1;
term = 150;
s = 0:inc:term;
x = -0.5*cos(pi*s/100) + 1;
y = 0.5*sin(pi*s/100);
% x = 1*ones(1,length(s));
% y = 0.01*s;
% z = 1.5*ones(1,length(s));
z = 0*ones(1,length(s));

C = [x; y; z];
joy = vrjoystick(1);
dist_final = 1000;
dist_term = .1;
tol_final = 0.1;
Vd1A = 0;

try
    while dist_final > dist_term
        if toc(t_control) > t_amostragem
            t_control = tic;
            
            B.rGetSensorDataLocal;
            P.rGetSensorData;
            
            X_P = P.pPos.X;
            X_B = B.pPos.X;
            
            a_cont = axis(joy);
            %             Vd = -Ve*a_cont(5);
            Vd = Ve;
            dist_final = norm(C(1:2,end)-X_P(1:2));
            
            [dist, ind] = calcula_ponto_proximo(C(1:2,:),X_P(1:2));
            tan_inc(1,1) = 0.5*pi/100*sin(pi*s(ind)/100);
            tan_inc(2,1) = 0.5*pi/100*cos(pi*s(ind)/100);
%                         tan_inc(1,1) = 0;
%                         tan_inc(2,1) = 0.01;
            
            vtan = Vd*tan_inc/norm(tan_inc);
            
            Qd = [C(1,ind) C(2,ind) 0 0.8 0 pi/2.5]';
            
            if dist > tol_caminho
                dQd = [0 0 0 0 0 0]';
            else
                dQd = [vtan' 0 0 0 0]';
            end
            
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
            
            % Controlador Orientação Bebop
            Otil = X_P(6) - X_B(6);
            dBref_O = K1_ori*tanh(K2_ori*Otil);
            
            Vref_P = F_P\dXref(1:2);
            Vref_B = F_B\[dXref(4:6);dBref_O];
            
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
            
            P.rCommand;
            B.rCommand;
            
            % Histórico
            X_P_hist = [X_P_hist X_P];
            X_B_hist = [X_B_hist X_B];
            Qd_hist = [Qd_hist Qd];
            Q_hist = [Q_hist Q];
            t_hist = [t_hist toc(t)];
            
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

%%
figure
plot(X_P_hist(1,:),X_P_hist(2,:))
hold on
plot(X_B_hist(1,:),X_B_hist(2,:))


Qtil_hist = Qd_hist - Q_hist;
figure
plot(t_hist,Qtil_hist(1,:))

figure
plot(t_hist,Qtil_hist(2,:))

figure
plot(t_hist,Qtil_hist(3,:))

figure
plot(t_hist,Qtil_hist(4,:))

figure
plot(t_hist,Qtil_hist(5,:))

figure
plot(t_hist,Qtil_hist(6,:))
