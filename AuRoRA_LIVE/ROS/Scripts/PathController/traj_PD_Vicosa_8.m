
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
    setenv('ROS_MASTER_URI','http://192.168.0.148:11311')
    RI.rConnect('192.168.0.148');
    B = Bebop(1,'B');
    P = RPioneer(1);
    %     P = Pioneer3DX(1);
    %     idP = getID(OPT,P);
    
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
% B.rGetSensorDataLocal;
B.rGetSensorDataOpt;
P.rGetSensorDataOpt;

% Time variables initialization
t_amostragem = 0.1; % 200 ms de Amostragem | 5 Hz de Frequência
t_max = 120;

T = 45;
w = 2*pi/T;

fprintf('\nStart..............\n\n');

% Formação
L1 = diag([1.5 1.5 1 5 2 2]);
L2 = diag(.5*[1 1 1 1 1 1]);

% Pioneer
a_P = 0.15;
Vref_P_A = 0;

% Drone
Ku = diag([0.8417 0.8354 3.966 9.8524]);
Kv = diag([0.18227 0.17095 4.001 4.7295]);
K_bebop = diag([.8 .8 .8 .8]);
Vref_B_A = 0;
K1_ori = 1;
K2_ori = 1;

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
t_der = tic;

tol_caminho_in = 0.2;
tol_caminho_out = 0.60;
flag_robo_in = 0;

%Caminho
Ve = -0.6;
Vd = 0;
Vdmax = 0.3;

t_sw1 = 20;
t_sw2 = t_sw1 + 20;
t_sw3 = t_sw2 + 6;

raio_a = 0.8;
raio_b = 1.2;
% inc = .001;
% term = 200;
% s = 0:inc:term;

T = 50;
w = 2*pi/T;


% x = raio_a*sin(2*w*toc(t));
% y = raio_b*sin(w*toc(t));

% z = 0*ones(1,length(s));
% 
% C = [x; y; z];
joy = vrjoystick(1);
dist_final = 1000;
dist_term = .1;
tol_final = 0.1;
Vd1A = 0;
U_test_hist = [];
X_ref = [P.pPos.X(1);P.pPos.X(2)];
X_ref_hist = [];
try
    %     while dist_final > dist_term
    while t_max > toc(t)
        if toc(t_control) > t_amostragem
            t_control = tic;
            
            %             B.rGetSensorDataLocal;
            %             P.rGetSensorData;
            B.rGetSensorDataOpt;
            P.rGetSensorDataOpt;
            
            X_P = P.pPos.X;
            X_B = B.pPos.X;
            
            F_P = [cos(X_P(6)) -a_P*sin(X_P(6));
                sin(X_P(6)) a_P*cos(X_P(6))];
            
            F_B = [cos(X_B(6)) -sin(X_B(6)) 0 0;
                sin(X_B(6))  cos(X_B(6)) 0 0;
                0            0      1 0;
                0            0      0 1];
            
%             U_teste = F_P\[X_P(7);X_P(8)];
%             U_test_hist = [U_test_hist U_teste];
%             
%             Vd = Ve;
%             Vd = Vd/(abs(P.pSC.Ud(2)) + 1);
%             dist_final = norm(C(1:2,end)-X_P(1:2));
            
            x = raio_a*sin(2*w*toc(t));
            y = raio_b*sin(w*toc(t));
            
            dx = 2*w*raio_a*cos(2*w*toc(t));
            dy = w*raio_b*cos(w*toc(t));
            
            Qd = [x y 0 1 (X_P(6)+pi) pi/2.2]';
            dQd = [dx dy 0 0 (X_P(12)) 0]';
            
            Rho = norm(X_B(1:3) - X_P(1:3));
            Alpha = atan2(X_B(2) - X_P(2),X_B(1) - X_P(1));
            Beta = atan2(X_B(3) - X_P(3),norm(X_B(1:2) - X_P(1:2)));
            
            Q = [X_P(1) X_P(2) X_P(3) Rho Alpha Beta]';
            
            % Erro da formação
            Qtil = Qd - Q;

            % Tratamento de quadrante
            if abs(Qtil(5)) > pi
                if Qtil(5) > 0
                    Qtil(5) = -2*pi + Qtil(5);
                else
                    Qtil(5) =  2*pi + Qtil(5);
                end
            end
            
            dQref = dQd + L1*tanh(L2*Qtil);
            
            Jinv = [1 0 0       0                   0                           0;
                0 1 0           0                   0                           0;
                0 0 1           0                   0                           0;
                1 0 0 cos(Alpha)*cos(Beta) -Rho*sin(Alpha)*cos(Beta) -Rho*cos(Alpha)*sin(Beta);
                0 1 0 sin(Alpha)*cos(Beta)  Rho*cos(Alpha)*cos(Beta) -Rho*sin(Alpha)*sin(Beta);
                0 0 1       sin(Beta)               0                       Rho*cos(Beta)    ];
            
            dXref = Jinv*dQref;
%             X_ref = X_ref + dXref(1:2)*t_amostragem;
%             X_ref_hist = [X_ref_hist X_ref];
       
            % Controlador Orientação Bebop
%             Otil = X_P(6) - X_B(6);
%                         Otil = 0;
%             dBref_O = X_P(12) + K1_ori*tanh(K2_ori*Otil);
            dBref_O = 0;
            
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
            
%                         B.pSC.Ud = [0 0 0 0 0 0]';
            
            B = J.mControl(B);
            
            P.rCommand;
            B.rCommand;
            
            % Histórico
            X_P_hist = [X_P_hist X_P];
            X_B_hist = [X_B_hist X_B];
            Qd_hist = [Qd_hist Qd];
            Q_hist = [Q_hist Q];
            t_hist = [t_hist toc(t)];
            
            B.pFlag.EmergencyStop = 0;
            
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
filename = ['bebop_caminho_vicosa_' datestr(now,30) '.mat'];
% filename = 'teste_bebop_pouso2.mat';
% save(filename,'X_P_hist','X_B_hist','Qd_hist','Q_hist','t_hist')

%%
figure
plot3(X_P_hist(1,:),X_P_hist(2,:),X_P_hist(3,:))
hold on
plot3(Qd_hist(1,:),Qd_hist(2,:),X_P_hist(3,:),'--')
plot3(X_B_hist(1,:),X_B_hist(2,:),X_B_hist(3,:))
grid on
% plot(X_ref_hist(1,:),X_ref_hist(2,:))


Qtil_hist = Qd_hist - Q_hist;
figure
plot(t_hist,Qtil_hist(1,:))

% figure
% plot(t_hist,Qtil_hist(2,:))
% 
% figure
% plot(t_hist,Qtil_hist(3,:))
% 
figure
plot(t_hist,Qtil_hist(4,:))

figure
plot(t_hist,Qtil_hist(5,:))

figure
plot(t_hist,Qtil_hist(6,:))

% figure
% plot(t_hist,sqrt(X_P_hist(7,:).^2+X_P_hist(8,:).^2))

% figure
% plot(t_hist,U_test_hist(2,:))
