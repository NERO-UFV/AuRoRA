
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
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    B = Bebop(1,'B');
    P = RPioneer(1,'P1');
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


%% Variable initialization
X_P_hist = [];
X_B_hist = [];
Qd_hist = [];
Q_hist = [];
dQd_hist = [];
dQ_hist = [];
Vd_hist = [];
dV_P_hist = [];
dV_B_hist = [];
dV_P_ref_hist = [];
dV_B_ref_hist = [];
t_hist = [];

X_P_teste_hist = [];
X_B_teste_hist = [];
% B.rGetSensorDataLocal;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% detect rigid body ID from optitrack
% idP = getID(OPT,P);          % pioneer ID on optitrack
idP = 1;
idB = getID(OPT,B);          % drone ID on optitrack
%
rb = OPT.RigidBody;          % read optitrack data
B = getOptData(rb(idB),B);   % get ardrone data
P = getOptData(rb(idP),P);   % get pioneer data

B.rGetSensorDataOpt;
P.rGetSensorDataOpt;

% Bebop
disp('Start Take Off Timming....');
B.rTakeOff;
pause(3);
disp('Taking Off End Time....');

% Time variables initialization
t_amostragem = 0.1;

t_sw1 = 90;
t_max = t_sw1 + 5;


fprintf('\nStart..............\n\n');

% L1 = diag([1.2 1.2 1 5 1 1.8]);

% L1 = diag([1.2 1.2 1 3 0.75 0.75]);
L1 = diag([1.2 1.2 1 3 1.5 1.5]);

L2 = diag([1 1 1 1 1 1]);

% Pioneer
a_P = 0.15;
Vref_P_A = 0;

% Drone
Ku = diag([0.8417 0.8354 3.966 9.8524]);
Kv = diag([0.18227 0.17095 4.001 4.7295]);
K_bebop = diag([2 2 1.8 5]);
Vref_B_A = 0;
K1_ori = 1;
K2_ori = 1;
pouso = 0;
pousoFinal = 0;

B.pSC.Kinematics_control = 1;

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
t_der = tic;

tol_caminho = 0.2;

%Caminho
Ve = 0.5;
Vd = 0;

raio_a = 0.8;
raio_b = 1.2;
inc = .001;
term = 200;
s = 0:inc:term;
x = raio_a*sin(2*pi*s/100);
y = raio_b*sin(pi*s/100);
z = 0*ones(1,length(s));

C = [x; y; z];
joy = vrjoystick(1);
dist_final = 1000;
dist_term = .1;
tol_final = 0.1;
Vd1A = 0;

rho_pouso = 0.6;
beta_pouso = pi/2.5;
Curvatura_lim = 0.5;
try
    %     while dist_final > dist_term
    while t_max > toc(t)
        if toc(t_control) > t_amostragem
            t_control = tic;
            
            %             B.rGetSensorDataLocal;
            %             P.rGetSensorData;
            %                         B.rGetSensorDataOpt;
            %                         P.rGetSensorDataOpt;
            %
            %             X_P_teste = P.pPos.X;
            %             X_B_teste = B.pPos.X;
            
            rb = OPT.RigidBody;          % read optitrack data
            B = getOptData(rb(idB),B);   % get ardrone data
            P = getOptData(rb(idP),P);   % get pioneer data
            
            X_P = P.pPos.X;
            X_B = B.pPos.X;
            
            F_P = [cos(X_P(6)) -a_P*sin(X_P(6));
                sin(X_P(6)) a_P*cos(X_P(6))];
            
            F_B = [cos(X_B(6)) -sin(X_B(6)) 0 0;
                sin(X_B(6))  cos(X_B(6)) 0 0;
                0            0      1 0;
                0            0      0 1];
            
            
            
            Vd = Ve;
            if abs(P.pSC.Ud(2)) > Curvatura_lim
                Vd = Vd/(abs(P.pSC.Ud(2)) + 1);
            end
            dist_final = norm(C(1:2,end)-X_P(1:2));
            
            [dist, ind] = calcula_ponto_proximo(C(1:2,:),X_P(1:2));
            tan_inc(1,1) = 2*raio_a*pi/100*cos(2*pi*s(ind)/100);
            tan_inc(2,1) = raio_b*pi/100*cos(pi*s(ind)/100);
            
            Qd = [C(1,ind) C(2,ind) X_P(3) 1.2 (X_P(6)) pi/2]';
            
            if t_sw1 < toc(t)
                Vd = 0.0;
                Qd = [X_P(1) X_P(2) X_P(3) 0.75 (X_P(6)+pi) 80*pi/180]';
            end
            
            vtan = Vd*tan_inc/norm(tan_inc);

            if dist > tol_caminho
                dQd = [0 0 0 0 X_P(12) 0]';
            else
                dQd = [vtan' 0 0 X_P(12) 0]';
                if t_sw1 < toc(t)
                    dQd = [0 0 0 0 0 0]';
                end
            end
            
            Rho = norm(X_B(1:3) - X_P(1:3));
            Alpha = atan2(X_B(2) - X_P(2),X_B(1) - X_P(1));
            Beta = atan2(X_B(3) - X_P(3),norm(X_B(1:2) - X_P(1:2)));
            
            Q = [X_P(1) X_P(2) X_P(3) Rho Alpha Beta]';
            dQ = [X_P(7) X_P(8) X_P(10) 0 0 0]'; %Completar depois
            
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
            
            x1 = X_P(1);
            y1 = X_P(2);
            z1 = X_P(3);
            x2 = X_B(1);
            y2 = X_B(2);
            z2 = X_B(3);
            
            % Matriz Jacobiana Direta
            % Linha 4 da Matriz Jacobiana
            J41 = (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
            J42 = (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
            J43 = (z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
            J44 = -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
            J45 = -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
            J46 = -(z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
            
            % Linha 5 da Matriz Jacobiana
            J51 = -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2);
            J52 =  (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2);
            J53 =  0;
            J54 =  (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2);
            J55 = -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2);
            J56 =  0;
            
            % Linha 6 da Matriz Jacobiana
            J61 =  ((2*x1 - 2*x2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2));
            J62 =  ((2*y1 - 2*y2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2));
            J63 = -((x1 - x2)^2 + (y1 - y2)^2)^(1/2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
            J64 = -((2*x1 - 2*x2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2));
            J65 = -((2*y1 - 2*y2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2));
            J66 =  ((x1 - x2)^2 + (y1 - y2)^2)^(1/2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
            
            Jacob = [ 1  ,  0  , 0  , 0  , 0  , 0 ;...
                0  ,  1  , 0  , 0  , 0  , 0 ;...
                0  ,  0  , 1  , 0  , 0  , 0 ;...
                J41, J42, J43, J44, J45, J46;...
                J51, J52, J53, J54, J55, J56;...
                J61, J62, J63, J64, J65, J66 ];
            
            % Split Jacobian in Two Tasks
            Jp = Jacob(1:3,:);
            Js = Jacob(4:6,:);
            
%             dXref = Jacob\dQref;
                        dXref = pinv(Js)*dQref(4:6) + (eye(6) - pinv(Js)*Js)*pinv(Jp)*dQref(1:3);
%                     dXref = pinv(Jp)*dQref(1:3) + (eye(6) - pinv(Jp)*Jp)*pinv(Js)*dQref(4:6);
            
            % Controlador Orientação Bebop
            Otil = X_P(6) - X_B(6);
            if abs(Otil) > pi
                if Otil > 0
                    Otil = -2*pi + Otil;
                else
                    Otil =  2*pi + Otil;
                end
            end
            
            dBref_O = X_P(12) + K1_ori*tanh(K2_ori*Otil);
            
            B.pPos.Xr([7 8 9]) = dXref(4:6);
            
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
            
            Ud_P_ref = Vref_P;
            Ud_P = Vref_P;
            
            Ud_B_ref = dVref_B + K_bebop*(Vref_B - V_B);
            Ud_B = Ku\(dVref_B + K_bebop*(Vref_B - V_B) + Kv*V_B);
            
            
            
            P.pSC.Ud = Ud_P(1:2);
            B.pSC.Ud = [Ud_B(1:3)' 0 0 Ud_B(4)]';
            
            %                         B.pSC.Ud = [0 0 0 0 0 0]';
            
            B = J.mControl(B);
            
            if pousoFinal == 0
                P.rCommand;
            end
            if pouso == 0
                B.rCommand;
            end
            
            % Histórico
            X_P_hist = [X_P_hist X_P];
            X_B_hist = [X_B_hist X_B];
            Qd_hist = [Qd_hist Qd];
            Q_hist = [Q_hist Q];
            dQd_hist = [dQd_hist dQd];
            dQ_hist = [dQ_hist dQ];
            Vd_hist = [Vd_hist Vd];
            dV_P_hist = [dV_P_hist Vref_P];
            dV_B_hist = [dV_B_hist Vref_B];
            dV_P_ref_hist = [dV_P_ref_hist Ud_P_ref];
            dV_B_ref_hist = [dV_B_ref_hist Ud_B_ref];
            t_hist = [t_hist toc(t)];
            
            %             X_P_teste_hist = [X_P_teste_hist X_P_teste];
            %             X_B_teste_hist = [X_B_teste_hist X_B_teste];
            drawnow
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
    pause(1)
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%%
filename = ['bebop_caminho_Curvatura_EN_PrioridadeForma_0.5_ProblemasDroneFugindo_GoPro0113_1m50s_' datestr(now,30) '.mat'];
% save(filename,'X_P_hist','X_B_hist','Qd_hist','Q_hist','dQd_hist','dQ_hist','t_hist','Vd_hist','dV_P_hist','dV_B_hist','dV_P_ref_hist','dV_B_ref_hist')

%% PLOTA RESULTADOS
sl = 1.0;   %'default';    % largura da linha
st = 10;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

raio_a = 0.8;
raio_b = 1.2;
inc = .001;
term = 200;
s = 0:inc:term;
x = raio_a*sin(2*pi*s/100);
y = raio_b*sin(pi*s/100);
z = 0*ones(1,length(s));

figure;
plot3(X_P_hist(1,:),X_P_hist(2,:),zeros(size(X_B_hist(3,:))),'LineWidth',sl)
hold on
plot3(X_B_hist(1,:),X_B_hist(2,:),X_B_hist(3,:),'LineWidth',sl)
plot3(x,y,zeros(1,length(x)),'LineWidth',sl)
lgX = legend('$X_p$','$X_d$','$X_f$');
% xlabel('Time [s]','interpreter','Latex');
% ylabel('Displacement [m]','interpreter','Latex');
% xlim([0 t_hist(end)]);
% ylim(0.25*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
% title('X_f Error')
grid on;

Qtil_hist = Qd_hist - Q_hist;

figure;
subplot(321);
plot(t_hist,Qtil_hist(1,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
lgX = legend('$\widetilde{X}_f$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 t_hist(end)]);
ylim(0.25*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
% title('X_f Error')
grid on;

% figure;
subplot(322);
plot(t_hist,Qtil_hist(2,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
lgX = legend('$\widetilde{Y}_f$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 t_hist(end)]);
ylim(0.25*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
% title('Y_f Error')
grid on;

% figure;
subplot(323);
plot(t_hist,Qtil_hist(3,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
lgX = legend('$\widetilde{Z}_f$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 t_hist(end)]);
ylim(0.25*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
% title('Z_f Error')
grid on;

% figure;
subplot(324);
plot(t_hist,Qtil_hist(4,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
lgX = legend('$\widetilde{\rho}_f$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 t_hist(end)]);
% ylim(1*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
% title('\rho_f Error')
grid on;

Qtil_hist(5,:) = Qtil_hist(5,:)*180/pi;
% figure;
subplot(325);
plot(t_hist,Qtil_hist(5,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
lgX = legend('$\widetilde{\alpha}_f$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [rad]','interpreter','Latex');
xlim([0 t_hist(end)]);
ylim(100*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
% title('\alpha_f Error')
grid on;

Qtil_hist(6,:) = Qtil_hist(6,:).*180/pi;
% figure;
subplot(326);
plot(t_hist,Qtil_hist(6,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
lgX = legend('$\widetilde{\beta}_f$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [rad]','interpreter','Latex');
xlim([0 t_hist(end)]);
ylim(360*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
% title('\beta_f Error')
grid on;

velLinearP = sqrt(X_P_hist(7,:).^2 + X_P_hist(8,:).^2);
figure;
subplot(211);
plot(t_hist,velLinearP(1,:),'LineWidth',sl);
hold on;
plot(t_hist,dV_P_hist(1,:),'--','LineWidth',sl);
plot(t_hist,Vd_hist(1,:),'--','LineWidth',sl);
lgX = legend('$V$','$V_{ref}$','$V_d$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Linear Velocity [m/s]','interpreter','Latex');
xlim([0 t_hist(end)]);
% ylim([-0.02 0.02]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

% figure
subplot(212);
plot(t_hist,X_P_hist(12,:),'LineWidth',sl);
hold on;
plot(t_hist,dV_P_hist(2,:),'--','LineWidth',sl);
lgX = legend('$\omega$','$\omega_{ref}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Angular Velocity [rad/s]','interpreter','Latex');
xlim([0 t_hist(end)]);
% ylim([-0.02 0.02]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

% velLinearD = sqrt(X_B_hist(7,:).^2 + X_B_hist(8,:).^2);
% figure;
% subplot(211);
% plot(t_hist,velLinearD(1,:),'LineWidth',sl);
% hold on;
% plot(t_hist,dV_B_hist(1,:),'--','LineWidth',sl);
% lgX = legend('$V$','$V_{ref}$');
% xlabel('Time [s]','interpreter','Latex');
% ylabel('Linear Velocity [m/s]','interpreter','Latex');
% xlim([0 t_hist(end)]);
% % ylim([-0.02 0.02]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% grid on;
% 
% % figure
% subplot(212);
% plot(t_hist,X_B_hist(12,:),'LineWidth',sl);
% hold on;
% plot(t_hist,dV_B_hist(2,:),'--','LineWidth',sl);
% lgX = legend('$\omega$','$\omega_{ref}$');
% xlabel('Time [s]','interpreter','Latex');
% ylabel('Angular Velocity [rad/s]','interpreter','Latex');
% xlim([0 t_hist(end)]);
% % ylim([-0.02 0.02]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% grid on;
