%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [ xf yf zf rho alfa beta ]
% Initial Comands

%% Cleaning Variables:
clear;
close all;
clc;


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
%     '192.168.0.104'
%     '192.168.0.129'
    setenv('ROS_IP','192.168.0.108') % % IP of Servo Computer (Windows)
    setenv('ROS_MASTER_URI','192.168.0.114') % IP of Master Computer (Linux)
    RI.rConnect('192.168.0.114'); % IP of Master Computer (Linux)
    B{1} = Bebop(1,'B');
%     B{2} = Bebop(1,'B2');
         
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
B{1}.rTakeOff; %n That's the command that makes the drone takes off.
pause(3);
disp('Taking Off End Time....');

idB = 1;

%% Variable initialization
data1 = [];
qd_hist = [];
q_hist = [];
X1_hist = [];
X2_hist = [];
dX1_hist = [];
dX2_hist = [];
U1_hist = [];
U2_hist = [];

B{1}.rGetSensorDataLocal;
% B{2}.rGetSensorDataLocal;

% Time variables initialization
T_CONTROL = 0.04; % 40 ms de Amostragem | 25 Hz de Frequência
T_MAX = 80; % Maximum time for the experiment [s]
fprintf('\nStart..............\n\n');

% Parametros dos Controladores 
model = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ];
%          X     Y     Z    Psi
gains = [  1.2     1.2     2     2 ...
           1.5   1.5   3     2 ...
           1     1     1     1 ...
           1     1     1     1];


L1 = diag([1, 1, 1, 0, 0, .6]);
L2 = diag([1, 1, 1, 1, 1, 1]);
K = diag([.8, .8, .8, .8]);

% Dinamica Interna Não Modificar
Ku = diag([0.8417 0.8354 3.966 9.8524]);
Kv = diag([0.18227 0.17095 4.001 4.7295]);
 
t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

Vd1A = 0;
Vd2A = 0;

% Creating Time:
T = 0.04;
total_time = 180;

% Defining initial orientation:
% rb = OPT.RigidBody;          % read optitrack data
% B{1} = getOptData(rb(idB),B{1});   % get bebop data
% qd = [rb(idB).Quaternion(2), rb(idB).Quaternion(3), rb(idB).Quaternion(4), rb(idB).Quaternion(1)];
% qd = MyQuaternion(qd(1), qd(2), qd(3), qd(4));
qd = MyQuaternion(1, 0, 0, 0);

% Defining rotation speed:
w = 6;
n = [0, 0, 1];
qd_dot = turn_int_quaternion(w, n);

% Creating the desired orientation:
aux = quat_log(qd_dot);
aux = aux.quaternion();
aux = T * aux;
aux = MyQuaternion(aux(1), aux(2), aux(3), aux(4));
aux = quat_exp(aux);
K1 = 0.1;
K2 = 0.1;

% Saving Data:
xd_hist = [];
x_hist = [];
u_hist = [];
t_hist = [];
q_til_saved = [];
q_saved = [];
qd_saved = [];
control_signal_saved = [];

try    
    while toc(t) < T_MAX

        if toc(t_control) > T_CONTROL             

            B{1}.pPar.Ts = toc(t_incB);
%             B{2}.pPar.Ts = toc(t_incB);
            t_incB = tic;
            t_control = tic;
            
            
            % My Code:
            % Get current orientation:
            rb = OPT.RigidBody;          % read optitrack data
            B{1} = getOptData(rb(idB),B{1});   % get bebop data
            q = [rb(idB).Quaternion(1), rb(idB).Quaternion(2), rb(idB).Quaternion(3), rb(idB).Quaternion(4)];
%             q = [rb(idB).Quaternion(2), rb(idB).Quaternion(1), rb(idB).Quaternion(4), rb(idB).Quaternion(3)];
%             q = [rb(idB).Quaternion(1), rb(idB).Quaternion(2), rb(idB).Quaternion(3), rb(idB).Quaternion(4)];
            q = MyQuaternion(q(1), q(2), q(3), q(4));
            
            % Calculating the error:
            q_til = quat_sub(qd, q);

            % Computing the next desired velocity
            qd = quat_add(qd, aux);
            
            % Computing the control action:
            q_dot_aux = quat_log(q_til);
            q_dot_aux = q_dot_aux.quaternion();
            q_dot_aux = q_dot_aux * K1 * K2;
            q_dot_aux = MyQuaternion(q_dot_aux(1), q_dot_aux(2), q_dot_aux(3), q_dot_aux(4));
            q_dot = quat_add(qd_dot, q_dot_aux);
            control_signal = q_dot.arg();
            
            
            
%             if toc(t) > 10
%                 qd = [0 0 2 9 pi/2 0]';
%                 dqd = [0 0 0 0 0 0]';
%             end
%             qd = [(2 + 3*cos(w*toc(t))) (5 + 3*sin(w*toc(t))) 18 2.5 0 -pi/6]';
%             dqd = [-w*3*sin(w*toc(t)) w*3*cos(w*toc(t)) 0 0 0 0]';
            
%             B{1}.rGetSensorDataLocal;
            
%             qd = path_maker(qd, qd_ponto, sample_rate);
%             qd_saved = qd.arg();
%             rb = OPT.RigidBody;          % read optitrack data
%             B{1} = getOptData(rb(idB),B{1});   % get bebop data
%             aux1 = [rb(idB).Quaternion(2),rb(idB).Quaternion(3),rb(idB).Quaternion(4),rb(idB).Quaternion(1)];
%             q = MyQuaternion(aux1(1), aux1(2), aux1(3), aux1(4));
%             
%             aux2 = controller(qd, q, qd_ponto, 1, 1);
%             aux2.quaternion()
%             q_ponto = aux2.arg();
            
%             % My Doing Begin:
%             if w * toc(t) < pi
%                 psi = w * toc(t);
%             elseif w * toc(t) < 3*pi
%                 psi = w * toc(t) - 2 * pi;
%             elseif w * toc(t) < 5*pi
%                 psi = w * toc(t) - 4 * pi;
%             elseif w * toc(t) < 7*pi
%                 psi = w * toc(t) - 6 * pi;
%             elseif w * toc(t) < 9*pi
%                 psi = w * toc(t) - 8 * pi;
%             elseif w * toc(t) < 11*pi
%                 psi = w * toc(t) - 10 * pi;
%             end
            xd = [0, 0, 1.5, 0, 0, qd.arg()]';
%             dxd = [0, 0, 0, 0, 0, qd.arg()]'; 
%             xtil(1:3) = xd(1:3) - B{1}.pPos.X(1:3);
%             dxref = dxd(1:3) + L1*tanh(L2*xtil(1:3));
            
            % My Doing End:
            
%             rho = norm(B{2}.pPos.X(1:3) - B{1}.pPos.X(1:3));
%             alpha = atan2(B{2}.pPos.X(2) - B{1}.pPos.X(2),B{2}.pPos.X(1) - B{1}.pPos.X(1));
%             beta = atan2(B{2}.pPos.X(3) - B{1}.pPos.X(3),norm(B{2}.pPos.X(1:2) - B{1}.pPos.X(1:2)));
%             q = [B{1}.pPos.X(1) B{1}.pPos.X(2) B{1}.pPos.X(3) rho alpha beta]';
%             
%             qtil = qd - q;
%             
%             dqref = dqd + L1*tanh(L2*qtil);
%             
%             J_inv = [1 0 0           0                       0                          0          ;...
%                      0 1 0           0                       0                          0          ;...
%                      0 0 1           0                       0                          0          ;...
%                      1 0 0 cos(alpha)*cos(beta) -rho*sin(alpha)*cos(beta) -rho*cos(alpha)*sin(beta);...
%                      0 1 0 sin(alpha)*cos(beta)  rho*cos(alpha)*cos(beta) -rho*sin(alpha)*sin(beta);...
%                      0 0 1       sin(beta)                   0                    rho*cos(beta)    ];
%                  
%             dXref = J_inv*dqref;
%             
            F1 = [  cos(B{1}.pPos.X(6))   -sin(B{1}.pPos.X(6))     0     0; % Cinemática direta
                    sin(B{1}.pPos.X(6))    cos(B{1}.pPos.X(6))     0     0;
                            0                      0               1     0;
                            0                      0               0     1];
%                         
%             F2 = [  cos(B{2}.pPos.X(6))   -sin(B{2}.pPos.X(6))     0     0; % Cinemática direta
%                     sin(B{2}.pPos.X(6))    cos(B{2}.pPos.X(6))     0     0;
%                             0                      0               1     0;
%                             0                      0               0     1];
%                         
%             dXd1 = [dXref(1:3); 0];
%             dXd2 = [dXref(4:6); 0];
%             
%             Vd = inv(F1) * [dxref(1:3); dxref(6)];
              Vd = control_signal;

%             
%             %Derivando a velocidade desejada
            dVd = (Vd - Vd1A)/toc(t_incB);
            t_incB = tic;
%             dVd2 = (Vd2 - Vd2A)/toc(t_incB);
%             
            Vd1A = Vd;
%             Vd2A = Vd2;
%             
%             %Velocidade do robô em seu próprio eixo
%             Vb = inv(F1) * ([B{1}.pPos.X(7:9);B{1}.pPos.X(12)]);
            Vb = B{1}.pPos.X(12);
%             Vb2 = F2\([B{2}.pPos.X(7:9);B{2}.pPos.X(12)]);
%             
%             Ud1 = inv(Ku) * (dVd + K*(Vd - Vb) + Kv*Vb);
            Ud1 = inv(Ku(4,4)) * (dVd + K(4,4)*(Vd - Vb) + Kv(4,4)*Vb);
%             Ud2 = Ku\(dVd2 + K*(Vd2 - Vb2) + Kv*Vb2);
%             
%             B{1}.pSC.Ud = [Ud1(1:3)' 0 0 Ud1(4)]';
%             B{1}.pSC.Ud = [0 0 0 0 0 Vd(4)]';
            B{1}.pSC.Ud = [0 0 0 0 0 control_signal]'; % Populating the command input
%             B{1}.pSC.Ud = [0 0 0 0 0 0]'; % Populating the command input with zeros
%             B{1}.pSC.Ud = [0 0 0 0 0 Ud1(4)]';
%             B{2}.pSC.Ud = [Ud2(1:3)' 0 0 0]';
%             
%             % Zerando sinais de controle
%             B{1}.pSC.Ud = [0 0 0 0 0 0]';
% %             B{2}.pSC.Ud = [0 0 0 0 0 0]';
% 
% 
%             % Beboop
%             % Joystick Command Priority
            B{1} = J.mControl(B{1});
%             B{1}.pSC.Ud
%             B{2} = J.mControl(B{2});% joystick command (priority)   
            B{1}.rCommand; % This line sends the commands to the drone
%             B{2}.rCommand;
%             
            xd_hist = [xd_hist xd];
            x_hist = [x_hist B{1}.pPos.X(1:6)];
            u_hist = [u_hist B{1}.pSC.Ud];
            t_hist = [t_hist toc(t)];
            q_til_saved = [q_til_saved, q_til.quaternion()'];
            q_saved = [q_saved, q.quaternion()'];
            qd_saved = [qd_saved, qd.quaternion()'];
            control_signal_saved = [control_signal_saved, control_signal];
%             data1 = [data1  ; qd'  q' toc(t)];
%             qd_hist = [qd_hist; qd'];
%             q_hist = [q_hist; q'];
%             t_hist = [t_hist; toc(t)];
%             X1_hist = [X1_hist; [B{1}.pPos.X(1:3);B{1}.pPos.X(6)]'];
%             X2_hist = [X2_hist; [B{2}.pPos.X(1:3);B{2}.pPos.X(6)]'];
%             dX1_hist = [dX1_hist; [B{1}.pPos.X(7:9);B{1}.pPos.X(12)]'];
%             dX2_hist = [dX2_hist; [B{2}.pPos.X(7:9);B{2}.pPos.X(12)]'];
%             U1_hist = [U1_hist; B{1}.pSC.Ud];
%             U2_hist = [U2_hist; B{2}.pSC.Ud];
%             
            drawnow

            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0 || B{1}.pFlag.EmergencyStop ~= 0 || B{1}.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');
                B{1}.rCmdStop;
%                 B{2}.rCmdStop;
                B{1}.rLand
%                 B{2}.rLand 
                break;
            end  
        end
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B{1}.rCmdStop;
%     B{2}.rCmdStop;
    B{1}.rLand
%     B{2}.rLand         
    disp('');
    disp(ME);
    disp('');
   
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B{1}.rCmdStop;
%     B{2}.rCmdStop;
    B{1}.rLand
%     B{2}.rLand
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%%
% qtil = data1(:,1:6) - data1(:,7:end-1);


% dataPath = strcat(pwd,'\ROS\Scripts\');
% fileName = strcat('Experimento_Formação_Posicao_13','.mat');
% fullFileName = strcat(dataPath,fileName);
% save(fullFileName,'qd_hist','q_hist','t_hist','X1_hist','X2_hist','dX1_hist','dX2_hist','U1_hist','U2_hist');
% %%
% qtil = qd_hist - q_hist;
figure();
hold on;
grid on;
plot(t_hist,xd_hist(6,:));
plot(t_hist,x_hist(6,:));
% plot(t_hist,xd_hist);
% plot(t_hist,x_hist);
title('Erro de Posição');
legend('Psi Desejado', 'Psi Medido');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
hold on;
grid on;
plot(t_hist,u_hist(6,:));
title('Ação de controle');
legend('Psi');
xlabel('Tempo[s]');
ylabel('Ação [m]');
% 
% figure();
% hold on;
% grid on;
% plot(t_hist,qtil(:,2));
% title('Erro de Posição');
% legend('Pos Y');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(t_hist,qtil(:,3));
% title('Erro de Posição');
% legend('Pos Z');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(t_hist,qtil(:,4));
% title('Erro de Forma');
% legend('Rho');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(t_hist,qtil(:,5));
% title('Erro de Forma');
% legend('Alpha');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(t_hist,qtil(:,6));
% title('Erro de Forma');
% legend('Beta');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot3(X1_hist(:,1),X1_hist(:,2),X1_hist(:,3));
% plot3(X2_hist(:,1),X2_hist(:,2),X2_hist(:,3));
% title('Erro de Forma');
% legend('Beta');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');

% % Checking Path:
% figure();
% hold on;
% plot(t_hist, qd_saved(1, 1:end));
% plot(t_hist, qd_saved(2, 1:end));
% plot(t_hist, qd_saved(3, 1:end));
% plot(t_hist, qd_saved(4, 1:end));
% grid on;
% title('Desired Orientation');
% legend('w', 'i', 'j', 'k');

% Plotting:
figure();
subplot(3, 1, 1);
hold on;
plot(t_hist, qd_saved(1, 1:end));
plot(t_hist, qd_saved(2, 1:end));
plot(t_hist, qd_saved(3, 1:end));
plot(t_hist, qd_saved(4, 1:end));
grid on;
title('Desired Orientation');
legend('w', 'i', 'j', 'k');
subplot(3, 1, 2);
hold on;
plot(t_hist, q_saved(1, 1:end));
plot(t_hist, q_saved(2, 1:end));
plot(t_hist, q_saved(3, 1:end));
plot(t_hist, q_saved(4, 1:end));
grid on;
title('Measured Orientation');
legend('w', 'i', 'j', 'k');
subplot(3, 1, 3);
plot(t_hist, control_signal_saved(1:end));
grid on;
title('Euclidian Control Signal');
legend('rad');
figure,
hold on;
plot(t_hist, q_til_saved(1, 1:end));
plot(t_hist, q_til_saved(2, 1:end));
plot(t_hist, q_til_saved(3, 1:end));
plot(t_hist, q_til_saved(4, 1:end));
grid on;
title('Simulated Error');
legend('w', 'i', 'j', 'k');