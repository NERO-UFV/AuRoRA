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
%     RI.rConnect('192.168.0.101');
        setenv('ROS_IP','192.168.0.108')
    setenv('ROS_MASTER_URI','192.168.0.114')
    RI.rConnect('192.168.0.114'); % IP of Master Computer
    B = Bebop(1,'B');
        
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
% Beboop
disp('Start Take Off Timming....');
% B.rTakeOff;
% pause(3);
disp('Taking Off End Time....');

%% Variable initialization
data = [];
B.rGetSensorDataLocal;

% Time variables initialization
T_CONTROL = 0.2; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 120;

T = 30;
w = 2*pi/T;

fprintf('\nStart..............\n\n');

% Parametros dos Controladores 
model = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ];
%          X     Y     Z    Psi
gains = [  1     1     1     1 ...
           1.5   1.5   2     1 ...
           1     1     1     1 ...
           1     1     1     1];
       
% Dinamica Interna Não Modificar
Ku = diag([0.8417 0.8354 3.966 9.8524]);
Kv = diag([0.18227 0.17095 4.001 4.7295]);
K = diag([.8 .8 .8 .8]);

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
t_der = tic;


tol = 0.1;

%Caminho
Ve = 0.1;
Vd = 0;
Vdmax = 0.3;

inc = 0.1;
term = 1500;
s = 0:inc:term;
x = -0.5*cos(pi*s/100) + 1;
y = 0.5*sin(pi*s/100);
z = 1.5*ones(1,length(s));

C = [x; y; z];
joy = vrjoystick(1);
dist_final = 1000;
tol_final = 0.1;
Vd1A = 0;

try    
    while toc(t) < T_MAX

        if toc(t_control) > T_CONTROL             

            B.pPar.Ts = toc(t_incB);        
            t_incB = tic;
            t_control = tic;
            
            % Ardrone
            B.rGetSensorDataLocal;
            
%             a_cont = axis(joy);
%             Vd = -Ve*a_cont(1)
%             Vd = 0.2;
            b_cont = button(joy);
            if b_cont(2) == 1 && Vd < Vdmax
                Vd = Vd + Ve
                disp('aumenta')
            end
            
            if b_cont(3) == 1 && Vd > -Vdmax
                Vd = Vd - Ve
                disp('diminui')
            end
            
            X = B.pPos.X;
            dist_final = norm(C(:,end)-X(1:3));
            
            [dist, ind] = calcula_ponto_proximo(C,X(1:3));
            tan_inc(1,1) = 0.5*pi/100*sin(pi*s(ind)/100);
            tan_inc(2,1) = 0.5*pi/100*cos(pi*s(ind)/100);
            tan_inc(3,1) = 0;
            tan_inc(4,1) = 0;
            vtan = Vd*tan_inc/norm(tan_inc);
            
            Xd = [C(:,ind); 0];
            
            if dist > tol
                %             dXd1 = [vector_vel; 0];
                dXd = [0 0 0 0]';
            else
                dXd = [vtan];
            end
            
            Xtil = Xd(1:4) - X(1:4);
            
            Xref = dXd + diag(gains(1:4))*tanh(diag(gains(5:8))*Xtil);

            F = [  cos(B.pPos.X(6))   -sin(B.pPos.X(6))     0     0; % Cinemática direta
                    sin(B.pPos.X(6))    cos(B.pPos.X(6))     0     0;
                            0                      0               1     0;
                            0                      0               0     1];
                        

            Vd1 = F\Xref;
            
            %Derivando a velocidade desejada
            dVd1 = (Vd1 - Vd1A)/toc(t_der);
            t_der = tic;
            
            Vd1A = Vd1;
            
            %Velocidade do robô em seu próprio eixo
            Vb1 = F\([B.pPos.X(7:9);B.pPos.X(12)]);
            
            Ud1 = Ku\(dVd1 + K*(Vd1 - Vb1) + Kv*Vb1);
            
            B.pSC.Ud = [Ud1(1:3)' 0 0 0]';
%             B.cInverseDynamicController(model,gains);


            %% Save data

            % Variable to feed plotResults function
             data = [  data  ; B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
                toc(t)];

            % Zerando sinais de controle
%             B.pSC.Ud = [0 0 0 0 0 0]';
            %B.pSC.Ud(6) = 0;
            % Beboop
            % Joystick Command Priority
            B = J.mControl(B);                    % joystick command (priority)   
%             B.pSC.Ud
            B.rCommand;

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
Xtil = data(:,1:12) - data(:,13:24);

figure();
hold on;
grid on;
plot(data(:,35),Xtil(:,1));
plot(data(:,35),Xtil(:,2));
plot(data(:,35),Xtil(:,3));
plot(data(:,35),Xtil(:,6));
title('Erro de Posição');
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

% figure();
% subplot(311)
% hold on;
% grid on;
% plot(data(:,35),data(:,7));
% plot(data(:,35),data(:,19));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dX', 'dXd');
% 
% subplot(312)
% hold on;
% grid on;
% plot(data(:,35),data(:,8));
% plot(data(:,35),data(:,20));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dY', 'dYd');
% 
% subplot(313)
% hold on;
% grid on;
% plot(data(:,35),data(:,9));
% plot(data(:,35),data(:,21));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dZ', 'dZd');

% figure
% hold on
% plot(tempo,poses(:,1))
% plot(tempo,poses(:,2))
% title('Posicoes')
% legend('Pos X','Pos Y','Pos Z', 'Ori Z');
% xlabel('Tempo(s)');
% ylabel('Pos');
% hold off

