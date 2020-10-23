clear; close all; clc;
try
    fclose(instrfindall);
catch
end
%
% % Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Load Classes

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.103');
%     B = Bebop(1,'B');
    
%     P = Pioneer3DX(1);  % Pioneer Instance
    P = RPioneer(1,'RosAria',1);
    
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
data = [];

% Time variables initialization
T_CONTROL = 1/10; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 40;

Xd_i = 1;
Xd(:,1) = [0.5 0 0 0]';

P.pPos.Xd(1:3) = Xd(1:3,Xd_i);
P.pPos.Xd(6) = Xd(4,Xd_i);

fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
t_derivada = tic;
t_derivada2 = tic;

rX = 1.5;           % [m]
rY = 1.5;           % [m]
T = T_MAX;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;

pgains = [0.35 0.35 0.8 0.8];
Kp1 = diag([pgains(1), pgains(2)]);
Kp2 = diag([pgains(3), pgains(4)]);

k_u = 4;
k_w = 4;

%%% thetas primeira identificacao
% theta_1 = 0.23025;
% theta_2 = 0.22615;
% theta_3 = 0.00028953;
% theta_4 = 0.95282;
% theta_5 = 0.021357;
% theta_6 = 0.95282;

%%% thetas filipe martins
% theta_1 = 0.2604;
% theta_2 = 0.2509;
% theta_3 = -0.000499;
% theta_4 = 0.9965;
% theta_5 = 0.00263;
% theta_6 = 1.0768;

%%% thetas segunda identificacao
theta_1_r = 0.22475;
theta_2_r = 0.19672;
theta_3_r = 0.0026587;
theta_4_r = 0.97439;
theta_5_r = 0.026025;
theta_6_r = 0.93105;

u_ref_anterior = 0;
w_ref_anterior = 0;
u_robo_anterior = 0;
w_robo_anterior = 0; 
u_p_ref = 0;
w_p_ref = 0;

U_hist = [];

Theta = [theta_1_r;theta_2_r;theta_3_r;theta_4_r;theta_5_r;theta_6_r];

xd_hist = [];
x_hist = [];
u_ref_hist = [];
w_ref_hist = [];
u_ref_robo_hist = [];
w_ref_robo_hist = [];
t_hist = [];
vel_linear_desejada_hist = [];
vel_angular_desejada_hist = [];
u_p_ref_hist = [];
w_p_ref_hist = [];
u_robo_hist = [];
w_robo_hist = [];
u_p_robo_hist = [];
w_p_robo_hist = [];
Theta_est_hist = [];

Theta_til = [0;0;0;0;0;0];
Theta_est = [theta_1_r+0.1;theta_2_r+0.1;theta_3_r+0.1;theta_4_r+0.1;theta_5_r+0.1;theta_6_r+0.1];

z = 0;
k=1;

parametros{k} = zeros(6,1);
G1{k} = zeros(1,3);
G2{k} = zeros(1,3);

u(1,k) = 0;
u(2,k) = 0;

tempo(k) = z;

Yfp1 = 0;
Yfp2 = 0;
      
Tfp1 = G1{1};
Tfp2 = G2{1};

Theta1 = classRLS(3);
    Theta1.inicializa(Yfp1(1),G1{1});
Theta2 = classRLS(3);
    Theta2.inicializa(Yfp2(1),G2{1});

Yfp1 = vertcat(u(1,1));
Yfp2 = vertcat(u(2,1));

theta = zeros(6,1);
% parametros = zeros(6,1);

try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
%% POSIÇÃO            
            k = k+1;
            % Dados Odometria
            P.rGetSensorData;
            
%             disp(k);
            
%             P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual + phase);-rY*cos(ww*t_atual + phase)+rY];
%             P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];
            
            P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual + phase);rY*sin(2*ww*t_atual + phase)];
            P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);2*ww*rY*cos(2*ww*t_atual + phase)];
            
         %%%% -------------   compensador cinematico ---------------
           
          K = [cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
               sin(P.pPos.X(6)), P.pPar.a*cos(P.pPos.X(6))];
                        
          P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
            
          sinal_cinematico = K\(P.pPos.Xd(7:8) + Kp1*tanh(Kp2*P.pPos.Xtil(1:2)));
                    
          u_ref = sinal_cinematico(1);
          w_ref = sinal_cinematico(2);
          
          %%%% --------Medicao de velocidade e aceleracao do robo --------       
          u_robo = P.pPos.X(7);        
          w_robo = P.pPos.X(12);          
          u_p_robo = (u_robo - u_robo_anterior)/toc(t_derivada2);
          w_p_robo = (w_robo - w_robo_anterior)/toc(t_derivada2);
          t_derivada2 = tic;
          u_robo_anterior = u_robo;
          w_robo_anterior = w_robo;          
          %%%%%% -------------------------------------

          %%%% -------------   compensador dinâmico ---------------
          u_p_ref = (u_ref - u_ref_anterior)/toc(t_derivada);
          w_p_ref = (w_ref - w_ref_anterior)/toc(t_derivada);
          t_derivada = tic;
          u_ref_anterior = u_ref;
          w_ref_anterior = w_ref;
          
          G = [0 0 -P.pPos.X(12)^2 P.pPos.X(7) 0 0; ...
               0 0 0 0 P.pPos.X(12)*P.pPos.X(7) P.pPos.X(12)];
          
          delta_1 = u_p_ref + k_u*(u_ref - P.pPos.X(7));
          delta_2 = w_p_ref + k_w*(w_ref - P.pPos.X(12));
          
          Theta = Theta_est;
          
          tic
          if k>10
          [parametros{k}, G1{k}, G2{k}] = calculo_parametros([u_p_robo; w_p_robo], [u_robo; w_robo], [u_ref; w_ref]);
          [Theta_est,Y1,Y2,T1,T2] = RLS_recursivo(parametros{k}, G1{k}, G2{k}, [u_ref; w_ref],k,Theta1,Theta2,Yfp1,Yfp2,Tfp1,Tfp2);
            Yfp1 = Y1;
            Yfp2 = Y2;
            Tfp1 = T1;
            Tfp2 = T2;
          end
%           Theta_est = RLS_recursivo([u_p_robo; w_p_robo],[u_robo; w_robo],[u_ref; w_ref]);
          toc
          Theta_til = Theta_est - Theta;
          A = [Theta_est(1) 0; 0 Theta_est(2)];
          
          P.pSC.Ud(1:2) = A*[delta_1;delta_2] + G*Theta_est + G*Theta_til;
          
          %%%%%%%%%%% ------------------------------------------
          
          %  P.pSC.Ud(1:2) = sinal_cinematico;

          P = J.mControl(P);                    % joystick command (priority)

          P.rCommand;
          
          xd_hist = [xd_hist P.pPos.Xd(1:2)];
          x_hist = [x_hist P.pPos.X(1:2)];
          t_hist = [t_hist toc(t)];
          u_ref_hist = [u_ref_hist u_ref];
          w_ref_hist = [w_ref_hist w_ref];
          u_ref_robo_hist = [u_ref_robo_hist P.pSC.Ud(1)];
          w_ref_robo_hist = [w_ref_robo_hist P.pSC.Ud(2)];
          u_p_ref_hist = [u_p_ref_hist u_p_ref];
          w_p_ref_hist = [w_p_ref_hist w_p_ref];
          u_robo_hist = [u_robo_hist u_robo];
          w_robo_hist = [w_robo_hist w_robo];
          u_p_robo_hist = [u_p_robo_hist u_p_robo];
          w_p_robo_hist = [w_p_robo_hist w_p_robo];
          Theta_est_hist = [Theta_est_hist Theta_est];
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            drawnow
            if btnEmergencia ~= 0 
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                
                break;
            end
            
            
        end
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    P.rCmdStop;
    
end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:));
plot(x_hist(1,:),x_hist(2,:));
legend('desejado','real')
xlabel('X [m]');
ylabel('Y [m]');

figure();
hold on;
grid on;
plot(t_hist(1,:),xd_hist(1,:) - x_hist(1,:));
plot(t_hist(1,:),xd_hist(2,:) - x_hist(2,:));
legend('erro x','erro y')
xlabel('t');
ylabel('erro');

figure();
hold on;
grid on;
plot(t_hist(1,:),w_ref_hist(1,:));
plot(t_hist(1,:),w_ref_robo_hist(1,:));
legend('veloc angular cinematico','veloc angular dinamico')
xlabel('t');
ylabel('sinal');

figure();
hold on;
grid on;
plot(t_hist(1,:),u_ref_hist(1,:));
plot(t_hist(1,:),u_ref_robo_hist(1,:));
legend('veloc linear cinematico','veloc linear dinamico')
xlabel('t');
ylabel('sinal');

figure();
hold on;
grid on;
plot(t_hist(1,:),Theta_est_hist);
legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5''\theta_6');
xlabel('t');
ylabel('thetas');


%%
% path = [pwd '\ROS\Scripts\DiegoCBA\Identificacao Pioneer\'];
% filename = ['Diego_teste_lemniscata_cinematico' num2str(T) '_Data_' datestr(now,30) '.mat'];
% fullname = [filename];
% save(fullname,'xd_hist','x_hist','t_hist','u_ref_hist','w_ref_hist','u_ref_robo_hist','w_ref_robo_hist')