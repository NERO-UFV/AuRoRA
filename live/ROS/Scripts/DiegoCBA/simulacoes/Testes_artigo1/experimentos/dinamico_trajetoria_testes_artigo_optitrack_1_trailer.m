clear all
close all
clc

try
    fclose(instrfindall);
catch
end
%
% % Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Class
try
    % Load Classes
    setenv('ROS_MASTER_URI','http://192.168.0.106:11311')
    setenv('ROS_IP','192.168.0.108')
    RI = RosInterface;
    RI.rConnect('192.168.0.106');
    
%     P = RPioneer(1,'RosAria',1);
    P = RPioneer(1,'RosAria',2);
    P2 = Pioneer3DX(1);
%     P3 = Pioneer3DX(2);
    
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
fig1 = figure;
fig2 = figure;

nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Parent',fig2,'Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

%% Variable initialization
data = [];

idP = 1;
idT1 = 2;
% idT2 = 3;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%
rb = OPT.RigidBody;          % read optitrack data
P = getOptData(rb(idP),P);   % get pioneer data
P2 = getOptData(rb(idT1),P2);   % get pioneer data

% Variáveis do Robô
velocidade_real = zeros(4,1); % Velocidades controle do Robô real
U_robo_real = zeros(6,1); % Comandos do Robô
P_robo = zeros(6,1); % Comandos do Robô
P_centro_trailer1 = zeros(6,1);
U1_robo_real_anterior = 0;
U3_robo_real_anterior = 0;

% Cinemática estendida do pioneer
alpha = pi;
sat_v = 0.75;
sat_w = 100*pi/180;
b = 0.2;

% Variaveis para plot corpo
raio = 0.15;
circ = 0:0.01:2*pi;

% Variaveis de Ganho
K_1 = 0.8*diag([1 1]);
K_2 = 0.8*diag([1 1]);
k_u = 4;
k_w = 4;

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_sim = tic; % Temporizador de simulação
t_derivada_real = tic;
t_derivada_sim = tic;
T_CONTROL = 1/10;
t_control = tic;

% thetas primeira identificacao
% t_1 = 0.23025;
% t_2 = 0.22615;
% t_3 = 0.00028953;
% t_4 = 0.95282;
% t_5 = 0.021357;
% t_6 = 0.95282;

% thetas identificacao Diego
% t_1 = 0.29844;
% t_2 = 0.1221;
% t_3 = -0.018223;
% t_4 = 0.86781;
% t_5 = -0.04395;
% t_6 = 0.85486;

t_1=0.23882;
t_2=0.23938;
t_3=0.0038418;
t_4=0.9435;
t_5=-0.007849;
t_6=0.92249;


Theta_original = [t_1;t_2;t_3;t_4;t_5;t_6];

% Variáveis de histórico
t_hist = [];
P_real_X_hist = [];
P2_real_Xtil_hist = [];
P2_real_X_hist = [];
P2_real_Xd_hist = [];
Theta_est_hist_real = [];

% Variaveis da trajetoria
T_MAX = 45;
rX = 2;           % [m]
rY = 1.5;           % [m]
T = T_MAX;             % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;

%%% dimensoes trailers
L0 = 0.3;
L1 = 0.455;
%%% dimensoes trailers simulacao
ret_lg = 0.415;
ret_lp = 0.285;

%%%%% Adaptativo
Theta_til = [0;0;0;0;0;0];
Theta_est_real = Theta_original;

k=1;

parametros{k} = zeros(6,1);
G1{k} = zeros(1,3);
G2{k} = zeros(1,3);

Yfp1 = 0;
Yfp2 = 0;
      
Tfp1 = G1{1};
Tfp2 = G2{1};

Theta1 = classRLS(3);
    Theta1.inicializa(Yfp1(1),G1{1});
Theta2 = classRLS(3);
    Theta2.inicializa(Yfp2(1),G2{1});

theta = zeros(6,1);

X_robo_anterior = zeros(6,1);

n = 200000000000000000000;  %%%% numero de iteracoes para controle adaptativo

fprintf('\nStart..............\n\n');

try
while toc(t) < T_MAX
    if toc(t_control) > T_CONTROL
        
        k = k+1;
        
        t_control = tic;
        t_atual = toc(t);
        
        rb = OPT.RigidBody; 
        P = getOptData(rb(idP),P);   % get pioneer data
        P2 = getOptData(rb(idT1),P2);   % get pioneer data
        
         %%% ---- Trajetoria desejada ------
         
         %%%% circulo
%         Xd_robo([1 2]) = [rX*sin(ww*t_atual + phase); rY*cos(ww*t_atual + phase)-rY];
%         Vd_robo([1 2]) = [ww*rX*cos(ww*t_atual + phase);-ww*rY*sin(ww*t_atual + phase)];

        %%%% lemniscatta real
        P2.pPos.Xd([1 2])= [-rX*sin(ww*t_atual + phase); -rY*sin(2*ww*t_atual + phase)];
        P2.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);-2*ww*rY*cos(2*ww*t_atual + phase)];
        
        x_r = P2.pPos.Xc(1) + L0*cos(P.pPos.Xc(6)) + (L1+b)*cos(P2.pPos.Xc(6));
        y_r = P2.pPos.Xc(2) + L0*sin(P.pPos.Xc(6)) + (L1+b)*sin(P2.pPos.Xc(6));
        
        x_c_t1 = P2.pPos.Xc(1) + b*cos(P2.pPos.Xc(6));
        y_c_t1 = P2.pPos.Xc(2) + b*sin(P2.pPos.Xc(6));
        
        P_robo([1 2]) = ([x_r y_r]);
        P_centro_trailer1([1 2]) = ([x_c_t1 y_c_t1]);

        %%% erros
        P2.pPos.Xtil = P2.pPos.Xd - P2.pPos.Xc;
        
        %%% Velocidades - lei de controle real
        v_real = P2.pPos.Xd([7 8]) + K_1*tanh(K_2*P2.pPos.Xtil([1 2]));
        vx_real = v_real([1]);
        vy_real = v_real([2]);
        v_fi_real = (-vx_real/(b*cos(alpha)))*sin(P2.pPos.Xc([6])) +  (vy_real/(b*cos(alpha)))*cos(P2.pPos.Xc([6]));
        
       velocidade_real([1 2 4]) = [vx_real vy_real v_fi_real];
        
       K_real = [ cos(P2.pPos.Xc([6])), sin(P2.pPos.Xc([6])), b*sin(alpha); ...
            -sin(P2.pPos.Xc([6])), cos(P2.pPos.Xc([6])), -b*cos(alpha); ...
            0, 0, 1];
      
       %%%% sinal de controle cinematico real
       U_trailer1_real = K_real*velocidade_real([1 2 4]);
       U_trailer1_real(2) = 0;
       
       theta_1 = P.pPos.Xc([6]) - P2.pPos.Xc([6]);
       
       v0_c = U_trailer1_real(1)*cos(theta_1) + (U_trailer1_real(3))*(L1)*sin(theta_1);
       w0_c = U_trailer1_real(1)*sin(theta_1)/L0 - (U_trailer1_real(3))*(L1)*cos(theta_1)/L0;
                
       U_robo_real([1 2 3]) = [v0_c 0 w0_c];   %%%% sinal de controle cinematico para o robo1

         %%%% -------------   compensador dinâmico real ---------------
          U1_robo_real_ref = (U_robo_real(1) - U1_robo_real_anterior)/toc(t_derivada_real);
          U3_robo_real_ref = (U_robo_real(3) - U3_robo_real_anterior)/toc(t_derivada_real);
          t_derivada_real = tic;
          U1_robo_real_anterior = U_robo_real(1);
          U3_robo_real_anterior = U_robo_real(3);
          
          G_real = [0 0 -P.pPos.Xc(12)^2 P.pPos.Xc(7) 0 0; ...
               0 0 0 0 P.pPos.Xc(12)*P.pPos.Xc(7) P.pPos.Xc(12)];
          
          delta_1_real = U1_robo_real_ref + k_u*(U_robo_real(1) - P.pPos.Xc(7));
          delta_2_real = U3_robo_real_ref + k_w*(U_robo_real(3) - P.pPos.Xc(12));

        %%% -----  Controle Adaptativo real-------
        
        Theta_real = Theta_est_real;
        
        %%%%% a partir de k - comeca processo de adaptacao
        if k>=n
          [parametros{k}, G1{k}, G2{k}] = calculo_parametros_simulacao([U1_robo_real_ref; U3_robo_real_ref], [P.pPos.X(7); P.pPos.X(12)], [U_robo_real(1); U_robo_real(3)]);
          [Theta_est_real,Y1,Y2,T1,T2] = RLS_recursivo_simulacao(parametros{k},G1{k},G2{k},[U_robo_real(1); U_robo_real(3)],k,Theta1,Theta2,Yfp1,Yfp2,Tfp1,Tfp2);
            Yfp1 = Y1;
            Yfp2 = Y2;
            Tfp1 = T1;
            Tfp2 = T2;
         end
         
          Theta_til_real = Theta_est_real - Theta_real;
          A_est_real = [Theta_est_real(1) 0; 0 Theta_est_real(2)];
                    
          %%%%% ----- Adaptativo começa a partir da iteracao k ---------
          
          %%%%% comeca com controlador dinamico normal - ate k
          if k<n
          disp('dinamico');
          A_din = [Theta_original(1) 0; 0 Theta_original(2)];   
          P.pSC.Ud(1:2) = A_din*[delta_1_real;delta_2_real] + G_real*Theta_original;
%           P.pSC.Ud(1:2) = U_robo_real([1 3]);
          end
          
          %%%%% a partir de k - comeca com controlador adaptativo
          if k>=n
          disp('adaptativo');
          P.pSC.Ud(1:2) = A_est_real*[delta_1_real;delta_2_real] + G_real*Theta_est_real + G_real*Theta_til_real;
          end
          
          Theta_est_real
          k
        
        P = J.mControl(P);  
        P.rCommand;
              
%         if abs(theta_1) > 95*pi/180     
%         P.rCmdStop;
%         disp('jacknife')
%         end
              
        t_hist = [t_hist toc(t)];
        P_real_X_hist = [P_real_X_hist P.pPos.Xc(1:2)];
        P2_real_Xtil_hist = [P2_real_Xtil_hist P2.pPos.Xtil(1:2)];
        P2_real_X_hist = [P2_real_X_hist P2.pPos.Xc(1:2)];
        P2_real_Xd_hist = [P2_real_Xd_hist P2.pPos.Xd(1:2)];
        Theta_est_hist_real = [Theta_est_hist_real Theta_est_real];
        
        %%
        figure(fig1)
        
        Corpo1_real = [raio*cos(circ);raio*sin(circ)] + P_robo(1:2);
        Corpo_frente = P_robo(1:2) + [(raio+0.15)*cos(P.pPos.Xc(6));(raio+0.15)*sin(P.pPos.Xc(6))];
        
        Corpo2_real = [cos(P2.pPos.Xc(6)) -sin(P2.pPos.Xc(6));sin(P2.pPos.Xc(6)) cos(P2.pPos.Xc(6))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + P_centro_trailer1(1:2);
        haste_L0 = L0*[cos(P.pPos.Xc(6)) -sin(P.pPos.Xc(6));sin(P.pPos.Xc(6)) cos(P.pPos.Xc(6))]*[-1;0] + P_robo(1:2);
        haste_L1 = L1*[cos(P2.pPos.Xc(6)) -sin(P2.pPos.Xc(6));sin(P2.pPos.Xc(6)) cos(P2.pPos.Xc(6))]*[1;0] + P_centro_trailer1(1:2);
        
        plot(P2.pPos.Xd(1),P2.pPos.Xd(2),'bh')
        hold on
        grid on
        plot(P2_real_Xd_hist(1,:),P2_real_Xd_hist(2,:),'b-')
        plot(Corpo1_real(1,:),Corpo1_real(2,:),'k','LineWidth',2)
        plot(P2_real_X_hist(1,:),P2_real_X_hist(2,:),'r--','LineWidth',2)
        plot(Corpo2_real(1,:),Corpo2_real(2,:),'k','LineWidth',2)
        plot([P_robo(1) Corpo_frente(1)],[P_robo(2) Corpo_frente(2)],'k','LineWidth',2)
        plot([P_robo(1) haste_L0(1)],[P_robo(2) haste_L0(2)],'k','LineWidth',2)
        plot([P_centro_trailer1(1) haste_L1(1)],[P_centro_trailer1(2) haste_L1(2)],'k','LineWidth',2)
        plot([haste_L0(1)],[haste_L0(2)],'ko')
        axis([-2.5 2.5 -2 2])
        hold off
        
        drawnow
        
                
%         hold on
%         grid on
%         plot(t_hist(1,:),Xtil_hist(1,:))
%         plot(t_hist(1,:),Xtil_hist(2,:))
%         axis([0 20 -0.5 0.5])
%         hold off

%         hold on
%         grid on
%         plot(t_hist(1,:),P_real_Xtil_hist(1,:))
%         plot(t_hist(1,:),P_real_Xtil_hist(2,:))
%         axis([0 20 -0.5 0.5])
%         hold off  

%         hold on
%         grid on
%         plot(t_hist(1,:),Theta_est_hist_real(1,:))
%         plot(t_hist(1,:),Theta_est_hist_real(2,:))
%         plot(t_hist(1,:),Theta_est_hist_real(3,:))
%         plot(t_hist(1,:),Theta_est_hist_real(4,:))
%         plot(t_hist(1,:),Theta_est_hist_real(5,:))
%         plot(t_hist(1,:),Theta_est_hist_real(6,:))
%         axis([0 t_hist(end) -10 10])
%         hold off
% 
%         hold on
%         grid on
%         plot(t_hist(1,:),Theta_est_hist_sim(1,:))
%         plot(t_hist(1,:),Theta_est_hist_sim(2,:))
%         plot(t_hist(1,:),Theta_est_hist_sim(3,:))
%         plot(t_hist(1,:),Theta_est_hist_sim(4,:))
%         plot(t_hist(1,:),Theta_est_hist_sim(5,:))
%         plot(t_hist(1,:),Theta_est_hist_sim(6,:))
%         axis([0 t_hist(end) -10 10])
%         hold off

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

%Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");
