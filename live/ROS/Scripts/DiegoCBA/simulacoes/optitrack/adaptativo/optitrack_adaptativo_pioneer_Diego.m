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
    setenv('ROS_MASTER_URI','http://192.168.0.109:11311')
    setenv('ROS_IP','192.168.0.104')
    RI = RosInterface;
    RI.rConnect('192.168.0.109');
    
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
fig1 = figure;
fig2 = figure;

nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Parent',fig1,'Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

%% Variable initialization
data = [];

idP = 1;
% idT1 = 2;
% idT2 = 3;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%
rb = OPT.RigidBody;          % read optitrack data
P = getOptData(rb(idP),P);   % get pioneer data
% P2 = getOptData(rb(idT1),P2);   % get pioneer data

% Variáveis do Robô
X_robo = zeros(6,1); % posicao atual do robo
Xd_robo = zeros(6,1); % posicoes desejadas da trajetoria
Vd_robo = zeros(6,1); % velocidades desejadas da trajetoria
Xtil = zeros(6,1); % Erros
velocidade_real = zeros(4,1); % Velocidades controle do Robô real
velocidade_sim = zeros(4,1); % Velocidades controle do Robô simulacao
U_robo_sim = zeros(6,1); % Comandos do Robô
U_robo_real = zeros(6,1); % Comandos do Robô
dX_robo = zeros(6,1); % Variacoes posicoes do robo entre interacoes
U1_robo_sim_anterior = 0;
U3_robo_sim_anterior = 0;
U1_robo_real_anterior = 0;
U3_robo_real_anterior = 0;
K_vel = zeros(5,2);

% Cinemática estendida do pioneer
alpha = 0;
sat_v = 0.75;
sat_w = 100*pi/180;
b = 0.2;

% Variaveis para plot corpo
raio = 0.15;
circ = 0:0.01:2*pi;

% Variaveis de Ganho
K_1 = diag([1 1]);
K_2 = diag([1 1]);
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
theta_1 = 0.23025;
theta_2 = 0.22615;
theta_3 = 0.00028953;
theta_4 = 0.95282;
theta_5 = 0.021357;
theta_6 = 0.95282;

Theta_original = [theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];

% Variáveis de histórico
t_hist = [];
Xtil_hist = [];
X_robo_hist = [];
Xd_robo_hist = [];
dX_robo_hist = [];
P_real_Xtil_hist = [];
P_real_X_hist = [];
P_real_Xd_hist = [];
Theta_est_hist_real = [];
Theta_est_hist_sim = [];

% Variaveis da trajetoria
T_MAX = 180;
rX = 2;           % [m]
rY = 1.5;           % [m]
T = T_MAX/2;             % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;

%%%%% Adaptativo
Theta_til = [0;0;0;0;0;0];
Theta_est_real = Theta_original;
Theta_est_sim = Theta_original;

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

n = 350;  %%%% numero de iteracoes para controle adaptativo

fprintf('\nStart..............\n\n');

% try
while toc(t) < T_MAX
    if toc(t_control) > T_CONTROL
        
        k = k+1;
        
        t_control = tic;
        t_atual = toc(t);
        
        rb = OPT.RigidBody; 
        P = getOptData(rb(idP),P);   % get pioneer data
        
         %%% ---- Trajetoria desejada ------
         
         %%%% circulo
%         Xd_robo([1 2]) = [rX*sin(ww*t_atual + phase); rY*cos(ww*t_atual + phase)-rY];
%         Vd_robo([1 2]) = [ww*rX*cos(ww*t_atual + phase);-ww*rY*sin(ww*t_atual + phase)];

        %%%% lemniscatta real
        P.pPos.Xd([1 2])= [rX*sin(ww*t_atual + phase); rY*sin(2*ww*t_atual + phase)];
        P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);2*ww*rY*cos(2*ww*t_atual + phase)];
        
        %%%% lemniscatta simulacao
        Xd_robo([1 2]) = [rX*sin(ww*t_atual + phase); rY*sin(2*ww*t_atual + phase)];
        Vd_robo([1 2]) = [ww*rX*cos(ww*t_atual + phase);2*ww*rY*cos(2*ww*t_atual + phase)];

        %%% erros
        Xtil = Xd_robo - X_robo;
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        %%% Velocidades - lei de controle real
        v_real = P.pPos.Xd([7 8]) + K_1*tanh(K_2*P.pPos.Xtil([1 2]));
        vx_real = v_real([1]);
        vy_real = v_real([2]);
        v_fi_real = (-vx_real/(b*cos(alpha)))*sin(P.pPos.X([6])) +  (vy_real/(b*cos(alpha)))*cos(P.pPos.X([6]));
        
       velocidade_real([1 2 4]) = [vx_real vy_real v_fi_real];
        
       K_real = [ cos(P.pPos.X([6])), sin(P.pPos.X([6])), b*sin(alpha); ...
            -sin(P.pPos.X([6])), cos(P.pPos.X([6])), -b*cos(alpha); ...
            0, 0, 1];
        
        %%% Velocidades - lei de controle simulacao
        v_sim = Vd_robo([1 2]) + K_1*tanh(K_2*Xtil([1 2]));
        vx_sim = v_sim([1]);
        vy_sim = v_sim([2]);
        v_fi_sim = (-vx_sim/(b*cos(alpha)))*sin(X_robo([4])) +  (vy_sim/(b*cos(alpha)))*cos(X_robo([4]));
        
       velocidade_sim([1 2 4]) = [vx_sim vy_sim v_fi_sim];
        
       K_sim = [ cos(X_robo([4])), sin(X_robo([4])), b*sin(alpha); ...
            -sin(X_robo([4])), cos(X_robo([4])), -b*cos(alpha); ...
            0, 0, 1];
      
       %%%% sinal de controle cinematico real
       U_robo_real = K_real*velocidade_real([1 2 4]);
       U_robo_real(2) = 0;
               
       %%%% sinal de controle cinematico simulacao
       U_robo_sim = K_sim*velocidade_sim([1 2 4]);
       U_robo_sim(2) = 0;
       
         %%%% -------------   compensador dinâmico real ---------------
          U1_robo_real_ref = (U_robo_real(1) - U1_robo_real_anterior)/toc(t_derivada_real);
          U3_robo_real_ref = (U_robo_real(3) - U3_robo_real_anterior)/toc(t_derivada_real);
          t_derivada_real = tic;
          U1_robo_real_anterior = U_robo_real(1);
          U3_robo_real_anterior = U_robo_real(3);
          
          G_real = [0 0 -P.pPos.X(12)^2 P.pPos.X(7) 0 0; ...
               0 0 0 0 P.pPos.X(12)*P.pPos.X(7) P.pPos.X(12)];
          
          delta_1_real = U1_robo_real_ref + k_u*(U_robo_real(1) - P.pPos.X(7));
          delta_2_real = U3_robo_real_ref + k_w*(U_robo_real(3) - P.pPos.X(12));

%           A_real = [Theta(1) 0; 0 Theta(2)];
         
          %%%% -------------   compensador dinâmico simulacao ---------------
          U1_robo_sim_ref = (U_robo_sim(1) - U1_robo_sim_anterior)/toc(t_derivada_sim);
          U3_robo_sim_ref = (U_robo_sim(3) - U3_robo_sim_anterior)/toc(t_derivada_sim);
          t_derivada_sim = tic;
          U1_robo_sim_anterior = U_robo_sim(1);
          U3_robo_sim_anterior = U_robo_sim(3);
          
          G_sim = [0 0 -X_robo(6)^2 X_robo(5) 0 0; ...
               0 0 0 0 X_robo(6)*X_robo(5) X_robo(6)];
          
          delta_1_sim = U1_robo_sim_ref + k_u*(U_robo_sim(1) - X_robo(5));
          delta_2_sim = U3_robo_sim_ref + k_w*(U_robo_sim(3) - X_robo(6));

%          A_sim = [Theta(1) 0; 0 Theta(2)];

        %%% -----  Controle Adaptativo real-------
        
        Theta_real = Theta_est_real;
        
        %%%%% a partir de k - comeca processo de adaptacao
        if k>=n/2
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
          A_din = [Theta_original(1) 0; 0 Theta_original(2)];   
          P.pSC.Ud(1:2) = A_din*[delta_1_real;delta_2_real] + G_real*Theta_original;
          end
          
          %%%%% a partir de k - comeca com controlador adaptativo
          if k>=n
          P.pSC.Ud(1:2) = A_est_real*[delta_1_real;delta_2_real] + G_real*Theta_est_real + G_real*Theta_til_real;
          end

         %%% -----  Controle Adaptativo simulacao -------
        
        Theta_sim = Theta_est_sim;
        
        %%%%% a partir de k - comeca processo de adaptacao
        if k>=n/2
          [parametros{k}, G1{k}, G2{k}] = calculo_parametros_simulacao([dX_robo(5); dX_robo(6)], [X_robo(5); X_robo(6)], [U_robo_sim(1); U_robo_sim(3)]);
          [Theta_est_sim,Y1,Y2,T1,T2] = RLS_recursivo_simulacao(parametros{k},G1{k},G2{k},[U_robo_sim(1); U_robo_sim(3)],k,Theta1,Theta2,Yfp1,Yfp2,Tfp1,Tfp2);
            Yfp1 = Y1;
            Yfp2 = Y2;
            Tfp1 = T1;
            Tfp2 = T2;
         end
         
          Theta_til_sim = Theta_est_sim - Theta_sim;
          A_est_sim = [Theta_est_sim(1) 0; 0 Theta_est_sim(2)];
                    
          %%%%% ----- Adaptativo começa a partir da iteracao k ---------
          
          %%%%% comeca com controlador dinamico normal - ate k
          if k<n
          A_din = [Theta_original(1) 0; 0 Theta_original(2)];   
          U_robo_sim([1 3]) = A_din*[delta_1_sim;delta_2_sim] + G_sim*Theta_original;
          end
          
          %%%%% a partir de k - comeca com controlador adaptativo
          if k>=n
          U_robo_sim([1 3]) = A_est_sim*[delta_1_sim;delta_2_sim] + G_sim*Theta_est_sim + G_sim*Theta_til_sim;
          end
          
         %%%% habilita sinal de controle dinamico
%         P.pSC.Ud(1:2) = U_robo_real([1 3]);  %%% cinematico
%         P.pSC.Ud(1:2) = A_real*[delta_1_real;delta_2_real] + G_real*Theta;  %%% dinamico
%         U_robo_sim([1 3]) = A_sim*[delta_1_sim;delta_2_sim] + G_sim*Theta;  %%% simulacao
%         
        %Saturação
        if abs(U_robo_sim(1)) > sat_v
            U_robo_sim(1) = sat_v*sign(U_robo_sim(1));
        end
        
        if abs(U_robo_sim(3)) > sat_w
            U_robo_sim(3) = sat_w*sign(U_robo_sim(3));
        end
        
        P = J.mControl(P);  
        P.rCommand;
        
        %%% Modelo Dinamico Pioneer
        K_din = [(Theta_original(3)/Theta_original(1))*X_robo(6)^2 - (Theta_original(4)/Theta_original(1))*X_robo(5); -(Theta_original(5)/Theta_original(2))*X_robo(6)*X_robo(5) - (Theta_original(6)/Theta_original(2))*X_robo(6)];
        K_vel = [1/Theta_original(1) 0; 0 1/Theta_original(2)];
        ddXd_robo = K_din+K_vel*U_robo_sim([1 3]);
        
        K_direta = [X_robo(5)*cos(X_robo([4]))-b*X_robo(6)*sin(X_robo([4])); ...
                X_robo(5)*sin(X_robo([4]))+b*X_robo(6)*cos(X_robo([4])); ...
                                    0;
                                X_robo(6)];
        K_robo = vertcat(K_direta,ddXd_robo);
        
        %% Simulação
        
        % Velocidade do robô na referencia do mundo
        dX_robo([1 2 3 4 5 6]) = K_robo;
        dX_robo(3) = 0;
        
        % Cálculo de posição na referência do mundo
        X_robo = X_robo + dX_robo*toc(t_sim);
        t_sim = tic;
        
            %Saturação
        if abs(X_robo(5)) > sat_v
            X_robo(5) = sat_v*sign(X_robo(5));
        end
        
        if abs(X_robo(6)) > sat_w
            X_robo(6) = sat_w*sign(X_robo(6));
        end
              
        t_hist = [t_hist toc(t)];
        Xd_robo_hist = [Xd_robo_hist Xd_robo(1:2)];
        dX_robo_hist = [dX_robo_hist dX_robo];
        X_robo_hist = [X_robo_hist X_robo];
        Xtil_hist = [Xtil_hist Xtil];
        P_real_Xtil_hist = [P_real_Xtil_hist P.pPos.Xtil(1:2)];
        P_real_X_hist = [P_real_X_hist P.pPos.X(1:2)];
        P_real_Xd_hist = [P_real_Xd_hist P.pPos.Xd(1:2)];
        Theta_est_hist_real = [Theta_est_hist_real Theta_est_real];
        Theta_est_hist_sim = [Theta_est_hist_sim Theta_est_sim];

        
        %%
        figure(fig2)
        Corpo_sim = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);
        Corpo_frente_sim = X_robo(1:2) + [(raio+0.3)*cos(X_robo(4));(raio+0.3)*sin(X_robo(4))];
       
        Corpo_real = [raio*cos(circ);raio*sin(circ)] + P.pPos.X(1:2);
        Corpo_frente_real = P.pPos.X(1:2) + [(raio+0.3)*cos(P.pPos.X(6));(raio+0.3)*sin(P.pPos.X(6))];
       
        plot(Xd_robo(1),Xd_robo(2),'x')
        hold on
        grid on
        plot(Xd_robo_hist(1,:),Xd_robo_hist(2,:),'b--')
        plot(X_robo_hist(1,:),X_robo_hist(2,:),'r-')
        plot(Corpo_sim(1,:),Corpo_sim(2,:))
        plot([X_robo(1) Corpo_frente_sim(1)],[X_robo(2) Corpo_frente_sim(2)])
        plot(P_real_X_hist(1,:),P_real_X_hist(2,:),'g-')
        plot(Corpo_real(1,:),Corpo_real(2,:))
        plot([P.pPos.X(1) Corpo_frente_real(1)],[P.pPos.X(2) Corpo_frente_real(2)])
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

% catch ME
%     
%     disp('Bebop Landing through Try/Catch Loop Command');
%     P.rCmdStop;
%     
% end
% 
% %Close ROS Interface
% RI.rDisconnect;
% rosshutdown;
% 
% disp("Ros Shutdown completed...");
