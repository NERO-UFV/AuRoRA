clear all
close all
clc

% Variáveis do Robô
X_robo = zeros(6,1); % posicao atual do robo
Xd_robo = zeros(6,1); % posicoes desejadas da trajetoria
Vd_robo = zeros(6,1); % velocidades desejadas da trajetoria
Xtil = zeros(6,1); % Erros
velocidade = zeros(4,1); % Velocidades controle do Robô
U_robo = zeros(6,1); % Comandos do Robô
dX_robo = zeros(6,1); % Variacoes posicoes do robo entre interacoes
dX_robo1_anterior = 0;
dX_robo4_anterior = 0;
U1_robo_anterior = 0;
U3_robo_anterior = 0;
K_vel = zeros(5,2);
U_robo_din = zeros(5,1);

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
t_der = tic; % Temporizador de derivada
t_derivada2 = tic;
t_derivada = tic;
T_MAX = 120;
T_CONTROL = 1/10;
t_control = tic;
t_sim = tic; % Temporizador de simulação

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
X1_hist = [];
dX1_hist = [];
Xd1_hist = [];
dXd1_hist = [];
Xtil_hist = [];
X_robo_hist = [];
U1_hist = [];
Xd_robo_hist = [];
dX_robo_hist = [];
U_robo_hist = [];
Theta_est_hist = [];

% Variaveis da trajetoria
rX = 1.5;           % [m]
rY = 1.5;           % [m]
T = T_MAX/2;             % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;

%%%%% Adaptativo
Theta_til = [0;0;0;0;0;0];
Theta_est = Theta_original;

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

n = 100;  %%%% numero de iteracoes para controle adaptativo

while toc(t) < T_MAX
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        
        k = k+1;
         %%% ---- Trajetoria desejada ------
         
         %%%% circulo
%         Xd_robo([1 2]) = [rX*sin(ww*t_atual + phase); rY*cos(ww*t_atual + phase)-rY];
%         Vd_robo([1 2]) = [ww*rX*cos(ww*t_atual + phase);-ww*rY*sin(ww*t_atual + phase)];

        %%%% lemniscatta
        Xd_robo([1 2]) = [rX*sin(ww*t_atual + phase); rY*sin(2*ww*t_atual + phase)];
        Vd_robo([1 2]) = [ww*rX*cos(ww*t_atual + phase);2*ww*rY*cos(2*ww*t_atual + phase)];
               
        Xtil = Xd_robo - X_robo;
        
        %%% Velocidades - lei de controle
        v = Vd_robo([1 2]) + K_1*tanh(K_2*Xtil([1 2]));
        vx = v([1]);
        vy = v([2]);
        v_fi = (-vx/(b*cos(alpha)))*sin(X_robo([4])) +  (vy/(b*cos(alpha)))*cos(X_robo([4]));
        
       velocidade([1 2 4]) = [vx vy v_fi];
        
       K = [ cos(X_robo([4])), sin(X_robo([4])), b*sin(alpha); ...
            -sin(X_robo([4])), cos(X_robo([4])), -b*cos(alpha); ...
            0, 0, 1];
      
        %%%% sinal de controle cinematico
       U_robo = K*velocidade([1 2 4]);
       U_robo(2) = 0;
       
         %%%% -------------   compensador dinâmico ---------------
          U1_robo_ref = (U_robo(1) - U1_robo_anterior)/toc(t_derivada);  
          U3_robo_ref = (U_robo(3)- U3_robo_anterior)/toc(t_derivada);
          t_derivada = tic;
          U1_robo_anterior = U_robo(1);
          U3_robo_anterior = U_robo(3);
          
          G = [0 0 -X_robo(6)^2 X_robo(5) 0 0; ...
               0 0 0 0 X_robo(6)*X_robo(5) X_robo(6)];
          
        delta_1 = U1_robo_ref  + k_u*(U_robo(1) - X_robo(5));
        delta_2 = U3_robo_ref + k_w*(U_robo(3)- X_robo(6));
 
        %%% -----  Controle Adaptativo -------
        
        Theta = Theta_est;
        
        %%%%% a partir de k - comeca processo de adaptacao
        if k>=n/2
          [parametros{k}, G1{k}, G2{k}] = calculo_parametros_simulacao([dX_robo(5); dX_robo(6)], [X_robo(5); X_robo(6)], [U_robo(1); U_robo(3)]);
          [Theta_est,Y1,Y2,T1,T2] = RLS_recursivo_simulacao(parametros{k},G1{k},G2{k},[U_robo(1); U_robo(3)],k,Theta1,Theta2,Yfp1,Yfp2,Tfp1,Tfp2);
            Yfp1 = Y1;
            Yfp2 = Y2;
            Tfp1 = T1;
            Tfp2 = T2;
         end
         
          Theta_til = Theta_est - Theta;
          A_est = [Theta_est(1) 0; 0 Theta_est(2)];
          
          Theta_est
                    
          %%%%% ----- Adaptativo começa a partir da iteracao k ---------
          
          %%%%% comeca com controlador dinamico normal - ate k
          if k<n
          A_din = [Theta_original(1) 0; 0 Theta_original(2)];   
          U_robo([1 3]) = A_din*[delta_1;delta_2] + G*Theta_original;
          end
          
          %%%%% a partir de k - comeca com controlador adaptativo
          if k>=n
          U_robo([1 3]) = A_est*[delta_1;delta_2] + G*Theta_est + G*Theta_til;
          end
          
          
           %Saturação
        if abs(U_robo(1)) > sat_v
            U_robo(1) = sat_v*sign(U_robo(1));
        end
        
        if abs(U_robo(3)) > sat_w
            U_robo(3) = sat_w*sign(U_robo(3));
        end
        
        %%% Modelo Dinamico Pioneer
        K_din = [(Theta_original(3)/Theta_original(1))*X_robo(6)^2 - (Theta_original(4)/Theta_original(1))*X_robo(5); -(Theta_original(5)/Theta_original(2))*X_robo(6)*X_robo(5) - (Theta_original(6)/Theta_original(2))*X_robo(6)];
        K_vel = [1/Theta_original(1) 0; 0 1/Theta_original(2)];
        ddXd_robo = K_din+K_vel*U_robo([1 3]);
        
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
        U_robo_hist = [U_robo_hist U_robo];
        Theta_est_hist = [Theta_est_hist Theta_est];
        
        %%
        
        Corpo = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);
        Corpo_frente = X_robo(1:2) + [(raio+1)*cos(X_robo(4));(raio+1)*sin(X_robo(4))];
       
        plot(Xd_robo(1),Xd_robo(2),'x')
        hold on
        grid on
        plot(Xd_robo_hist(1,:),Xd_robo_hist(2,:),'-')
        plot(X_robo_hist(1,:),X_robo_hist(2,:),'o')
        plot(Corpo(1,:),Corpo(2,:))
        plot([X_robo(1) Corpo_frente(1)],[X_robo(2) Corpo_frente(2)])
        axis([-2 2 -2 2])
        hold off
                
%         hold on
%         grid on
%         plot(t_hist(1,:),Xtil_hist(1,:))
%         plot(t_hist(1,:),Xtil_hist(2,:))
%         axis([0 20 -0.5 0.5])
%         hold off
% 
%         hold on
%         grid on
%         plot(t_hist(1,:),Theta_est_hist(1,:))
%         plot(t_hist(1,:),Theta_est_hist(2,:))
%         plot(t_hist(1,:),Theta_est_hist(3,:))
%         plot(t_hist(1,:),Theta_est_hist(4,:))
%         plot(t_hist(1,:),Theta_est_hist(5,:))
%         plot(t_hist(1,:),Theta_est_hist(6,:))
%         axis([0 t_hist(end) -10 10])
%         hold off



        drawnow   
        
        
    end
    
end
