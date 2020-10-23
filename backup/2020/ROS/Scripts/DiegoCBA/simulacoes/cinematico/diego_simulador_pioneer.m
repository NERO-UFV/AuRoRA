clear all
close all
clc

% Variáveis do Robô
X_robo = zeros(4,1); % posicao atual do robo
Xd_robo = zeros(4,1); % posicoes desejadas da trajetoria
Vd_robo = zeros(4,1); % velocidades desejadas da trajetoria
Xtil = zeros(4,1); % Erros
velocidade = zeros(4,1); % Velocidades controle do Robô
U_robo = zeros(4,1); % Comandos do Robô
dX_robo = zeros(4,1); % Variacoes posicoes do robo entre interacoes

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

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_sim = tic; % Temporizador de simulação
t_der = tic; % Temporizador de derivada
T_MAX = 60;
T_CONTROL = 1/10;
t_control = tic;
t_sim = tic; % Temporizador de simulação

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

% Variaveis da trajetoria
rX = 1.5;           % [m]
rY = 1.5;           % [m]
T = T_MAX;             % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;

while toc(t) < T_MAX
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        
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
                %% Cinemática
        % Matriz de cinemática estendida
       K_inv = [ cos(X_robo([4])), sin(X_robo([4])), b*sin(alpha); ...
            -sin(X_robo([4])), cos(X_robo([4])), -b*cos(alpha); ...
            0, 0, 1];
        
        U_robo = K_inv*velocidade([1 2 4]);
        U_robo(2) = 0;
        
        %Saturação
        if abs(U_robo(1)) > sat_v
            U_robo(1) = sat_v*sign(U_robo(1));
        end
        
        if abs(U_robo(3)) > sat_w
            U_robo(3) = sat_w*sign(U_robo(3));
        end
                
        %% Simulação
        
        % Velocidade do robô na referencia do mundo
        dX_robo([1 2 4]) = K_inv\U_robo([1 2 3]);
        dX_robo(3) = 0;
        
        % Cálculo de posição na referência do mundo
        X_robo = X_robo + dX_robo*toc(t_sim);
        t_sim = tic;
              
        t_hist = [t_hist toc(t)];
        Xd_robo_hist = [Xd_robo_hist Xd_robo(1:2)];
        dX_robo_hist = [dX_robo_hist dX_robo];
        X_robo_hist = [X_robo_hist X_robo];
        Xtil_hist = [Xtil_hist Xtil];
        
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

        drawnow   
        
        
    end
    
end
