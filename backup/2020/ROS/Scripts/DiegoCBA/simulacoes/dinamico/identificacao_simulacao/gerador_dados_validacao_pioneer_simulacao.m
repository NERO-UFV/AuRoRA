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

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_sim = tic; % Temporizador de simulação
t_der = tic; % Temporizador de derivada
t_derivada2 = tic;
t_derivada = tic;
T_MAX = 90;
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

Theta = [theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];

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
u_w_d_hist = [];
u_w_hist = [];
t_hist = [];
u_p_hist = [];
w_p_hist = [];

% Variaveis da trajetoria
T = T_MAX;             % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;

X_robo5_anterior = 0;
X_robo6_anterior = 0;

while toc(t) < T_MAX
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        
         %%% ---- Trajetoria desejada ------
         
         %%%% movimentacao pioneer aleatoria
        u_ref = 0.2*(0.2*sin(ww*t_atual) - 0.1*sin(0.4*ww*t_atual) + 0.75*sin(1.5*ww*t_atual) -2*sin(0.2*ww*t_atual) + 4*sin(0.25*ww*t_atual)- 0.35*sin(0.2*ww*t_atual));
        w_ref = 0.12*(sin(ww*t_atual) - 1.8*sin(0.99*ww*t_atual) - 0.5*sin(0.9*ww*t_atual) + 0.9*sin(2*ww*t_atual)- 0.35*sin(4*ww*t_atual)- 0.7*sin(0.45*ww*t_atual));
        
        U_robo([1 3]) = [u_ref;w_ref];
        
        %%% Modelo Dinamico Pioneer
        K_din = [(Theta(3)/Theta(1))*X_robo(6)^2 - (Theta(4)/Theta(1))*X_robo(5); -(Theta(5)/Theta(2))*X_robo(6)*X_robo(5) - (Theta(6)/Theta(2))*X_robo(6)];
        K_vel = [1/Theta(1) 0; 0 1/Theta(2)];
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
        
        X = [X_robo(5) X_robo(6)]
        
        u_p = dX_robo(5);
        w_p = dX_robo(6);
              
        t_hist = [t_hist toc(t)];
        u_w_d_hist = [u_w_d_hist [u_ref w_ref]'];
        u_w_hist = [u_w_hist X_robo([5 6])];
        u_p_hist = [u_p_hist u_p];
        w_p_hist = [w_p_hist w_p];
        X_robo_hist = [X_robo_hist X_robo]; 
        
        %%
        
        Corpo = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);
        Corpo_frente = X_robo(1:2) + [(raio+1)*cos(X_robo(4));(raio+1)*sin(X_robo(4))];
       
        plot(X_robo(1,:),X_robo(2,:))
        hold on
        grid on
        plot(X_robo_hist(1,:),X_robo_hist(2,:),'o')
        plot(Corpo(1,:),Corpo(2,:))
        plot([X_robo(1) Corpo_frente(1)],[X_robo(2) Corpo_frente(2)])
        axis([-20 20 -20 20])
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

%%
path = ['C:\Users\Diego\Desktop\DiegoCBA_17_07\Identificacao Pioneer\simulacoes\dinamico'];
filename = ['Diego_validacao_simulacao1' num2str(T) '_Data_' datestr(now,30) '.mat'];
fullname = [path filename];
save(fullname,'t_hist','u_w_d_hist','u_w_hist','u_p_hist','w_p_hist')
