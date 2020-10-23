clear all
close all
clc

% Variáveis do Robô
X_robo = zeros(4,1); % posicao atual do robo
X_trailer1 = zeros(4,1); % posicao atual do trailer1
Xd_trajetoria = zeros(4,1); % posicoes desejadas da trajetoria
Vd_trajetoria = zeros(4,1); % velocidades desejadas da trajetoria
Xtil = zeros(4,1); % Erros robo
Xtil_trailer1 = zeros(4,1); % Erros trailer 1
velocidade = zeros(4,1); % Velocidades controle
U_robo = zeros(4,1); % Comandos do Robô
U_trailer1 = zeros(4,1); % Comandos do trailer1
dX_robo = zeros(4,1); % Variacoes posicoes do robo entre interacoes
dX_trailer1 = zeros(4,1);% Variacoes posicoes do trailer1 entre interacoes

% Cinemática estendida do pioneer
alpha = pi;
sat_v = 0.75;
sat_w = 100*pi/180;
b = 0.2;

% Variaveis para plot corpo
raio = 0.15;
circ = 0:0.01:2*pi;

% Variaveis de Ganho
K_1 = diag([0.8 0.8]);
K_2 = diag([0.8 0.8]);

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
Xd_trajetoria_hist = [];
dX_robo_hist = [];
dX_trailer1_hist = [];
X_trailer1_hist = [];
Xtil_trailer1_hist = [];
theta_1_hist = [];

% Variaveis da trajetoria
rX = 2;           % [m]
rY = 2;           % [m]
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


while toc(t) < T_MAX
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        
         %%% ---- Trajetoria desejada ------
         
         %%%% circulo
%         Xd_trajetoria([1 2]) = [-rX*sin(ww*t_atual + phase) - (L0+L1); -rY*cos(ww*t_atual + phase)+rY];
%         Vd_trajetoria([1 2]) = [-ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];

        %%%% lemniscatta
        Xd_trajetoria([1 2]) = [-rX*sin(ww*t_atual + phase) - (L0+L1);-rY*sin(2*ww*t_atual + phase)];
        Vd_trajetoria([1 2]) = [-ww*rX*cos(ww*t_atual + phase);-2*ww*rY*cos(2*ww*t_atual + phase)];
        
        %%% ---- Calculo posicao no mundo do trailer 1 ------
        x_t1 = X_robo(1) - L0*cos(X_robo(4)) - (L1+b)*cos(X_trailer1(4));
        y_t1 = X_robo(2) - L0*sin(X_robo(4)) - (L1+b)*sin(X_trailer1(4));
        
        x_t1_c = X_robo(1) - L0*cos(X_robo(4)) - (L1)*cos(X_trailer1(4));
        y_t1_c = X_robo(2) - L0*sin(X_robo(4)) - (L1)*sin(X_trailer1(4));
        
        X_trailer1([1 2]) = ([x_t1 y_t1]);
        X_trailer1_centro = ([x_t1_c;y_t1_c]);  %%% plot
               
%         Xtil = Xd_trajetoria - X_robo;
        Xtil_trailer1 = Xd_trajetoria - X_trailer1;
                
        %%% Velocidades - lei de controle
        v = Vd_trajetoria([1 2]) + K_1*tanh(K_2*Xtil_trailer1([1 2]));
        vx = v([1]);
        vy = v([2]);
        v_fi = (-vx/(b*cos(alpha)))*sin(X_trailer1([4])) +  (vy/(b*cos(alpha)))*cos(X_trailer1([4]));
        
        velocidade([1 2 4]) = [vx vy v_fi];
                
        %% Cinemática
        % Matriz de cinemática estendida do trailer
       K_inv = [ cos(X_trailer1([4])), sin(X_trailer1([4])), b*sin(alpha); ...
            -sin(X_trailer1([4])), cos(X_trailer1([4])), -b*cos(alpha); ...
            0, 0, 1];
        
        U_trailer1 = K_inv*velocidade([1 2 4]);
        U_trailer1(2) = 0;
        
        theta_1 = X_robo(4)-X_trailer1(4);
        
        v0_c = U_trailer1(1)*cos(theta_1) + (U_trailer1(3))*(L1)*sin(theta_1);
        w0_c = U_trailer1(1)*sin(theta_1)/L0 - (U_trailer1(3))*(L1)*cos(theta_1)/L0;
                
        U_robo([1 2 3]) = [v0_c 0 w0_c];
        
        %Saturação
        if abs(U_robo(1)) > sat_v
            U_robo(1) = sat_v*sign(U_robo(1));
        end
        
        if abs(U_robo(3)) > sat_w
            U_robo(3) = sat_w*sign(U_robo(3));
        end
        
        % Matriz de cinemática estendida do robo
        K_inv_robo = [ cos(X_robo([4])), sin(X_robo([4])), 0; ...
            -sin(X_robo([4])), cos(X_robo([4])), 0; ...
            0, 0, 1];
        
        v1 = U_robo(1)*cos(theta_1) + (U_robo(3))*(L0)*sin(theta_1);
        w1 = U_robo(1)*sin(theta_1)/L1 - (U_robo(3))*(L0)*cos(theta_1)/L1;
        U_trailer1([1 2 3]) = [v1 0 w1];
        
        %% Simulação
        % Velocidade do robô na referencia do mundo
        dX_robo([1 2 4]) = K_inv_robo\U_robo([1 2 3]);
        dX_trailer1([1 2 4]) = K_inv\U_trailer1([1 2 3]);
        dX_trailer1(3) = 0;
        dX_robo(3) = 0;
        
        % Cálculo de posição na referência do mundo
        X_robo = X_robo + dX_robo*toc(t_sim);
        X_trailer1 = X_trailer1 + dX_trailer1*toc(t_sim);
        t_sim = tic;
              
        t_hist = [t_hist toc(t)];
        Xd_trajetoria_hist = [Xd_trajetoria_hist Xd_trajetoria(1:2)];
        dX_robo_hist = [dX_robo_hist dX_robo];
        dX_trailer1_hist = [dX_trailer1_hist dX_trailer1];
        X_robo_hist = [X_robo_hist X_robo];
        X_trailer1_hist = [X_trailer1_hist X_trailer1];
        Xtil_hist = [Xtil_hist Xtil];
        X_trailer1_hist = [X_trailer1_hist X_trailer1];
        Xtil_trailer1_hist = [Xtil_trailer1_hist Xtil_trailer1];
        theta_1_hist =[theta_1_hist theta_1];
        
        %%
        
        Corpo1 = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);
        Corpo_frente = X_robo(1:2) + [(raio+0.3)*cos(X_robo(4));(raio+0.3)*sin(X_robo(4))];
        
        Corpo2 = [cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer1_centro(1:2);
        haste_L0 = L0*[cos(X_robo(4)) -sin(X_robo(4));sin(X_robo(4)) cos(X_robo(4))]*[-1;0] + X_robo(1:2);
        haste_L1 = L1*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[1;0] + X_trailer1_centro(1:2);
       
        plot(Xd_trajetoria(1),Xd_trajetoria(2),'bh')
        hold on
        grid on
        plot(Xd_trajetoria_hist(1,:),Xd_trajetoria_hist(2,:),'b-')
        plot(X_robo_hist(1,:),X_robo_hist(2,:),'k','LineWidth',2)
        plot(Corpo1(1,:),Corpo1(2,:),'k','LineWidth',2)
        plot(X_trailer1_hist(1,:),X_trailer1_hist(2,:),'r--','LineWidth',2)
        plot(Corpo2(1,:),Corpo2(2,:),'k','LineWidth',2)
        plot([X_robo(1) Corpo_frente(1)],[X_robo(2) Corpo_frente(2)],'k','LineWidth',2)
        plot([X_robo(1) haste_L0(1)],[X_robo(2) haste_L0(2)],'k','LineWidth',2)
        plot([haste_L0(1)],[haste_L0(2)],'ko')
        plot([X_trailer1_centro(1) haste_L1(1)],[X_trailer1_centro(2) haste_L1(2)],'k','LineWidth',2)
%         axis([-3.5 1.5 -0.5 4.5])  %%% circulo raio 1,5
        axis([-3 1.5 -2.5 2.5])  %%% lemniscata raio 1,5
        hold off
                
%         hold on
%         grid on
%         plot(t_hist(1,:),Xtil_trailer1_hist(1,:))
%         plot(t_hist(1,:),Xtil_trailer1_hist(2,:))
%         legend('erro_x','erro_y')
%         axis([0 T_MAX -0.5 0.5])
%         hold off

%         hold on
%         grid on
%         plot(t_hist(1,:),theta_1_hist(1,:)*180/pi)
%         legend('\theta_1')
%         axis([0 T_MAX -1 50])
%         hold off


        drawnow   
        
        
    end
    
end
