clear all;
close all;
clc;

% Variáveis do Robô
X_robo = zeros(4,1); % posicao atual do robo
X_trailer1 = zeros(4,1); % posicao atual do trailer1
X_trailer2 = zeros(4,1); % posicao atual do trailer2
X_trailer2_aux = zeros(4,1); % posicao atual do trailer2
Xd_trajetoria = zeros(4,1); % posicoes desejadas da trajetoria
Vd_trajetoria = zeros(4,1); % velocidades desejadas da trajetoria
Xtil = zeros(4,1); % Erros robo
Xtil_trailer1 = zeros(4,1); % Erros trailer 1
Xtil_trailer2 = zeros(4,1); % Erros trailer 2

velocidade = zeros(4,1); % Velocidades controle
velocidade_plot = zeros(4,1); % Velocidades controle

U_robo = zeros(4,1); % Comandos do Robô
U_trailer1 = zeros(4,1); % Comandos do trailer1
U_trailer2 = zeros(4,1); % Comandos do trailer2
dX_robo = zeros(4,1); % Variacoes posicoes do robo entre interacoes
dX_trailer1 = zeros(4,1);% Variacoes posicoes do trailer1 entre interacoes
dX_trailer2 = zeros(4,1);% Variacoes posicoes de interesse do controle do trailer2 entre interacoes
dX_trailer2_aux = zeros(4,1);% Variacoes posicoes de interesse do controle do trailer2 entre interacoes

% X_trailer2_aux = zeros(4,1);

% Cinemática estendida do pioneer
alpha = pi;    %%%% angulo cinematica estendida - alpha = pi (movimento a re)/ alpha = 0 (movimento frente)
sat_v = 0.75;   %%% saturacao velocidade linear Pioneer
sat_w = 100*pi/180;   %%% saturacao velocidade angular Pioneer
b = 0.2075;    %%% ponto de controle distante do centro do ultimo trailer

% Variaveis para plot corpo Pioneer
raio = 0.15;
circ = 0:0.01:2*pi;

% Variaveis de Ganho
K_1 = diag([0.8 0.8]);
K_2 = diag([0.8 0.8]);

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_sim = tic; % Temporizador de simulação
t_der = tic; % Temporizador de derivada
T_MAX = 120; % Tempo maximo experimento (velocidade do robo)
T_CONTROL = 1/10;  % Tempo de cada iteracao de controle 
t_control = tic; % Temporizador de controle
t_sim = tic; % Temporizador de simulação

% Variáveis de histórico
t_hist = [];
Xd_trajetoria_hist = [];
X_robo_hist = [];
X_trailer1_hist = [];
X_trailer2_hist = [];
X_trailer2_aux_hist =[];
dX_robo_hist = [];
dX_trailer1_hist = [];
dX_trailer2_hist = [];
Xtil_robo_hist = [];
Xtil_trailer1_hist = [];
Xtil_trailer2_hist = [];
theta_1_hist = [];
theta_2_hist = [];


% Variaveis da trajetoria
rX = 2;           % [m]
rY = 2;           % [m]
% rX = 8;           % [m]
% rY = 5;           % [m]
T = T_MAX;             % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;

%%% dimensoes trailers
L0 = 0.3;    % distancia entre o engate e centro das rodas do Pioneer;
L1 = 0.455;  % distancia entre o engate e centro das rodas do primeiro trailer;
L10 = 0.28;  % distancia entre o centro das rodas do primeiro trailer e o engate do segundo trailer;
L2 = 0.455;  % distancia entre o engate e centro das rodas do segundo trailer;

%%% dimensoes trailers simulacao
ret_lg = 0.415;
ret_lp = 0.285;



while toc(t) < T_MAX
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        
         %%% ------------ Trajetoria desejada ----------
         
         %%%% circulo
        Xd_trajetoria([1 2]) = [-rX*sin(ww*t_atual + phase) - (L0+L1+L2+L10+b); -rY*cos(ww*t_atual + phase)+rY];
        Vd_trajetoria([1 2]) = [-ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];

        %%%% lemniscatta
%         Xd_trajetoria([1 2]) = [-rX*sin(ww*t_atual + phase)-(L0+L1+L2+L10+b); -rY*sin(2*ww*t_atual + phase)];
%         Vd_trajetoria([1 2]) = [-ww*rX*cos(ww*t_atual + phase);-2*ww*rY*cos(2*ww*t_atual + phase)];

        %%%% -------------------------------------------
        
                %%% ---- Calculo posicao no mundo do trailer 1 ------
        x_t1 = X_robo(1) - L0*cos(X_robo(4)) - (L1)*cos(X_trailer1(4));
        y_t1 = X_robo(2) - L0*sin(X_robo(4)) - (L1)*sin(X_trailer1(4));

        %%% ---- Calculo posicao no mundo do trailer 2 para controle------               
        x_t2 = x_t1 - L10*cos(X_trailer1(4)) - (L2+b)*cos(X_trailer2(4));
        y_t2 = y_t1 - L10*sin(X_trailer1(4)) - (L2+b)*sin(X_trailer2(4));

        %%% ---- Calculo posicao do centro no mundo do trailer 2 para plot ------  
        x_t2_c = x_t1 - L10*cos(X_trailer1(4)) - (L2)*cos(X_trailer2_aux(4));
        y_t2_c = y_t1 - L10*sin(X_trailer1(4)) - (L2)*sin(X_trailer2_aux(4));

        X_trailer1([1 2]) = ([x_t1 y_t1]);  %posicoes do trailer 1 no espaço (centro)
        X_trailer2([1 2]) = ([x_t2 y_t2]);  %posicoes do trailer 2 no espaço  (ponto de controle atras do ultimo trailer)
        X_trailer2_aux([1 2]) = ([x_t2_c;y_t2_c]);  %%% centro do trailer 2 (plotar)
%         X_trailer2_aux([1 2]) = (X_trailer2([1 2]) + [b*cos(X_trailer2(4));b*sin(X_trailer2(4))]);

        %%% calculo dos erros de posicao       
        Xtil_robo = Xd_trajetoria - X_robo; %erros de posicoes do robo no espaço (centro)
        Xtil_trailer1 = Xd_trajetoria - X_trailer1;  %erros de posicoes do trailer 1 no espaço (centro)
        Xtil_trailer2 = Xd_trajetoria - X_trailer2;  %erros de posicoes do trailer 2 no espaço (ponto de controle atras do ultimo trailer)
        
        %%% Velocidades - lei de controle
        v = Vd_trajetoria([1 2]) + K_1*tanh(K_2*Xtil_trailer2([1 2]));
        vx = v([1]);
        vy = v([2]);
        v_fi = (-vx/(b*cos(alpha)))*sin(X_trailer2([4])) +  (vy/(b*cos(alpha)))*cos(X_trailer2([4]));
        
        velocidade([1 2 4]) = [vx vy v_fi];
        %% Cinemática
        % Matriz de cinemática estendida do trailer 2
        K_inv2 = [ cos(X_trailer2([4])), sin(X_trailer2([4])), b*sin(alpha); ...
            -sin(X_trailer2([4])), cos(X_trailer2([4])), -b*cos(alpha); ...
            0, 0, 1];
                      
        U_trailer2 = K_inv2*velocidade([1 2 4]);
        U_trailer2(2) = 0;

        theta_1 = X_robo(4)-X_trailer1(4);
        theta_2 = X_trailer1(4)-X_trailer2(4);
         
        v1 = U_trailer2(1)*cos(theta_2) + (U_trailer2(3))*(L2)*sin(theta_2);
        w1 = U_trailer2(1)*sin(theta_2)/L10 - (U_trailer2(3))*(L2)*cos(theta_2)/L10;
        
        U_trailer1([1 2 3]) = [v1 0 w1]; 
                
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
        
        % Cinematica direta com 2 trailers
        K_inv_robo = [ cos(X_robo([4])), sin(X_robo([4])), 0; ...
            -sin(X_robo([4])), cos(X_robo([4])), 0; ...
            0, 0, 1];
        
        v1 = U_robo(1)*cos(theta_1) + (U_robo(3))*(L0)*sin(theta_1);
        w1 = U_robo(1)*sin(theta_1)/L1 - (U_robo(3))*(L0)*cos(theta_1)/L1;
        U_trailer1([1 2 3]) = [v1 0 w1];
        
             
        K_inv1 = [ cos(X_trailer1([4])), sin(X_trailer1([4])), 0; ...
            -sin(X_trailer1([4])), cos(X_trailer1([4])), 0; ...
            0, 0, 1];
        
        v2 = U_trailer1(1)*cos(theta_2) + (U_trailer1(3))*(L10)*sin(theta_2);
        w2 = U_trailer1(1)*sin(theta_2)/L2 - (U_trailer1(3))*(L10)*cos(theta_2)/L2;
        U_trailer2([1 2 3]) = [v2 0 w2];
        
        %%%% Matriz para plotar ponto centro trailer2
        K_inv2_aux = [ cos(X_trailer2_aux([4])), sin(X_trailer2_aux([4])), 0; ...
            -sin(X_trailer2_aux([4])), cos(X_trailer2_aux([4])), 0; ...
            0, 0, 1];
        
        %% Simulação
        % Velocidade do robô na referencia do mundo
        dX_robo([1 2 4]) = K_inv_robo\U_robo([1 2 3]);
        dX_trailer1([1 2 4]) = K_inv1\U_trailer1([1 2 3]);
        dX_trailer2([1 2 4]) = K_inv2\U_trailer2([1 2 3]);
        dX_trailer2_aux([1 2 4]) = K_inv2_aux\U_trailer2([1 2 3]);
        dX_robo(3) = 0;
        dX_trailer1(3) = 0;
        dX_trailer2(3) = 0;
        dX_trailer2_aux(3) = 0;
        
        % Cálculo de posição na referência do mundo do robo
        X_robo = X_robo + dX_robo*toc(t_sim);
        
        % Cálculo de angulos na referência do mundo dos trailers
        X_trailer1(4) = X_trailer1(4) + dX_trailer1(4)*toc(t_sim);
        X_trailer2(4) = X_trailer2(4) + dX_trailer2(4)*toc(t_sim);
        X_trailer2_aux(4) = X_trailer2_aux(4) + dX_trailer2_aux(4)*toc(t_sim);
        t_sim = tic;
              
        % Criacao de historico das variaveis
        t_hist = [t_hist toc(t)];
        Xd_trajetoria_hist = [Xd_trajetoria_hist Xd_trajetoria(1:2)];
        X_robo_hist = [X_robo_hist X_robo];
        X_trailer1_hist = [X_trailer1_hist X_trailer1];
        X_trailer2_hist = [X_trailer2_hist X_trailer2];
        X_trailer2_aux_hist = [X_trailer2_aux_hist X_trailer2_aux];
        dX_robo_hist = [dX_robo_hist dX_robo];
        dX_trailer1_hist = [dX_trailer1_hist dX_trailer1];
        dX_trailer2_hist = [dX_trailer2_hist dX_trailer2];
        Xtil_robo_hist =[Xtil_robo_hist Xtil_robo];
        Xtil_trailer1_hist = [Xtil_trailer1_hist Xtil_trailer1];
        Xtil_trailer2_hist = [Xtil_trailer2_hist Xtil_trailer2];
        theta_1_hist =[theta_1_hist theta_1];
        theta_2_hist = [theta_2_hist theta_2];
        
       
          
        %%               
        Corpo1 = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);
        Corpo_frente = X_robo(1:2) + [(raio+0.15)*cos(X_robo(4));(raio+0.15)*sin(X_robo(4))];

        Corpo2 = [cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer1(1:2);
        haste_L0 = L0*[cos(X_robo(4)) -sin(X_robo(4));sin(X_robo(4)) cos(X_robo(4))]*[-1;0] + X_robo(1:2);
        haste_L1 = (L1)*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[1;0] + X_trailer1(1:2);

        Corpo3 = [cos(X_trailer2(4)) -sin(X_trailer2(4));sin(X_trailer2(4)) cos(X_trailer2(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer2_aux(1:2);
        haste_L10 = L10*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-1;0] + X_trailer1(1:2);
        haste_L2 = (L2)*[cos(X_trailer2(4)) -sin(X_trailer2(4));sin(X_trailer2(4)) cos(X_trailer2(4))]*[1;0] + X_trailer2_aux(1:2);
        

        plot(Xd_trajetoria(1),Xd_trajetoria(2),'bh')
        hold on
        grid on
        plot(Xd_trajetoria_hist(1,:),Xd_trajetoria_hist(2,:),'b-')
        plot(X_robo_hist(1,:),X_robo_hist(2,:),'k','LineWidth',2)
        plot(Corpo1(1,:),Corpo1(2,:),'k','LineWidth',2)
        plot([X_robo(1) Corpo_frente(1)],[X_robo(2) Corpo_frente(2)],'k','LineWidth',2)
        plot(X_trailer1_hist(1),X_trailer1_hist(2),'k','LineWidth',2)
        plot(Corpo2(1,:),Corpo2(2,:),'k','LineWidth',2)
        plot([X_robo(1) haste_L0(1)],[X_robo(2) haste_L0(2)],'k','LineWidth',2)
        plot([haste_L0(1)],[haste_L0(2)],'ko')
        plot([X_trailer1(1) haste_L1(1)],[X_trailer1(2) haste_L1(2)],'k','LineWidth',2)
        plot(X_trailer2_hist(1,:),X_trailer2_hist(2,:),'r--','LineWidth',2)
        plot(Corpo3(1,:),Corpo3(2,:),'k','LineWidth',2)
        plot([X_trailer1(1) haste_L10(1)],[X_trailer1(2) haste_L10(2)],'k','LineWidth',2)
        plot([haste_L10(1)],[haste_L10(2)],'ko')
        plot([X_trailer2_aux(1) haste_L2(1)],[X_trailer2_aux(2) haste_L2(2)],'k','LineWidth',2)
        axis([-4 0.75 -0.5 4.5]) %%% circulo
%         axis([-10 7 -6 6]) %%% lemniscata
        hold off
        pause(0.001);
        drawnow



        
        
%         hold on
%         grid on
%         plot(t_hist(1,:),Xtil_trailer2_hist(1,:))
%         plot(t_hist(1,:),Xtil_trailer2_hist(2,:))
%         legend('erro_x','erro_y')
%         axis([0 T_MAX -0.5 0.5])
%         hold off
% 
%         hold on
%         grid on
%         plot(t_hist(1,:),theta_1_hist(1,:)*180/pi)
%         plot(t_hist(1,:),theta_2_hist(1,:)*180/pi)
%         legend('\theta_1','\theta_2')
%         axis([0 T_MAX -1 50])
%         hold off

           
        

    end
    
end
