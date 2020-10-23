clear all
close all
clc

% Variáveis do Robô
X_robo = zeros(6,1); % posicao atual do robo
X_trailer1 = zeros(6,1); % posicao atual do trailer1
X_trailer1_centro = zeros(6,1); % posicao atual do trailer1
Xd_trailer1 = zeros(6,1); % posicao atual do trailer1
Vd_trailer1 = zeros(6,1); % velocidades desejadas do caminho
v = zeros(4,1); % Velocidades controle do Robô
velocidade = zeros(4,1); % Velocidades controle do Robô
U_robo = zeros(6,1); % Comandos do Robô
dX_robo = zeros(6,1); % Variacoes posicoes do robo entre interacoes
U_trailer1 = zeros(6,1); % Comandos do trailer1
dX_trailer1 = zeros(6,1);% Variacoes posicoes do trailer1 entre interacoes
dX_trailer1_centro = zeros(6,1);
Xtil_trailer1 = zeros(6,1); % Erros trailer 1
U1_robo_anterior = 0;
U3_robo_anterior = 0;
K_vel = zeros(5,2);
% X_robo = [2.5 0.7 0 pi/2 0 0]';
% X_robo = [3.8 0.5 0 0 0 0]';
itae_anterior = 0;
iae_anterior = 0;
itae_v_linear_anterior = 0;
iae_v_linear_anterior = 0;
itae_v_angular_anterior = 0;
iae_v_angular_anterior = 0;

% Variáveis do Caminho
RaioX = 2;
RaioY = 1.5;
CentroX = 3;
CentroY = 0;
inc = 0.1; % Resolução do Caminho
nCaminho = 350; % Número de pontos total no caminho
s = 5:inc:nCaminho; % Abcissa curvilinea
x = -RaioX*sin(pi*s/180) + CentroX;
y = RaioY*sin(2*pi*s/180) + CentroY;
% y = RaioY*cos(pi*s/180) + CentroY;
z = 0*ones(1,length(s));
C_normal = [x; y; z];
dist_fim = 0.1; % Distância ao ponto final onde considero que terminei a navegação
dist_final = 1000;
tol_caminho = 0.1; % Tolerância na qual considero o robô sobre o caminho
%%%%%%%% Velocidade limite - circulo=0.4 / lemniscata = 0.35 - dinamico vai menos que isso
Ve = 0.1; % Velocidade desejada no caminho

% Cinemática estendida do pioneer
alpha = pi;
sat_v = 0.75;
sat_w = 100*pi/180;
b = 0.2;

% Variaveis para plot corpo
raio = 0.15;
circ = 0:0.01:2*pi;

% Variaveis de Ganho
K_1 = 0.8*diag([1 1 1]);
K_2 = 0.8*diag([1 1 1]);
k_u = 4;
k_w = 4;

% thetas primeira identificacao
% theta_1 = 0.23025;
% theta_2 = 0.22615;
% theta_3 = 0.00028953;
% theta_4 = 0.95282;
% theta_5 = 0.021357;
% theta_6 = 0.95282;

%%% thetas artigo Filipe Martins
% theta_1 = 0.2604;
% theta_2 = 0.2509;
% theta_3 = -0.000499;
% theta_4 = 0.9965;
% theta_5 = 0.00263;
% theta_6 = 1.0768;

theta_1=0.23882;
theta_2=0.23938;
theta_3=0.0038418;
theta_4=0.9435;
theta_5=-0.007849;
theta_6=0.92249;

Theta_original = [theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];
% Theta_original = [1;1;1;1;1;1];

% Variáveis de histórico
t_hist = [];
Xd_trailer1_hist = [];
dX_robo_hist = [];
dX_trailer1_hist = [];
X_robo_hist = [];
X_trailer1_hist = [];
X_trailer1_centro_hist = [];
Xtil_trailer1_hist = [];
theta_1_hist =[];
Theta_est_hist = [];
sinal_controle_hist = [];
sinal_controle_trailer_hist = [];
erro_hist = [];
erro_v_linear_hist = [];
erro_v_angular_hist = [];
Vd_trailer1_hist = [];
V_desejada_caminho_hist = [];

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_c = tic; % Temporizador de controle
t_sim = tic; % Temporizador de simulação
T_c = 0.1; % Tempo de controle
t_der = tic;
t_derivada = tic;
tempo_erro = tic;
tempo_erro_v = tic;

%%% dimensoes trailers
L0 = 0.3;
L1 = 0.455;
%%% dimensoes trailers simulacao
ret_lg = 0.415;
ret_lp = 0.285;

limite_curva = 0.5;
K_curva = 1;

sat_angulo = pi/2;
U_robo(3) = 0;
k=0;

% f1 = figure;
% f2 = figure;
% f3 = figure;
% f4 = figure;
% f5 = figure;

%%% dentro circulo
% X_robo = [3.25 1.5 0 0 0 0]';
%%% fora circulo
% X_robo = [3.25 2.2 0 0 0 0]';
%%% favoravel circulo
% X_robo = [3.5 2 0 0 0 0]';
%%% desfavoravel circulo
% X_robo = [0.6 2 0 pi 0 0]';
% X_trailer1 = [0.6+L0+L1+b 2 0 pi 0 0]'; % posicao atual do trailer1
% X_trailer1_centro = [0.6+L0+L1 2 0 pi 0 0]'; % posicao atual do trailer1

%%% dentro lemniscata
% X_robo = [2.5 0.5 0 0 0 0]';
%%% fora lemniscata
X_robo = [2.75 1.75 0 0 0 0]';
%%% favoravel lemniscata
% X_robo = [2.75 1.5 0 0 0 0]';
%%% desfavoravel lemniscata
% X_robo = [0 1.5 0 pi 0 0]';
% X_trailer1 = [0+L0+L1+b 1.5 0 pi 0 0]'; % posicao atual do trailer1
% X_trailer1_centro = [0+L0+L1 1.5 0 pi 0 0]'; % posicao atual do trailer1

while dist_final > dist_fim
    if toc(t_c) > T_c
        t_c = tic;
        k = k+1;
        
             %% Controlador Caminho
        % Velocidade desejada atual
        Vd = Ve;
        
        if abs(U_trailer1(3)) > limite_curva
        Vd = Vd/(1+K_curva*abs(U_trailer1(3)));
        end
        
        %%% ---- Calculo posicao no mundo do trailer 1 ------
        x_t1 = X_robo(1) - L0*cos(X_robo(4)) - (L1+b)*cos(X_trailer1(4));
        y_t1 = X_robo(2) - L0*sin(X_robo(4)) - (L1+b)*sin(X_trailer1(4));
        
        x_t1_c = X_robo(1) - L0*cos(X_robo(4)) - (L1)*cos(X_trailer1(4));
        y_t1_c = X_robo(2) - L0*sin(X_robo(4)) - (L1)*sin(X_trailer1(4));
        
        X_trailer1([1 2]) = ([x_t1 y_t1]);
        X_trailer1_centro([1 2]) = ([x_t1_c;y_t1_c]);  %%% plot
              
        % Distancia do robô para o final do caminho
        dist_final = norm(C_normal(:,end) - X_trailer1(1:3));
        
        % Calcula o ponto do caminho mais próximo do robô
        [dist, ind] = calcula_ponto_proximo(C_normal(1:2,:),X_trailer1(1:2));
        
        % Define o ponto desejado
        Xd_trailer1(1:2) = C_normal(1:2,ind);
        
        % Calcula o angulo do vetor tangente ao caminho
        theta_caminho = atan2(C_normal(2,ind+1*sign(Vd)) - C_normal(2,ind),C_normal(1,ind+1*sign(Vd)) - C_normal(1,ind));
        
        % Calcula as projeções nos eixos x e y do vetor velocidade tangente
        % ao caminho
        Vx = abs(Vd)*cos(theta_caminho);
        Vy = abs(Vd)*sin(theta_caminho);
        
        % Define a velocidade desejada no referencial do mundo
        
        Vd_trailer1(1) = Vx;
        Vd_trailer1(2) = Vy;

        %%%% calculo erro de posicao do trailer1
        Xtil_trailer1(1:3) = Xd_trailer1(1:3) - X_trailer1(1:3);
        
        % Lei de Controle posicao
        v(1:3) = Vd_trailer1(1:3) + K_1*tanh(K_2*Xtil_trailer1(1:3));
        
        % Lei de Controle orientacao
        v_fi = (-v(1)/(b*cos(alpha)))*sin(X_trailer1([4])) + (v(2)/(b*cos(alpha)))*cos(X_trailer1([4]));
        
        velocidade([1 2 4]) = [v(1) v(2) v_fi];
        
        K = [cos(X_trailer1([4])), sin(X_trailer1([4])), b*sin(alpha); ...
            -sin(X_trailer1([4])), cos(X_trailer1([4])), -b*cos(alpha); ...
            0, 0, 1];
      
       %%%% sinal de controle cinematico
       U_trailer1 = K*velocidade([1 2 4]);
       U_trailer1(2) = 0;
              
       %%%% angulo relativo entre trailer e robo
       theta_1 = X_robo(4)-X_trailer1(4);
       
       %%%%%% transmissao de velocidade do trailer para o robo
       v0_c = U_trailer1(1)*cos(theta_1) + (U_trailer1(3))*(L1)*sin(theta_1);
       w0_c = U_trailer1(1)*sin(theta_1)/L0 - (U_trailer1(3))*(L1)*cos(theta_1)/L0;
                
       U_robo([1 2 3]) = [v0_c 0 w0_c];   %%%% sinal de controle cinematico para o robo1
       
       %%%% -------------   compensador dinâmico ---------------
       U1_robo_ref = (U_robo(1) - U1_robo_anterior)/toc(t_derivada);
       U3_robo_ref = (U_robo(3) - U3_robo_anterior)/toc(t_derivada);
       t_derivada = tic;
       U1_robo_anterior = U_robo(1);
       U3_robo_anterior = U_robo(3);
          
       G = [0 0 -X_robo(6)^2 X_robo(5) 0 0; ...
               0 0 0 0 X_robo(6)*X_robo(5) X_robo(6)];
          
       delta_1 = U1_robo_ref + k_u*(U_robo(1) - X_robo(5));
       delta_2 = U3_robo_ref + k_w*(U_robo(3) - X_robo(6));
          
       A_din = [Theta_original(1) 0; 0 Theta_original(2)];   
%        U_robo([1 3]) = A_din*[delta_1;delta_2] + G*Theta_original;
        
        %Saturação
        if abs(U_robo(1)) > sat_v
            U_robo(1) = sat_v*sign(U_robo(1));
        end
        
        if abs(U_robo(3)) > sat_w
            U_robo(3) = sat_w*sign(U_robo(3));
        end
        
%         if k>50 && k<100
%             U_robo([1 3]) = [0;0];
%         end
%         
%          if k>250 && k<300
%             U_robo([1 3]) = [0;0];
%          end
%          
%          k
        
        %%% Modelo Dinamico Pioneer
        K_din = [(Theta_original(3)/Theta_original(1))*X_robo(6)^2 - (Theta_original(4)/Theta_original(1))*X_robo(5); -(Theta_original(5)/Theta_original(2))*X_robo(6)*X_robo(5) - (Theta_original(6)/Theta_original(2))*X_robo(6)];
        K_vel = [1/Theta_original(1) 0; 0 1/Theta_original(2)];
        ddXd_robo = K_din+K_vel*U_robo([1 3]);
        
        K_direta = [X_robo(5)*cos(X_robo([4]))-b*X_robo(6)*sin(X_robo([4])); ...
                X_robo(5)*sin(X_robo([4]))+b*X_robo(6)*cos(X_robo([4])); ...
                                    0;
                                X_robo(6)];
        K_robo = vertcat(K_direta,ddXd_robo);
        
        %%% transmissao velocidade direta do robo para o trailer1
        U_trailer1(1) = X_robo(5)*cos(theta_1) + (X_robo(6))*(L0)*sin(theta_1);
        U_trailer1(3) = X_robo(5)*sin(theta_1)/(L1) - (X_robo(6))*(L0)*cos(theta_1)/(L1);
        U_trailer1(2) = 0;
        
        K_centro = [ cos(X_trailer1([4])), sin(X_trailer1([4])), 0; ...
            -sin(X_trailer1([4])), cos(X_trailer1([4])), 0; ...
            0, 0, 1];
        
        %% Simulação
        
        % Velocidade do robô na referencia do mundo
        dX_robo([1 2 3 4 5 6]) = K_robo;
        dX_trailer1([1 2 4]) = K\U_trailer1([1 2 3]);
        dX_trailer1_centro([1 2 4]) = K_centro\U_trailer1([1 2 3]);
        dX_trailer1_centro(3) = 0;
        dX_trailer1(3) = 0;
        dX_robo(3) = 0;
        
        % Cálculo de posição na referência do mundo
        X_robo = X_robo + dX_robo*toc(t_sim);
        X_trailer1 = X_trailer1 + dX_trailer1*toc(t_sim);
        X_trailer1_centro = X_trailer1_centro + dX_trailer1_centro*toc(t_sim);
        t_sim = tic;
        
            %Saturação
        if abs(X_robo(5)) > sat_v
            X_robo(5) = sat_v*sign(X_robo(5));
        end
        
        if abs(X_robo(6)) > sat_w
            X_robo(6) = sat_w*sign(X_robo(6));
        end
        
        if abs(theta_1) > 90*pi/180
        disp('jacknife');
        theta_1*180/pi
        break;
        end
        
      %%%%%%%%%%% ---------------------Calculo erro posicao ---------------------
      erro = sqrt((Xd_trailer1(1) - X_trailer1(1))^2 + (Xd_trailer1(2) - X_trailer1(2))^2);
      time_erro = toc(tempo_erro);

      itae = (time_erro)*erro;
      itae = itae + itae_anterior;
      itae_anterior = itae;

      iae = erro;
      iae = iae + iae_anterior;
      iae_anterior = iae;
      
      erros = [iae itae];
      %%%%% ------------------------------------------------
      
      %%%%%%%%%%% ---------------------Calculo erro velocidade ---------------------
      erro_v_linear = abs(-Vd - U_trailer1(1));
      erro_v_angular = abs(v_fi - U_trailer1(3));
      
      time_erro_v = toc(tempo_erro_v);

      itae_v_linear = (time_erro_v)*erro_v_linear;
      itae_v_linear = itae_v_linear + itae_v_linear_anterior;
      itae_v_linear_anterior = itae_v_linear;
      
      itae_v_angular = (time_erro_v)*erro_v_angular;
      itae_v_angular = itae_v_angular + itae_v_angular_anterior;
      itae_v_angular_anterior = itae_v_angular;

      iae_v_linear = erro_v_linear;
      iae_v_linear = iae_v_linear + iae_v_linear_anterior;
      iae_v_linear_anterior = iae_v_linear;
      
      iae_v_angular = erro_v_angular;
      iae_v_angular = iae_v_angular + iae_v_angular_anterior;
      iae_v_angular_anterior = iae_v_angular;
      
      erros_v_linear = [iae_v_linear itae_v_linear];
      erros_v_angular = [iae_v_angular itae_v_angular];
      
      %%%%% ------------------------------------------------
      
        t_hist = [t_hist toc(t)];
        Xd_trailer1_hist = [Xd_trailer1_hist Xd_trailer1(1:2)];
        dX_robo_hist = [dX_robo_hist dX_robo];
        dX_trailer1_hist = [dX_trailer1_hist dX_trailer1];
        X_robo_hist = [X_robo_hist X_robo];
        X_trailer1_hist = [X_trailer1_hist X_trailer1];
        X_trailer1_centro_hist = [X_trailer1_centro_hist X_trailer1_centro];
        Vd_trailer1_hist = [Vd_trailer1_hist vertcat(Vd_trailer1(1:2),v_fi)];
        Xtil_trailer1_hist = [Xtil_trailer1_hist Xtil_trailer1];
        theta_1_hist =[theta_1_hist theta_1];
        sinal_controle_hist = [sinal_controle_hist U_robo([1 3])];
        sinal_controle_trailer_hist = [sinal_controle_trailer_hist U_trailer1([1 3])];
        erro_hist = [erro_hist erros'];
        erro_v_linear_hist = [erro_v_linear_hist erros_v_linear'];
        erro_v_angular_hist = [erro_v_angular_hist erros_v_angular'];
        V_desejada_caminho_hist = [V_desejada_caminho_hist Vd];
        
        %%
        Corpo1 = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);
        Corpo_frente = X_robo(1:2) + [(raio+0.15)*cos(X_robo(4));(raio+0.15)*sin(X_robo(4))];
        
        Corpo2 = [cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer1_centro(1:2);
        haste_L0 = L0*[cos(X_robo(4)) -sin(X_robo(4));sin(X_robo(4)) cos(X_robo(4))]*[-1;0] + X_robo(1:2);
        haste_L1 = L1*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[1;0] + X_trailer1_centro(1:2);
       
        plot(X_trailer1(1,:),X_trailer1(2,:),'-')
        hold on
        grid on
        plot(C_normal(1,:),C_normal(2,:),'--')
        plot(Corpo1(1,:),Corpo1(2,:),'k','LineWidth',2)
        plot(X_trailer1_hist(1,:),X_trailer1_hist(2,:),'-')
        plot(Corpo2(1,:),Corpo2(2,:),'k','LineWidth',2)
        plot([X_robo(1) Corpo_frente(1)],[X_robo(2) Corpo_frente(2)],'k','LineWidth',2)
        plot([X_robo(1) haste_L0(1)],[X_robo(2) haste_L0(2)],'k','LineWidth',2)
        plot([X_trailer1_centro(1) haste_L1(1)],[X_trailer1_centro(2) haste_L1(2)],'k','LineWidth',2)
        plot([haste_L0(1)],[haste_L0(2)],'ko')
        axis([-2 5.5 -2.5 2.5])
%         axis([-4.5 10.5 -3.5 3.5])
%         axis([-10 20 -10 20])
        hold off
        
         drawnow  
        
        
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
                
        hold on
        grid on
        plot(t_hist(1,:),Xtil_trailer1_hist(1,:))
        plot(t_hist(1,:),Xtil_trailer1_hist(2,:))
        axis([0 20 -0.5 0.5])
        hold off
% 
%         hold on
%         grid on
%         plot(t_hist(1,:),theta_1_hist(1,:))
%         axis([0 35 -2 2])
%         hold off
% 

%         
%         hold on
%         grid on
%         plot(t_hist(1,:),sinal_controle_hist(1,:))
%         plot(t_hist(1,:),sinal_controle_hist(2,:))
%         axis([0 35 -2 2])
%         hold off
% %         
%         hold on
%         grid on
%         plot(t_hist(1,:),sinal_controle_trailer_hist(1,:))
%         plot(t_hist(1,:),sinal_controle_trailer_hist(2,:))
%         axis([0 35 -2 2])
%         hold off
% 
%         figure(1);
%         hold on
%         grid on
%         plot(t_hist(1,:),-V_desejada_caminho_hist(1,:))
%         plot(t_hist(1,:),sinal_controle_trailer_hist(1,:))
%         axis([0 50 -2 2])
%         legend('u_{desejado}','u_{real}')
%         hold off
%         
%         figure(2);
%         hold on
%         grid on
%         plot(t_hist(1,:),Vd_trailer1_hist(3,:))
%         plot(t_hist(1,:),sinal_controle_trailer_hist(2,:))
%         axis([0 50 -2 2])
%         legend('w_{desejado}','w_{real}')
%         hold off
%         
%         figure(3);
%         hold on
%         grid on
%         plot(t_hist(1,:),erro_hist(1,:))
%         plot(t_hist(1,:),erro_hist(2,:))
%         axis([0 50 0 700])
%         legend('iae','itae')
%         hold off
% % 
%         figure(4);
%         hold on
%         grid on
%         plot(t_hist(1,:),erro_v_linear_hist(1,:))
%         plot(t_hist(1,:),erro_v_linear_hist(2,:))
%         axis([0 50 0 1000])
%         legend('iae_{vlinear}','itae_{vlinear}')
%         hold off
% % %         
%         figure(5);
%         hold on
%         grid on
%         plot(t_hist(1,:),erro_v_angular_hist(1,:))
%         plot(t_hist(1,:),erro_v_angular_hist(2,:))
%         axis([0 50 0 5000])
%         legend('iae_{vangular}','itae_{vangular}')
%         hold off

Error = [erros;erros_v_linear;erros_v_angular]
%      
        
    end
    
end



