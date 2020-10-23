clear all
close all
clc

% Variáveis do Robô
X_robo = zeros(6,1); % posicao atual do robo
X_trailer1 = zeros(6,1); % posicao atual do trailer1
X_trailer2 = zeros(6,1); % posicao atual do trailer2
X_trailer2_centro = zeros(6,1); % posicao atual do trailer2
Xd_trailer2 = zeros(6,1); % posicao atual do trailer1
Vd_trailer2 = zeros(6,1); % velocidades desejadas do caminho
v = zeros(4,1); % Velocidades controle do Robô
velocidade = zeros(4,1); % Velocidades controle do Robô
U_robo = zeros(6,1); % Comandos do Robô
dX_robo = zeros(6,1); % Variacoes posicoes do robo entre interacoes
U_trailer1 = zeros(6,1); % Comandos do trailer1
dX_trailer1 = zeros(6,1);% Variacoes posicoes do trailer1 entre interacoes
U_trailer2 = zeros(6,1); % Comandos do trailer2
dX_trailer2 = zeros(6,1);% Variacoes posicoes do trailer2 entre interacoes
dX_trailer2_centro = zeros(6,1);
Xtil_trailer2 = zeros(6,1); % Erros trailer 2
dX_robo1_anterior = 0;
dX_robo4_anterior = 0;
U1_robo_anterior = 0;
U3_robo_anterior = 0;
K_vel = zeros(5,2);
U_robo_din = zeros(5,1);
% X_robo = [2.5 0.7 0 pi/2 0 0]';
X_robo = [5 3.2 0 0 0 0]';
Xd_aux = 0;
Vd_aux = 0;

% Variáveis do Caminho
RaioX = 5;
RaioY = 3;
CentroX = 3;
CentroY = 0;
inc = 0.1; % Resolução do Caminho
inc_desvio = 0.1;
nCaminho = 350; % Número de pontos total no caminho
s = 0:inc:nCaminho; % Abcissa curvilinea
x = -RaioX*sin(pi*s/180) + CentroX;
% y = RaioY*sin(2*pi*s/180) + CentroY;
y = RaioY*cos(pi*s/180) + CentroY;
z = 0*ones(1,length(s));
C_normal = [x; y; z];
dist_fim = 0.1; % Distância ao ponto final onde considero que terminei a navegação
dist_final = 1000;
tol_caminho = 0.2; % Tolerância na qual considero o robô sobre o caminho
Ve = 0.35; % Velocidade desejada no caminho

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
theta_1 = 0.23025;
theta_2 = 0.22615;
theta_3 = 0.00028953;
theta_4 = 0.95282;
theta_5 = 0.021357;
theta_6 = 0.95282;

Theta_original = [theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];
% Theta_original = [1;1;1;1;1;1];

% Variáveis de histórico
t_hist = [];
Xd_trailer2_hist = [];
dX_robo_hist = [];
dX_trailer1_hist = [];
dX_trailer2_hist = [];
X_robo_hist = [];
X_trailer1_hist = [];
X_trailer2_hist = [];
X_trailer2_centro_hist = [];
Xtil_trailer2_hist = [];
theta_1_hist =[];
theta_2_hist = [];
Theta_est_hist = [];


% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_c = tic; % Temporizador de controle
t_sim = tic; % Temporizador de simulação
T_c = 0.1; % Tempo de controle
t_der = tic;
t_derivada = tic;

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

n = 1;  %%%% numero de iteracoes para controle adaptativo

%%% dimensoes trailers
L0 = 0.3;
L1 = 0.455;
L10 = 0.28;  % distancia entre o centro das rodas do primeiro trailer e o engate do segundo trailer;
L2 = 0.455;  % distancia entre o engate e centro das rodas do segundo trailer;

%%% dimensoes trailers simulacao
ret_lg = 0.415;
ret_lp = 0.285;

sat_angulo = pi/2;


while dist_final > dist_fim
    if toc(t_c) > T_c
        t_c = tic;
        k = k+1;
        
             %% Controlador Caminho
        % Velocidade desejada atual
        Vd = Ve;
        
        %%% ---- Calculo posicao no mundo do trailer 1 ------
        x_t1 = X_robo(1) - L0*cos(X_robo(4)) - (L1)*cos(X_trailer1(4));
        y_t1 = X_robo(2) - L0*sin(X_robo(4)) - (L1)*sin(X_trailer1(4));
        
        %%% ---- Calculo posicao no mundo do trailer 2 para controle------               
        x_t2 = x_t1 - L10*cos(X_trailer1(4)) - (L2+b)*cos(X_trailer2(4));
        y_t2 = y_t1 - L10*sin(X_trailer1(4)) - (L2+b)*sin(X_trailer2(4));

        %%% ---- Calculo posicao do centro no mundo do trailer 2 para plot ------  
        x_t2_c = x_t1 - L10*cos(X_trailer1(4)) - (L2)*cos(X_trailer2_centro(4));
        y_t2_c = y_t1 - L10*sin(X_trailer1(4)) - (L2)*sin(X_trailer2_centro(4));
        
        X_trailer1([1 2]) = ([x_t1 y_t1]);
        X_trailer2([1 2]) = ([x_t2 y_t2]);
        X_trailer2_centro([1 2]) = ([x_t2_c;y_t2_c]);  %%% plot
              
        % Distancia do robô para o final do caminho
        dist_final = norm(C_normal(:,end) - X_trailer2(1:3));
        
        % Calcula o ponto do caminho mais próximo do robô
        [dist, ind] = calcula_ponto_proximo(C_normal(1:2,:),X_trailer2(1:2));
        
        % Define o ponto desejado
        Xd_trailer2(1:2) = C_normal(1:2,ind);
        
        % Calcula o angulo do vetor tangente ao caminho
        theta_caminho = atan2(C_normal(2,ind+1*sign(Vd)) - C_normal(2,ind),C_normal(1,ind+1*sign(Vd)) - C_normal(1,ind));
        
        % Calcula as projeções nos eixos x e y do vetor velocidade tangente
        % ao caminho
        Vx = abs(Vd)*cos(theta_caminho);
        Vy = abs(Vd)*sin(theta_caminho);
        
        % Define a velocidade desejada no referencial do mundo. Isto é
        % dividido nos casos onde o robô esta fora do caminho e onde ele
        % está sobre o caminho

        Vd_trailer2(1) = Vx;
        Vd_trailer2(2) = Vy;
              
        %%%% calculo erro de posicao do trailer1
        Xtil_trailer2(1:3) = Xd_trailer2(1:3) - X_trailer2(1:3);  %erros de posicoes do tr  
        
        % Lei de Controle posicao
        v(1:3) = Vd_trailer2(1:3) + K_1*tanh(K_2*Xtil_trailer2(1:3));
        
        % Lei de Controle orientacao
        v_fi = (-v(1)/(b*cos(alpha)))*sin(X_trailer2([4])) +  (v(2)/(b*cos(alpha)))*cos(X_trailer2([4]));
        
        velocidade([1 2 4]) = [v(1) v(2) v_fi];
        
        K = [ cos(X_trailer2([4])), sin(X_trailer2([4])), b*sin(alpha); ...
            -sin(X_trailer2([4])), cos(X_trailer2([4])), -b*cos(alpha); ...
            0, 0, 1];
      
       %%%% sinal de controle cinematico
       U_trailer2 = K*velocidade([1 2 4]);
       U_trailer2(2) = 0;
       
       %%%% transmissao de velocidade inversa - do trailer 1 para o robo
       theta_1 = X_robo(4)-X_trailer1(4);
       theta_2 = X_trailer1(4)-X_trailer2(4);
       
       %Saturação angulo
        if abs(theta_1) > sat_angulo
            theta_1 = sat_angulo*sign(theta_1);
        end
        
        if abs(theta_2) > sat_angulo
            theta_2 = sat_angulo*sign(theta_2);
        end
        
       v1 = U_trailer2(1)*cos(theta_2) + (U_trailer2(3))*(L2)*sin(theta_2);
       w1 = U_trailer2(1)*sin(theta_2)/L10 - (U_trailer2(3))*(L2)*cos(theta_2)/L10;
        
       U_trailer1([1 2 3]) = [v1 0 w1]; 
        
       v0_c = U_trailer1(1)*cos(theta_1) + (U_trailer1(3))*(L1)*sin(theta_1);
       w0_c = U_trailer1(1)*sin(theta_1)/L0 - (U_trailer1(3))*(L1)*cos(theta_1)/L0;
                
       U_robo([1 2 3]) = [v0_c 0 w0_c];   %%%% sinal de controle cinematico para o robo1
       
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
        if k>=50*n
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
          if k<50*n
          A_din = [Theta_original(1) 0; 0 Theta_original(2)];   
          U_robo([1 3]) = A_din*[delta_1;delta_2] + G*Theta_original;
          disp('dinamico');
          end
          
          %%%%% a partir de k - comeca com controlador adaptativo
          if k>=50*n
          U_robo([1 3]) = A_est*[delta_1;delta_2] + G*Theta_est + G*Theta_til;
          disp('adaptativo');
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
        
        %%% transmissao velocidade direta do robo para o trailer1
        U_trailer1(1) = X_robo(5)*cos(theta_1) + (X_robo(6))*(L0)*sin(theta_1);
        U_trailer1(3) = X_robo(5)*sin(theta_1)/(L1) - (X_robo(6))*(L0)*cos(theta_1)/(L1);
        U_trailer1(2) = 0;
        
        K_est1 = [ cos(X_trailer1([4])), sin(X_trailer1([4])), 0; ...
                   -sin(X_trailer1([4])), cos(X_trailer1([4])), 0; ...
                    0, 0, 1];
                
        %%% transmissao velocidade direta do trailer1 para o trailer2
        U_trailer2(1) = U_trailer1(1)*cos(theta_2) + (U_trailer1(3))*(L10)*sin(theta_2);
        U_trailer2(3) = U_trailer1(1)*sin(theta_2)/(L2) - (U_trailer1(3))*(L10)*cos(theta_2)/(L2);
        U_trailer2(2) = 0;
        
        K_est2_centro = [ cos(X_trailer2_centro([4])), sin(X_trailer2_centro([4])), 0; ...
                        -sin(X_trailer2_centro([4])), cos(X_trailer2_centro([4])), 0; ...
                         0, 0, 1];
        
        %% Simulação
        
        % Velocidade do robô na referencia do mundo
        dX_robo([1 2 3 4 5 6]) = K_robo;
        dX_trailer1([1 2 4]) = K_est1\U_trailer1([1 2 3]);
        dX_trailer2([1 2 4]) = K\U_trailer2([1 2 3]);
        dX_trailer2_centro([1 2 4]) = K_est2_centro\U_trailer2([1 2 3]);
        dX_robo(3) = 0;
        dX_trailer1(3) = 0;
        dX_trailer2(3) = 0;
        dX_trailer2_centro(3) = 0;
        
        % Cálculo de posição na referência do mundo
        X_robo = X_robo + dX_robo*toc(t_sim);
        X_trailer1 = X_trailer1 + dX_trailer1*toc(t_sim);
        X_trailer2 = X_trailer2 + dX_trailer2*toc(t_sim);
        X_trailer2_centro = X_trailer2_centro + dX_trailer2_centro*toc(t_sim);
        t_sim = tic;
        
        %Saturação
        if abs(X_robo(5)) > sat_v
            X_robo(5) = sat_v*sign(X_robo(5));
        end
        
        if abs(X_robo(6)) > sat_w
            X_robo(6) = sat_w*sign(X_robo(6));
        end      
        
        
        t_hist = [t_hist toc(t)];
        Xd_trailer2_hist = [Xd_trailer2_hist Xd_trailer2(1:2)];
        dX_robo_hist = [dX_robo_hist dX_robo];
        dX_trailer1_hist = [dX_trailer1_hist dX_trailer1];
        dX_trailer2_hist = [dX_trailer2_hist dX_trailer2];
        X_robo_hist = [X_robo_hist X_robo];
        X_trailer1_hist = [X_trailer1_hist X_trailer1];
        X_trailer2_hist = [X_trailer2_hist X_trailer2];
        X_trailer2_centro_hist = [X_trailer2_centro_hist X_trailer2_centro];
        Xtil_trailer2_hist = [Xtil_trailer2_hist Xtil_trailer2];
        theta_1_hist =[theta_1_hist theta_1];
        theta_2_hist = [theta_2_hist theta_2];
        Theta_est_hist = [Theta_est_hist Theta_est];

        %%
        Corpo1 = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);
        Corpo_frente = X_robo(1:2) + [(raio+0.15)*cos(X_robo(4));(raio+0.15)*sin(X_robo(4))];
        
        Corpo2 = [cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer1(1:2);
        haste_L0 = L0*[cos(X_robo(4)) -sin(X_robo(4));sin(X_robo(4)) cos(X_robo(4))]*[-1;0] + X_robo(1:2);
        haste_L1 = L1*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[1;0] + X_trailer1(1:2);
       
        Corpo3 = [cos(X_trailer2(4)) -sin(X_trailer2(4));sin(X_trailer2(4)) cos(X_trailer2(4))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer2_centro(1:2);
        haste_L10 = L10*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-1;0] + X_trailer1(1:2);
        haste_L2 = L2*[cos(X_trailer2(4)) -sin(X_trailer2(4));sin(X_trailer2(4)) cos(X_trailer2(4))]*[1;0] + X_trailer2_centro(1:2);
        
        plot(X_trailer2(1,:),X_trailer2(2,:),'-')
        hold on
        grid on
        plot(C_normal(1,:),C_normal(2,:),'--')
        plot(Corpo1(1,:),Corpo1(2,:),'k','LineWidth',2)
        plot(X_trailer2_hist(1,:),X_trailer2_hist(2,:),'-')
        plot(Corpo2(1,:),Corpo2(2,:),'k','LineWidth',2)
        plot([X_robo(1) Corpo_frente(1)],[X_robo(2) Corpo_frente(2)],'k','LineWidth',2)
        plot([X_robo(1) haste_L0(1)],[X_robo(2) haste_L0(2)],'k','LineWidth',2)
        plot([X_trailer1(1) haste_L1(1)],[X_trailer1(2) haste_L1(2)],'k','LineWidth',2)
        plot([haste_L0(1)],[haste_L0(2)],'ko')
        plot(Corpo3(1,:),Corpo3(2,:),'k','LineWidth',2)
        plot([X_trailer1(1) haste_L10(1)],[X_trailer1(2) haste_L10(2)],'k','LineWidth',2)
        plot([haste_L10(1)],[haste_L10(2)],'ko')
        plot([X_trailer2_centro(1) haste_L2(1)],[X_trailer2_centro(2) haste_L2(2)],'k','LineWidth',2)
%         axis([0 6 -2.5 2.5])
        axis([-4.5 10.5 -4 4])
%         axis([-10 20 -10 20])
        hold off   
        
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
                
%         hold on
%         grid on
%         plot(t_hist(1,:),Xtil_trailer2_hist(1,:))
%         plot(t_hist(1,:),Xtil_trailer2_hist(2,:))
%         axis([0 20 -0.5 0.5])
%         hold off

        drawnow   
        
        
    end
    
end
