clear all
close all
clc


%% Variáveis do Robô
% Pioneer
X1 = zeros(4,1); % Posição do Robô 1
dX1 = zeros(4,1); % Velocidade do Robô 1
ddX1_b = zeros(4,1); % Aceleração do Robô 1
Xd1 = zeros(4,1); % Posição Desejada do Robô 1
dXd1 = zeros(4,1); % Velocidade Desejada do Robô 1
U1 = zeros(4,1); % Comandos do Robô 1
% X1 = [0.3 0.1 0 0]';
Xd14a = 0;
dXd14a = 0;

% Drone
X2 = zeros(4,1); % Posição do Robô 1
dX2 = zeros(4,1); % Velocidade do Robô 1
ddX2_b = zeros(4,1); % Aceleração do Robô 1
Xd2 = zeros(4,1); % Posição Desejada do Robô 1
dXd2 = zeros(4,1); % Velocidade Desejada do Robô 1
U2 = zeros(4,1); % Comandos do Robô 1
X2 = [-2 1 1 -pi/4]';
Xd24a = 0;
dXd24a = 0;

% Controlador
Xtil = zeros(4,1);
Xr1 = zeros(4,1);

% Formação
Qd = zeros(6,1);
dQd = zeros(6,1);
Q = zeros(6,1);
Qtil = zeros(6,1);
dQr = zeros(6,1);

% Cinemática estendida do pioneer
par_a = 0.15;
alpha = pi/2;
sat_v = 0.75;
sat_w = 100*pi/180;

% Variaveis de corpo
raio = 0.15;
raio_obs = 0.5;
circ = 0:0.01:2*pi;
% Corpo = [raio*cos(circ);raio*sin(circ)] + X1(1:2);
% Corpo_frente = X1(1:2) + [raio*cos(X1(4));raio*sin(X1(4))];

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_c = tic; % Temporizador de controle
t_sim = tic; % Temporizador de simulação
T_c = 0.1; % Tempo de controle
t_der = tic;
t_max = 60;

% Variaveis de Ganho
K_1 = diag([1 1 1 1 1 1]);
K_2 = diag([1 1 1 1 1 1]);
K1_ori = 1;
K2_ori = 1;

% Variáveis do Caminho
RaioX = 2.5;
RaioY = 1.5;
CentroX = 0;
CentroY = 0;
inc = 0.1; % Resolução do Caminho
inc_desvio = 0.1;
nCaminho = 360; % Número de pontos total no caminho
s = 0:inc:nCaminho; % Abcissa curvilinea
x = -RaioX*sin(pi*s/180) + CentroX;
y = RaioY*sin(2*pi*s/180) + CentroY;
% x = s/60 - 2;
% y = 0*ones(1,length(s));
z = 0*ones(1,length(s));
C_normal = [x; y; z];
dist_fim = 0.2; % Distância ao ponto final onde considero que terminei a navegação
dist_final = 1000;
tol_caminho = 0.2; % Tolerância na qual considero o robô sobre o caminho
Ve = 0.3; % Velocidade desejada no caminho

% Variáveis dos obstáculos
Obstaculos = [2.0474 -0.9137 0; -1.0194 -0.664 0]';
% Obstaculos = [2.0474 -0.9137 0; -1.0194 -0.664 0;-2.5 0 0;2 1.8 0]';
% Obstaculos = [2.4474 -1.2537 0; -0.8 -1.204 0;-2.2 0 0;2 1 0]';
% Obstaculos = [2.4474 -1.2537 0; -0.8 -1.204 0;-2.2 0 0;2 -0.5 0]';
% Obstaculos = [2.4474 -1.2537 0; -2.6 -0.8 0;-2.7 0 0;2.2 -0.5 0]';
% Obstaculos = [50 50 0]';
% dist_detecta_obs = 1.2;
% dist_min_obs = 0.8;
dist_detecta_obs = 1.2;
dist_min_obs = 0.8;
flag_desvio = 0;
flag_desvio_acabando = 0;
flag_mudanca_obstaculo = 0;
obs_ativo_ind = 1;
obs_ativo_ind_antigo = 1;
[dist, ind] = calcula_ponto_proximo(C_normal(1:2,:),X1(1:2));

% Variáveis de curvatura
lim_curvatura = 0.5;
K_curv = 1;

% Variáveis de histórico
t_hist = [];
X1_hist = [];
dX1_hist = [];
Xd1_hist = [];
U1_hist = [];

X2_hist = [];
dX2_hist = [];
Xd2_hist = [];
U2_hist = [];

Q_hist = [];
Qd_hist = [];
Qtil_hist = [];

histerese_tempo = 0.01;
t_histerese = tic;

%Video
indVideo = 1;

ind = 1;
% C = C_normal;
figure
% try
% while dist_final > dist_fim
while t_max > toc(t)
    if toc(t_c) > T_c
        t_c = tic;
        
        %% Definidor de Caminho
        
        % Define o obstáculo ativo no momento. Há apenas um obstáculo ativo
        % e este deve estar com uma distancia menor que um limiar mínimo
        obs_ativo_dist = dist_detecta_obs;
        for i = 1:size(Obstaculos,2)
            obs_ativo_temp =  Obstaculos(:,i);
            obs_ativo_dist_temp = norm(X1(1:2) - obs_ativo_temp(1:2));
            if obs_ativo_dist_temp < obs_ativo_dist
                obs_ativo = obs_ativo_temp;
                obs_ativo_dist = obs_ativo_dist_temp;
                obs_ativo_ind = i;
            end
        end
        
        if obs_ativo_ind_antigo ~= obs_ativo_ind && toc(t_histerese) > histerese_tempo
            disp('mudou obstaculo')
            t_histerese = tic;
            flag_mudanca_obstaculo = 1;
            obs_ativo_antigo = Obstaculos(:,obs_ativo_ind_antigo);
        end
        obs_ativo_ind_antigo = obs_ativo_ind;
        
        if flag_mudanca_obstaculo == 1 && flag_desvio == 1
            
            [ind_desvio_inicio,flag_ameaca] = verifica_proximidade(C,obs_ativo,dist_min_obs,ind);
            
            if flag_ameaca == 1
                [C_desvio,ind_caminho_obs_final] = gera_caminho_desvio_mudanca_TH(X1,C_normal,obs_ativo,dist_min_obs,inc_desvio,ind,obs_ativo_antigo);
                
                flag_desvio = 1;
                flag_desvio_acabando = 1;
                flag_mudanca_obstaculo = 0;
            end
        end
        
        % Esta condição é feita apenas uma vez no início de cada manobra de
        % desvio. Ela vai criar o caminho novo que será seguido
        if (obs_ativo_dist < dist_detecta_obs && flag_desvio == 0 && flag_desvio_acabando == 0)
            
            [ind_desvio_inicio,flag_ameaca] = verifica_proximidade(C,obs_ativo,dist_min_obs,ind);
            
            if flag_ameaca == 1
                [C_desvio,ind_caminho_obs_final] = gera_caminho_desvio_TH(C_normal,obs_ativo,dist_min_obs,inc_desvio,ind);
                
                flag_desvio = 1;
                flag_desvio_acabando = 1;
                flag_mudanca_obstaculo = 0;
            end
        end
        
        if flag_desvio == 1
            if norm(X1(1:2)-C_normal(1:2,ind_caminho_obs_final)) < tol_caminho% || ind > size(C_desvio,2)-50
                flag_desvio = 0;
            end
        end
        
        if flag_desvio == 0
            C = C_normal;
        else
            C = C_desvio;
        end
        
        if flag_desvio == 0 && flag_desvio_acabando == 1
            if norm(X1(1:2)-obs_ativo(1:2)) > dist_detecta_obs || flag_mudanca_obstaculo == 1
                flag_desvio_acabando = 0;
            end
        end
        %% Controlador Caminho
        % Velocidade desejada atual
        Vd = Ve;
        if abs(U1(2)) > lim_curvatura
            Vd = Vd/(1+K_curv*abs(U1(2)));
        end
        
        % Distancia do robô para o final do caminho
        dist_final = norm(C_normal(:,end) - X1(1:3));
        
        % Calcula o ponto do caminho mais próximo do robô
        [dist, ind] = calcula_ponto_proximo(C(1:2,:),X1(1:2));
        
        % Define o ponto desejado
        Qd = [C(1:3,ind)' 1.5 X1(4) pi/2]';
        
        %         if toc(t) < 10
        %             Qd = [C(1:3,ind)' 1.5 X1(4) pi/2]';
        %         else
        %             Qd = [C(1:3,ind)' 0.8 X1(4)+pi (80*pi/180)]';
        %             disp('nova prioridade')
        %         end
        
        % Calcula o angulo do vetor tangente ao caminho
        %             theta_caminho = atan2(C(2,ind+1*sign(Vd)) - C(2,ind),C(1,ind+1*sign(Vd)) - C(1,ind));
        if ind == length(C)
            theta_caminho = atan2(C(2,ind) - C(2,1),C(1,ind) - C(1,1));
        else
            theta_caminho = atan2(C(2,ind+1*sign(Vd)) - C(2,ind),C(1,ind+1*sign(Vd)) - C(1,ind));
        end
        % Calcula as projeções nos eixos x e y do vetor velocidade tangente
        % ao caminho
        Vx = abs(Vd)*cos(theta_caminho);
        Vy = abs(Vd)*sin(theta_caminho);
        
        % Define a velocidade desejada no referencial do mundo. Isto é
        % dividido nos casos onde o robô esta fora do caminho e onde ele
        % está sobre o caminho
        if dist > tol_caminho
            dQd = [0 0 0 0 dX1(4) 0]';
        else
            dQd = [Vx Vy 0 0 dX1(4) 0]';
        end
        
        % Formação
        rho = norm(X2(1:3) - X1(1:3));
        alpha = atan2(X2(2) - X1(2),X2(1) - X1(1));
        beta = atan2(X2(3) - X1(3),norm(X2(1:2) - X1(1:2)));
        
        Q = [X1(1) X1(2) X1(3) rho alpha beta]';
        
        % Calcula o erro de posição
        Qtil = Qd - Q;
        
        if abs(Qtil(5)) > pi
            Qtil(5) = Qtil(5) - 2*pi*sign(Qtil(5));
        end
        
        % Lei de Controle
        dQr = dQd + K_1*tanh(K_2*Qtil);
        
        %% Jacobiano
        
        x1 = X1(1);
        y1 = X1(2);
        z1 = X1(3);
        x2 = X2(1);
        y2 = X2(2);
        z2 = X2(3);
        
        % Matriz Jacobiana Direta
        % Linha 4 da Matriz Jacobiana
        J41 = (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
        J42 = (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
        J43 = (z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
        J44 = -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
        J45 = -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
        J46 = -(z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
        
        % Linha 5 da Matriz Jacobiana
        J51 = -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2);
        J52 =  (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2);
        J53 =  0;
        J54 =  (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2);
        J55 = -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2);
        J56 =  0;
        
        % Linha 6 da Matriz Jacobiana
        J61 =  ((2*x1 - 2*x2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2));
        J62 =  ((2*y1 - 2*y2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2));
        J63 = -((x1 - x2)^2 + (y1 - y2)^2)^(1/2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
        J64 = -((2*x1 - 2*x2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2));
        J65 = -((2*y1 - 2*y2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2));
        J66 =  ((x1 - x2)^2 + (y1 - y2)^2)^(1/2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
        
        Jacob = [ 1  ,  0  , 0  , 0  , 0  , 0 ;...
            0  ,  1  , 0  , 0  , 0  , 0 ;...
            0  ,  0  , 1  , 0  , 0  , 0 ;...
            J41, J42, J43, J44, J45, J46;...
            J51, J52, J53, J54, J55, J56;...
            J61, J62, J63, J64, J65, J66 ];
        
        % Split Jacobian in Two Tasks
        Jp = Jacob(1:3,:);
        Js = Jacob(4:6,:);
        
        %         if toc(t) < 10
        dXr = Jacob\dQr;
        %         else
        %             dXr = pinv(Js)*dQr(4:6) + (eye(6) - pinv(Js)*Js)*pinv(Jp)*dQr(1:3);
        %         end
        % Controlador Orientação Bebop
        Otil = X1(4) - X2(4);
        if abs(Otil) > pi
            if Otil > 0
                Otil = -2*pi + Otil;
            else
                Otil =  2*pi + Otil;
            end
        end
        
        dBref_O = dX1(4) + K1_ori*tanh(K2_ori*Otil);
        
        
        %% Cinemática
        % Matriz de cinemática estendida
        %             K_inv = [cos(X1(4)) sin(X1(4)) par_a*sin(alpha);
        %                 -sin(X1(4)) cos(X1(4)) -par_a*cos(alpha);
        %                 0           0           1     ];
        
        % Matriz de cinemática Normal
        K = [cos(X1(4)) -par_a*sin(X1(4));
            sin(X1(4)) par_a*cos(X1(4))];
        
        K_d = [cos(X2(4)) -sin(X2(4)) 0 0;
            sin(X2(4)) cos(X2(4))     0 0;
            0           0             1 0;
            0           0             0 1];
        
        U1 = K\dXr([1 2]);
        U2 = K_d\[dXr(4:6); dBref_O];
        
        %% Saturação
        if abs(U1(1)) > sat_v
            U1(1) = sat_v*sign(U1(1));
        end
        
        if abs(U1(2)) > sat_w
            U1(2) = sat_w*sign(U1(2));
        end
        
        %% Saturação
        if abs(U2(1)) > sat_v
            U2(1) = sat_v*sign(U2(1));
        end
        
        if abs(U2(2)) > sat_w
            U2(2) = sat_w*sign(U2(2));
        end
        
        
        %% Simulação
        
        % Velocidade do robô na referencia do mundo
        dX1([1 2]) = K*U1(1:2);
        dX1(3) = 0;
        dX1(4) = U1(2);
        
        dX2 = K_d*U2;
        
        % Cálculo de posição na referência do mundo
        X1 = X1 + dX1*toc(t_sim);
        X2 = X2 + dX2*toc(t_sim);
        t_sim = tic;
        %% Figura
        t_hist = [t_hist toc(t)];
        X1_hist = [X1_hist X1];
        dX1_hist = [dX1_hist dX1];
        Xd1_hist = [Xd1_hist Xd1];
        U1_hist = [U1_hist U1];
        X2_hist = [X2_hist X2];
        dX2_hist = [dX2_hist dX2];
        Xd2_hist = [Xd2_hist Xd2];
        U2_hist = [U2_hist U2];
        Q_hist = [Q_hist Q];
        Qd_hist = [Qd_hist Qd];
        Qtil_hist = [Qtil_hist Qtil];
        
        Corpo = [raio*cos(circ);raio*sin(circ);0*sin(circ)] + X1(1:3) - [par_a*cos(X1(4)); par_a*sin(X1(4));0];
        Corpo_frente = X1(1:3) + [(raio+1)*cos(X1(4));(raio+1)*sin(X1(4));0];
        Drone_frente = X2(1:3) + [(raio+1)*cos(X2(4));(raio+1)*sin(X2(4));0];
        Corpo_obs_1 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,1);
        Corpo_obs_2 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,2);
%         Corpo_obs_3 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,3);
%         Corpo_obs_4 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,4);
        
        plot3(X1(1),X1(2),X1(3),'*')
        hold on
        grid on
        plot3(X2(1),X2(2),X2(3),'*')
        plot3(X1_hist(1,:),X1_hist(2,:),X1_hist(3,:),'-*','MarkerIndices',1:10:length(X2_hist))
        plot3(X2_hist(1,:),X2_hist(2,:),X2_hist(3,:),'-o','MarkerIndices',1:10:length(X2_hist))
        plot3(C_normal(1,:),C_normal(2,:),C_normal(3,:),'--')
        plot3(Corpo(1,:),Corpo(2,:),Corpo(3,:))
        plot(Corpo_obs_1(1,:),Corpo_obs_1(2,:),'r')
        plot(Corpo_obs_2(1,:),Corpo_obs_2(2,:),'r')
%         plot(Corpo_obs_3(1,:),Corpo_obs_3(2,:))
%         plot(Corpo_obs_4(1,:),Corpo_obs_4(2,:))
        plot3([X1(1) Corpo_frente(1)],[X1(2) Corpo_frente(2)],[X1(3) Corpo_frente(3)])
        plot3([X2(1) Drone_frente(1)],[X2(2) Drone_frente(2)],[X2(3) Drone_frente(3)])
        %         plot(xd_p(1),xd_p(2),'*')
        plot3(Obstaculos(1,:),Obstaculos(2,:),Obstaculos(3,:),'*')
        try
            plot3(C_desvio(1,:),C_desvio(2,:),C_desvio(3,:),'g')
        end
        %         axis([0 6 -2.5 2.5])
        axis([-3 3 -2 2 0 10])
%                 view(9,67)
        view(0,90)
        hold off
        
        Video(indVideo) = getframe(gcf);
        indVideo = indVideo + 1;
        
        drawnow
        
    end
end

% catch ME
%
%     disp('Bebop Landing through Try/Catch Loop Command');
%     P1.rCmdStop;
%
% end


% Figuras
% X1til_hist = Xd1_hist - X1_hist;

figure
subplot(3,2,1)
plot(t_hist,Qtil_hist(1,:))
grid on
ylabel('Erro X')
subplot(3,2,3)
plot(t_hist,Qtil_hist(2,:))
grid on
ylabel('Erro Y')
subplot(3,2,5)
plot(t_hist,Qtil_hist(3,:))
grid on
xlabel('Tempo')
ylabel('Erro Z')
subplot(3,2,2)
plot(t_hist,Qtil_hist(4,:))
grid on
ylabel('Erro Rho')
subplot(3,2,4)
plot(t_hist,Qtil_hist(5,:))
grid on
ylabel('Erro Alpha')
subplot(3,2,6)
plot(t_hist,Qtil_hist(6,:))
grid on
xlabel('Tempo')
ylabel('Erro Beta')


figure
plot(t_hist,U1_hist([1 2],:))
grid on
legend('Comandos V','Comandos W')

figure
plot(t_hist,U2_hist([1 2 3],:))
grid on
legend('Comandos X','Comandos Y','Comandos Z')

path = [pwd '\Dados\'];
filename = ['Desvio_Formacao_Simulacao' '_Data_' datestr(now,30) '.mat'];
fullname = [path filename];
% save(fullname,'t_hist','X1','X1_hist','dX1_hist','Xd1_hist',...
%     'U1_hist','X2','X2_hist','dX2_hist','Xd2_hist','U2_hist','Q_hist','Qd_hist',...
%     'Qtil_hist','Obstaculos','Corpo','C_normal','dist_detecta_obs','dist_min_obs','raio_obs')

% create the video writer with 1 fps
%   writerObj = VideoWriter('Desvio_Caminho_CaminhoImpossivel_4obs.mp4','MPEG-4');
%   writerObj.FrameRate = 10;
%   % set the seconds per image
%
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(Video)
%     % convert the image to a frame
%     frame = Video(i) ;
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);