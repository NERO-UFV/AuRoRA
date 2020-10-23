clear; close all; clc;
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

%% Load Classes

%% Load Class
try
    P2 = Pioneer3DX(1);
    P3 = Pioneer3DX(2);
    
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


%% Variable initialization
data = [];

idP = 1;
idT1 = 2;
idT2 = 3;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%
rb = OPT.RigidBody;          % read optitrack data
P2 = getOptData(rb(idT1),P2);   % get pioneer data
P3 = getOptData(rb(idT2),P3);   % get pioneer data

%% Variáveis do Robô
X1 = zeros(4,1); % Posição do Robô 1
dX1 = zeros(4,1); % Velocidade do Robô 1
ddX1_b = zeros(4,1); % Aceleração do Robô 1
Xd1 = zeros(4,1); % Posição Desejada do Robô 1
dXd1 = zeros(4,1); % Velocidade Desejada do Robô 1
U1 = zeros(4,1); % Comandos do Robô 1
Xtil = zeros(4,1);
Xr1 = zeros(4,1);
X1 = [-2 0 0 0]';
% X1 = [-0.5 0.5 0 pi/2]';
Xd14a = 0;
dXd14a = 0;
% xd_p = [6 0]';

% Cinemática estendida do pioneer
par_a = 0.15;
alpha = pi/2;
sat_v = 0.75;
sat_w = 100*pi/180;

% Variaveis de corpo
raio = 0.15;
raio_obs = 0.8;
circ = 0:0.01:2*pi;
% Corpo = [raio*cos(circ);raio*sin(circ)] + X1(1:2);
% Corpo_frente = X1(1:2) + [raio*cos(X1(4));raio*sin(X1(4))];

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_c = tic; % Temporizador de controle
t_sim = tic; % Temporizador de simulação
T_c = 0.1; % Tempo de controle
t_der = tic;
t_max = 120;

% Variaveis de Ganho
K_1 = diag([1 1 1]);
K_2 = diag([1 1 1]);
K_ori = [2 1];

% Variáveis do Caminho
RaioX = 2.5;
RaioY = 1.5;
CentroX = 0;
CentroY = 0;
inc = 0.1; % Resolução do Caminho
inc_desvio = 0.1;
nCaminho = 360; % Número de pontos total no caminho
s = 0:inc:nCaminho; % Abcissa curvilinea
% x = -RaioX*sin(pi*s/180) + CentroX;
% y = RaioY*sin(2*pi*s/180) + CentroY;
x = s/60 - 2;
y = 0*ones(1,length(s));
z = 0*ones(1,length(s));
C_normal = [x; y; z];
dist_fim = 0.2; % Distância ao ponto final onde considero que terminei a navegação
dist_final = 1000;
tol_caminho = 0.2; % Tolerância na qual considero o robô sobre o caminho
Ve = 0.3; % Velocidade desejada no caminho

% Variáveis dos obstáculos
% Obstaculos = [-1 0.8 0;-0.2 1.4 0;0.7 1.8 0 ;2.1 2.4 0]';
%Obstaculos = [0.0474 -0.9137 0; 0.0194 0.664 0];
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

histerese_tempo = 0.01;
t_histerese = tic;

%Video
indVideo = 1;

ind = 1;

figure
% try
    while dist_final > dist_fim
%     while t_max > toc(t)
        if toc(t_c) > T_c
            t_c = tic;
            
            rb = OPT.RigidBody;
            P2 = getOptData(rb(idT1),P2);   % get trailer 1
            P3 = getOptData(rb(idT2),P3);   % get trailer 2
            
%             Obstaculos = [P2.pPos.Xc(1:3) P3.pPos.Xc(1:3) [0.0474 -1.0737 0]' [0.0194 0.664 0]'];
            Obstaculos = [P2.pPos.Xc(1:3) P3.pPos.Xc(1:3) [1.5474 -1.9737 0]' [0.0194 0.664 0]'];
            
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
                    [C_desvio,ind_caminho_obs_final] = gera_caminho_desvio_mudanca(X1,C_normal,obs_ativo,dist_min_obs,inc_desvio,ind,obs_ativo_antigo);

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
                    [C_desvio,ind_caminho_obs_final] = gera_caminho_desvio(C_normal,obs_ativo,dist_min_obs,inc_desvio,ind);

                    flag_desvio = 1;
                    flag_desvio_acabando = 1;
                    flag_mudanca_obstaculo = 0;
                end
            end
            
            if flag_desvio == 1
                if norm(X1(1:2)-C_normal(1:2,ind_caminho_obs_final)) < tol_caminho || ind > size(C_desvio,2)-50
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
            Xd1(1:2) = C(1:2,ind);
            
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
                dXd1(1:4) = 0;
            else
                dXd1(1:2) = [Vx Vy]';
            end
            
            % Calcula o erro de posição
            Xtil(1:3) = Xd1(1:3) - X1(1:3);
            
            % Lei de Controle
            Xr1(1:3) = dXd1(1:3) + K_1*tanh(K_2*Xtil(1:3));
            
            
            %% Cinemática
            % Matriz de cinemática estendida
            %             K_inv = [cos(X1(4)) sin(X1(4)) par_a*sin(alpha);
            %                 -sin(X1(4)) cos(X1(4)) -par_a*cos(alpha);
            %                 0           0           1     ];
            
            % Matriz de cinemática Normal
            K = [cos(X1(4)) -par_a*sin(X1(4));
                sin(X1(4)) par_a*cos(X1(4))];
            
            U1 = K\Xr1([1 2]);
            
            %% Saturação
            if abs(U1(1)) > sat_v
                U1(1) = sat_v*sign(U1(1));
            end
            
            if abs(U1(2)) > sat_w
                U1(2) = sat_w*sign(U1(2));
            end
            
            
            %% Simulação

            % Velocidade do robô na referencia do mundo
            dX1([1 2]) = K*U1;
            dX1(3) = 0;
            dX1(4) = U1(2);

            % Cálculo de posição na referência do mundo
            X1 = X1 + dX1*toc(t_sim);
            t_sim = tic;
            
            %% Figura
            t_hist = [t_hist toc(t)];
            X1_hist = [X1_hist X1];
            dX1_hist = [dX1_hist dX1];
            Xd1_hist = [Xd1_hist Xd1];
            U1_hist = [U1_hist U1];
            
            Corpo = [raio*cos(circ);raio*sin(circ)] + X1(1:2);
            Corpo_frente = X1(1:2) + [(raio+1)*cos(X1(4));(raio+1)*sin(X1(4))];
            Corpo_obs_1 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,1);
            Corpo_obs_2 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,2);
            Corpo_obs_3 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,3);
            Corpo_obs_4 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,4);
            
            plot(X1(1),X1(2),'*')
            hold on
            grid on
            plot(X1_hist(1,:),X1_hist(2,:))
            plot(C_normal(1,:),C_normal(2,:),'--')
            plot(Corpo(1,:),Corpo(2,:))
            plot(Corpo_obs_1(1,:),Corpo_obs_1(2,:))
            plot(Corpo_obs_2(1,:),Corpo_obs_2(2,:))
            plot(Corpo_obs_3(1,:),Corpo_obs_3(2,:))
            plot(Corpo_obs_4(1,:),Corpo_obs_4(2,:))
            plot([X1(1) Corpo_frente(1)],[X1(2) Corpo_frente(2)])
            %         plot(xd_p(1),xd_p(2),'*')
            plot(Obstaculos(1,:),Obstaculos(2,:),'*')
            try
                plot(C_desvio(1,:),C_desvio(2,:),'g')
            end
            %         axis([0 6 -2.5 2.5])
            axis([-4 4 -3 3])
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
X1til_hist = Xd1_hist - X1_hist;

figure
plot(t_hist,X1til_hist(1:2,:))
grid on
legend('Erro X','Erro Y')

figure
plot(t_hist,U1_hist([1 2],:))
grid on
legend('Comandos V','Comandos W')


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