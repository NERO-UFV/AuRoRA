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
    % Load Classes
        setenv('ROS_MASTER_URI','http://192.168.0.106:11311')
        setenv('ROS_IP','192.168.0.105')
    RI = RosInterface;
    RI.rConnect('192.168.0.106');
    %     B = Bebop(1,'B');
    
    %     P = Pioneer3DX(1);  % Pioneer Instance
    P1 = RPioneer(1,'RosAria');
    P2 = Pioneer3DX(1);
    P3 = Pioneer3DX(2);
    
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
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

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
P1 = getOptData(rb(idP),P1);   % get pioneer data
P2 = getOptData(rb(idT1),P2);   % get pioneer data
P3 = getOptData(rb(idT2),P3);   % get pioneer data
%%

% Variáveis do Robô
X1 = zeros(4,1); % Posição do Robô 1
dX1 = zeros(4,1); % Velocidade do Robô 1
ddX1_b = zeros(4,1); % Aceleração do Robô 1
Xd1 = zeros(4,1); % Posição Desejada do Robô 1
dXd1 = zeros(4,1); % Velocidade Desejada do Robô 1
U1 = zeros(4,1); % Comandos do Robô 1
Xtil = zeros(4,1);
Xr1 = zeros(4,1);
X1 = [2.5 0.5 0 pi/2]';
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
% x = RaioX*sin(pi*s/180) + CentroX;
% y = RaioY*cos(pi*s/180) + CentroY;
x = RaioX*sin(pi*s/180) + CentroX;
y = RaioY*sin(2*pi*s/180) + CentroY;
z = 0*ones(1,length(s));
C_normal = [x; y; z];
dist_fim = 0.1; % Distância ao ponto final onde considero que terminei a navegação
dist_final = 1000;
tol_caminho = 0.2; % Tolerância na qual considero o robô sobre o caminho
Ve = 0.2; % Velocidade desejada no caminho

% Variáveis dos obstáculos
% Obstaculos = [3 2.1 0;4.9 0 0;4 -2 0;1.286 1 0;4.5 1.5 0;2 1.5 0;2 -1.5 0]';
Obstaculos = [1.5 -1.75 0;4.9 0 0;3.5 0.5 0]';
% Obstaculos = [1.5 -2.25 0;4.9 0 0]';
dist_detecta_obs = 1.2;
dist_min_obs = 0.8;
flag_desvio = 0;

% Variáveis de curvatura
lim_curvatura = 0.5;
K_curv = 1;

% Variáveis de histórico
t_hist = [];
X1_hist = [];
dX1_hist = [];
Xd1_hist = [];
U1_hist = [];


figure
try
%     while dist_final > dist_fim
    while t_max > toc(t)
        if toc(t_c) > T_c
            t_c = tic;
            
            rb = OPT.RigidBody;
            P1 = getOptData(rb(idP),P1);   % get pioneer data
            P2 = getOptData(rb(idT1),P2);   % get trailer 1
            P3 = getOptData(rb(idT2),P3);   % get trailer 2
            
            X1 = [P1.pPos.X(1:3);P1.pPos.X(6)];
            Obstaculos = [P2.pPos.Xc(1:3) P3.pPos.Xc(1:3)];
%             Obstaculos = [50 50 0]';
            %% Definidor de Caminho
            
            % Define o obstáculo ativo no momento. Há apenas um obstáculo ativo
            % e este deve estar com uma distancia menor que um limiar mínimo
            obs_ativo_dist = dist_min_obs;
            for i = 1:size(Obstaculos,2)
                obs_ativo_temp =  Obstaculos(:,i);
                obs_ativo_dist_temp = norm(X1(1:2) - obs_ativo_temp(1:2));
                if obs_ativo_dist_temp < obs_ativo_dist
                    obs_ativo = obs_ativo_temp;
                    obs_ativo_dist = obs_ativo_dist_temp;
                    obs_ativo_ind = i;
                end
            end
            
            % Esta condição é feita apenas uma vez no início de cada manobra de
            % desvio. Ela vai criar o caminho novo que será seguido
            if obs_ativo_dist < dist_min_obs && flag_desvio == 0
                % Descobre o índice atual de navegação do robô e o índice mais
                % próximo do obstáculo
                [dist_desvio_inicio, ind_desvio_inicio] = calcula_ponto_proximo(C_normal(1:2,:),X1(1:2));
                [dist_caminho_obs, ind_caminho_obs] = calcula_ponto_proximo(C_normal(1:2,:),obs_ativo(1:2));
                
                % Este laço utiliza o índice mais próximo do obstáculo, para
                % descobrir o primeiro índice depois desse onde a distância ao
                % obstáculo é maior que o limiar de desvio
                i = 1;
                dist_caminho_obs_final = 0;
                while dist_caminho_obs_final < dist_min_obs
                    dist_caminho_obs_final = norm(C_normal(1:2,ind_caminho_obs+i)-obs_ativo(1:2));
                    i = i + 1;
                end
                ind_caminho_obs_final = ind_caminho_obs+i;
                
                % Define o novo caminho de desvio
                RaioX_desvio = norm(C_normal(1:2,ind_desvio_inicio) - C_normal(1:2,ind_caminho_obs_final))/2;
                RaioY_desvio = RaioX_desvio;
                CentroX_desvio = obs_ativo(1);
                CentroY_desvio = obs_ativo(2);
                
                % Acha o ângulo entre o arco entre o obstáculo e os pontos
                % iniciais e finais do novo caminho
                ang_obs_inicio = atan2(X1(2) - obs_ativo(2),X1(1) - obs_ativo(1))*180/pi;
                ang_obs_final = atan2(C_normal(2,ind_caminho_obs_final) - obs_ativo(2),C_normal(1,ind_caminho_obs_final) - obs_ativo(1))*180/pi;
                
                if ang_obs_inicio < 0
                    ang_obs_inicio_positivo = ang_obs_inicio + 360;
                else
                    ang_obs_inicio_positivo = ang_obs_inicio;
                end
                
                if ang_obs_final < 0
                    ang_obs_final_positivo = ang_obs_final + 360;
                else
                    ang_obs_final_positivo = ang_obs_final;
                end
                
                if abs(ang_obs_inicio_positivo - ang_obs_final_positivo) < 180
                    if ang_obs_inicio_positivo > ang_obs_final_positivo
                        inc_desvio = -abs(inc_desvio);
                    else
                        inc_desvio = abs(inc_desvio);
                    end
                else
                    if ang_obs_inicio_positivo > ang_obs_final_positivo
                        inc_desvio = abs(inc_desvio);
                    else
                        inc_desvio = -abs(inc_desvio);
                    end
                end
                
                % Cuida da descontinuidade em [180,-180]
                if inc_desvio > 0 && ang_obs_inicio > 0 && ang_obs_final < 0
                    ang_obs_final = ang_obs_final_positivo;
                end
                
                if inc_desvio < 0 && ang_obs_inicio < 0 && ang_obs_final > 0
                    ang_obs_inicio = ang_obs_inicio_positivo;
                end
                
                s_desvio = ang_obs_inicio:inc_desvio:ang_obs_final;
                %             s_desvio = ang_obs_inicio:
                
                x_desvio = RaioX_desvio*cos(pi*s_desvio/180) + CentroX_desvio;
                y_desvio = RaioY_desvio*sin(pi*s_desvio/180) + CentroY_desvio;
                z_desvio = 0*ones(1,length(s_desvio));
                C_desvio = [x_desvio; y_desvio; z_desvio];
                
                flag_desvio = 1;
            end
            
            if flag_desvio == 1
                if norm(X1(1:2)-C_normal(1:2,ind_caminho_obs_final)) < tol_caminho
                    flag_desvio = 0;
                end
            end
            
            if flag_desvio == 0
                C = C_normal;
            else
                            C = C_desvio;
%                 C = C_normal;
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
            if ind == length(C)
                theta_caminho = atan2(C(2,ind) - C(2,1),C(1,ind) - C(1,1));
            else
                theta_caminho = atan2(C(2,ind+1*sign(Vd)) - C(2,ind),C(1,ind+1*sign(Vd)) - C(1,ind));
            end
            %         theta_caminho = 0;
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
            
%             %% Cálculo Psi desejado - Cinemática estendida
%             % Define o Psi e Psi_ponto desejados para a cinemática estendida
%             Xd1(4) = atan2(Xr1(2),Xr1(1));
%             %         dXd1(4) = Xr1(2)/(Xr1(1)^2 + Xr1(2)^2);
%             dXd1_aux = Xd1(4) - Xd14a;
%             if abs(dXd1_aux) > pi
%                 dXd1_aux = dXd1_aux - 2*pi*sign(dXd1_aux);
%             end
%             dXd1(4) = dXd1_aux/toc(t_der);
%             t_der = tic;
%             Xd14a = Xd1(4);
%             
%             % Calcula o erro de orientação
%             Xtil(4) = Xd1(4) - X1(4);
%             
%             if abs(Xtil(4)) > pi
%                 Xtil(4) = Xtil(4) - 2*pi*sign(Xtil(4));
%             end
%             
%             % Lei de Controle de Orientação
%             Xr1(4) = dXd1(4) + K_ori(1)*tanh(K_ori(2)*Xtil(4));
            
            
            
            %% Cinemática
            % Matriz de cinemática Normal
%             K_inv = [cos(X1(4)) sin(X1(4)) par_a*sin(alpha);
%                 -sin(X1(4)) cos(X1(4)) -par_a*cos(alpha);
%                 0           0           1     ];
            
            K = [cos(X1(4)) -par_a*sin(X1(4));
                sin(X1(4)) par_a*cos(X1(4))];
            
            U1 = K\Xr1([1 2]);
%             U1(2) = 0;
            
            %Saturação
            if abs(U1(1)) > sat_v
                U1(1) = sat_v*sign(U1(1));
            end
            
            if abs(U1(2)) > sat_w
                U1(2) = sat_w*sign(U1(2));
            end

            P1.pSC.Ud(1:2) = [U1(1); U1(2)];
            
            P1 = J.mControl(P1);                    % joystick command (priority)
            P1.rCommand;
            
            
            %% Figura
            t_hist = [t_hist toc(t)];
            X1_hist = [X1_hist X1];
%             dX1_hist = [dX1_hist dX1];
            Xd1_hist = [Xd1_hist Xd1];
            U1_hist = [U1_hist U1];
            
            Corpo = [raio*cos(circ);raio*sin(circ)] + X1(1:2);
            Corpo_frente = X1(1:2) + [(raio+1)*cos(X1(4));(raio+1)*sin(X1(4))];
            
            plot(X1(1),X1(2),'*')
            hold on
            grid on
            plot(X1_hist(1,:),X1_hist(2,:))
            plot(C_normal(1,:),C_normal(2,:),'--')
            plot(Corpo(1,:),Corpo(2,:))
            plot([X1(1) Corpo_frente(1)],[X1(2) Corpo_frente(2)])
            %         plot(xd_p(1),xd_p(2),'*')
            plot(Obstaculos(1,:),Obstaculos(2,:),'*')
            try
                plot(C_desvio(1,:),C_desvio(2,:),'g--')
            end
                        axis([-4 4 -3 3])
            hold off
%             
            drawnow
            
            if btnEmergencia ~= 0
                disp('Bebop Landing through Emergency Command ');
                
                % Send 3 times Commands 1 second delay to Drone Land
                
                break;
            end
        end
    end
    
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    P1.rCmdStop;
    
end



%% Figuras
X1til_hist = Xd1_hist - X1_hist;

figure
plot(t_hist,X1til_hist(1:2,:))
grid on
legend('Erro X','Erro Y')

figure
plot(t_hist,U1_hist(1:2,:))
grid on
legend('Comandos V','Comandos W')