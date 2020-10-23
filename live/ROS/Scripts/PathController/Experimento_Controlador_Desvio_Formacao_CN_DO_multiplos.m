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
    setenv('ROS_MASTER_URI','http://192.168.0.102:11311')
    setenv('ROS_IP','192.168.0.105')
    RI = RosInterface;
    RI.rConnect('192.168.0.102');
    
    B1 = Bebop(1,'B');
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

% Bebop
disp('Start Take Off Timming....');
B1.rTakeOff;
pause(3);
disp('Taking Off End Time....');

%% Variable initialization
data = [];

idB = 1;
idP = 2;
idT1 = 3;
idT2 = 4;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%
rb = OPT.RigidBody;          % read optitrack data
B1 = getOptData(rb(idB),B1);   % get bebop data
P1 = getOptData(rb(idP),P1);   % get pioneer data
P2 = getOptData(rb(idT1),P2);   % get pioneer data
P3 = getOptData(rb(idT2),P3);   % get pioneer data


%% Variáveis do Robô
% Pioneer
X1 = zeros(4,1); % Posição do Robô 1
dX1 = zeros(4,1); % Velocidade do Robô 1
ddX1_b = zeros(4,1); % Aceleração do Robô 1
Xd1 = zeros(4,1); % Posição Desejada do Robô 1
dXd1 = zeros(4,1); % Velocidade Desejada do Robô 1
U1 = zeros(4,1); % Comandos do Robô 1
X1 = [-2 0 0 0]';
Xd14a = 0;
dXd14a = 0;

% Drone
X2 = zeros(4,1); % Posição do Robô 1
dX2 = zeros(4,1); % Velocidade do Robô 1
ddX2_b = zeros(4,1); % Aceleração do Robô 1
Xd2 = zeros(4,1); % Posição Desejada do Robô 1
dXd2 = zeros(4,1); % Velocidade Desejada do Robô 1
U2 = zeros(4,1); % Comandos do Robô 1
X2 = [-2 1 1 0]';
Xd24a = 0;
dXd24a = 0;
U1_c_A = 0;
U2_c_A = 0;

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
raio_obs = 0.2;
circ = 0:0.01:2*pi;
% Corpo = [raio*cos(circ);raio*sin(circ)] + X1(1:2);
% Corpo_frente = X1(1:2) + [raio*cos(X1(4));raio*sin(X1(4))];

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_c = tic; % Temporizador de controle
t_sim = tic; % Temporizador de simulação
T_c = 0.1; % Tempo de controle
t_der = tic;
t_max = 600;
t_pouso = tic;
t_decola = tic;

% Variaveis de Ganho
K_1 = diag(0.8*[1.2 1.2 1 3 1.5 1.5]);
K_2 = diag(0.8*[1 1 1 1 1 1]);
K1_ori = 1;
K2_ori = 1;
Ku = diag([0.8417 0.8354 3.966 9.8524]);
Kv = diag([0.18227 0.17095 4.001 4.7295]);
K_bebop = diag([2 2 1.8 5]);

% Variáveis do Caminho
RaioX = 2.0;
RaioY = 1.2;
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
Ve = 0.4; % Velocidade desejada no caminho

% Variáveis dos obstáculos
% Obstaculos = [-1 0.8 0;-0.2 1.4 0;0.7 1.8 0 ;2.1 2.4 0]';
% Obstaculos = [0.0474 -0.9137 0; 0.0194 0.664 0]';
Obstaculos = [0.1 -0.2 0; 0.3 0.4 0]';
% Obstaculos = [50 50 0]';
dist_detecta_obs = 1.2;
dist_min_obs = 0.9;
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
rho_pouso = 1.5;
beta_pouso = 77*pi/180;
prepara_pouso = 0;
pouso = 0;
B1.pFlag.EmergencyStop = 0;
figure
try
    %     while dist_final > dist_fim
    while t_max > toc(t)
        if toc(t_c) > T_c
            t_c = tic;
            
            rb = OPT.RigidBody;          % read optitrack data
            B1 = getOptData(rb(idB),B1);   % get bebop data
            P1 = getOptData(rb(idP),P1);   % get pioneer data
            P2 = getOptData(rb(idT1),P2);   % get pioneer data
            P3 = getOptData(rb(idT2),P3);   % get pioneer data
            
            X1 = [P1.pPos.X(1:3);P1.pPos.X(6)];
            dX1 = [P1.pPos.X(7:9);P1.pPos.X(12)];
            X2 = [B1.pPos.X(1:3);B1.pPos.X(6)];
            dX2 = [B1.pPos.X(7:9);B1.pPos.X(12)];
            Obstaculos = [P2.pPos.Xc(1:2) P3.pPos.Xc(1:2);0 0];
            %             Obstaculos = [50 50 0]';
            
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
            
            if J.pDigital(2) == 1 && pouso == 0
                prepara_pouso = 1;
                t_pouso = tic;
            elseif J.pDigital(2) == 1 && pouso == 1
                prepara_pouso = 0;
                t_decola = tic;
            end
            % Define o ponto desejado
            Qd = [C(1:3,ind)' 1.5 0 pi/2]';
            % Posicionamento
            %             Qd = [X1(1:3)' 1.5 0 pi/2]';
            
            if prepara_pouso && pouso == 0
                Qd = [C(1:3,ind)' rho_pouso (X1(4)+pi) beta_pouso]';
                disp('prepara')
            end
            %             norm_erro = norm(Qd(4:6) - Q(4:6));
            norm_erro = 0;
            %
%             if (toc(t_pouso) > 5 && norm_erro < 0.03 && prepara_pouso) || (pouso == 1 && prepara_pouso == 1)
            if (J.pDigital(3) == 1 && prepara_pouso) || (pouso == 1 && prepara_pouso == 1)
                B1.rCmdStop;
                B1.rLand;
                B1.pFlag.EmergencyStop = 0;
                Qd = [C(1:3,ind)' norm(X2(1:3) - X1(1:3)) atan2(X2(2) - X1(2),X2(1) - X1(1)) atan2(X2(3) - X1(3),norm(X2(1:2) - X1(1:2)))]';
                pouso = 1;
                disp('pouso')
            end
            
            if prepara_pouso == 0 && pouso == 1
                B1.rTakeOff;
                B1.pFlag.EmergencyStop = 0;
                disp('decola')
                if toc(t_decola) > 2
                    pouso = 0;
                end
            end
            
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
            
            %Posicionamento
            %             dQd = [dX1(1) dX1(2) dX1(3) 0 dX1(4) 0]';
            
            % Formação
            rho = norm(X2(1:3) - X1(1:3));
            alpha = atan2(X2(2) - X1(2),X2(1) - X1(1));
            beta = atan2(X2(3) - X1(3),norm(X2(1:2) - X1(1:2)));
            
            Q = [X1(1) X1(2) X1(3) rho alpha beta]';
            
            % Calcula o erro de posição
            Qtil = Qd - Q;
            
            if abs(Qtil(5)) > pi
                if Qtil(5) > 0
                    Qtil(5) = -2*pi + Qtil(5);
                else
                    Qtil(5) =  2*pi + Qtil(5);
                end
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
            
            dXr = Jacob\dQr;
            
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
            U2_c = K_d\[dXr(4:6); dBref_O];
            
            %% Dinâmica
            
            %Derivando a velocidade desejada
            %             dU1_c = (U1_c - U1_c_A)/toc(t_der);
            dU2_c = (U2_c - U2_c_A)/toc(t_der);
            t_der = tic;
            
            %             U1_c_A = U1_c;
            U2_c_A = U2_c;
            
            %Velocidade do drone em seu próprio eixo
            V2 = K_d\([dX2(1:3);dX2(4)]);
            
            dU2_r = dU2_c + K_bebop*(U2_c - V2);
            U2 = Ku\(dU2_r + Kv*V2);
            
            %% Comando
            P1.pSC.Ud(1:2) = [U1(1); U1(2)];
            B1.pSC.Ud = [U2(1:3); 0; 0; U2(4)];
            %             B1.pSC.Ud = [0; 0; 0; 0; 0; 0];
            
            
            B1 = J.mControl(B1);                    % joystick command (priority)
            %             B1.pSC.Ud
            
%             P1.rCommand;
            if pouso == 0
                B1.rCommand;
            end
            
            
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
            
            Corpo = [raio*cos(circ);raio*sin(circ);0*sin(circ)] + X1(1:3);
            Corpo_frente = X1(1:3) + [(raio+1)*cos(X1(4));(raio+1)*sin(X1(4));0];
            Drone_frente = X1(1:3) + [(raio+1)*cos(X1(4));(raio+1)*sin(X1(4));0];
            Corpo_obs_1 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,1);
            Corpo_obs_2 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,2);
            %             Corpo_obs_3 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,3);
            %             Corpo_obs_4 = [raio_obs*cos(circ);raio_obs*sin(circ)] + Obstaculos(1:2,4);
            
            plot3(X1(1),X1(2),X1(3),'*')
            hold on
            grid on
            plot3(X2(1),X2(2),X2(3),'*')
            plot3(X1_hist(1,:),X1_hist(2,:),X1_hist(3,:))
            plot3(X2_hist(1,:),X2_hist(2,:),X2_hist(3,:))
            plot3(C_normal(1,:),C_normal(2,:),C_normal(3,:),'--')
            plot3(Corpo(1,:),Corpo(2,:),Corpo(3,:))
            plot(Corpo_obs_1(1,:),Corpo_obs_1(2,:))
            plot(Corpo_obs_2(1,:),Corpo_obs_2(2,:))
            %             plot(Corpo_obs_3(1,:),Corpo_obs_3(2,:))
            %             plot(Corpo_obs_4(1,:),Corpo_obs_4(2,:))
            plot3([X1(1) Corpo_frente(1)],[X1(2) Corpo_frente(2)],[X1(3) Corpo_frente(3)])
            %         plot(xd_p(1),xd_p(2),'*')
            plot3(Obstaculos(1,:),Obstaculos(2,:),Obstaculos(3,:),'*')
            try
                plot3(C_desvio(1,:),C_desvio(2,:),C_desvio(3,:),'g')
            end
            %         axis([0 6 -2.5 2.5])
            axis([-4 4 -3 3 0 2])
            axis([-2.5 2.5 -1.5 1.5 0 2])
%             view(-47,48)
            view(0,90)
            hold off
            
            Video(indVideo) = getframe(gcf);
            indVideo = indVideo + 1;
            
            if (J.pDigital(3) == 1 && prepara_pouso) || (pouso == 1 && prepara_pouso == 1)
                B1.pFlag.EmergencyStop = 0;
            end
            
            if prepara_pouso == 0 && pouso == 1
%                 B1.pFlag.EmergencyStop = 0;
            end
            
            if btnEmergencia ~= 0 || B1.pFlag.EmergencyStop ~= 0 || B1.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');
                B1.rCmdStop;
                B1.rLand;
                break;
            end
            
            drawnow
            
        end
    end
    
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B1.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    
end


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


% figure
% plot(t_hist,U1_hist([1 2],:))
% grid on
% legend('Comandos V','Comandos W')
%
% figure
% plot(t_hist,U2_hist([1 2 3],:))
% grid on
% legend('Comandos X','Comandos Y','Comandos Z')

%% Save
path = [pwd '\ROS\Scripts\PathController\'];
filename = ['Desvio_Formação_Video2' '_Data_' datestr(now,30) '.mat'];
fullname = [path filename];
% save(fullname,'t_hist','X1_hist','dX1_hist','Xd1_hist','U1_hist','X2_hist','dX2_hist','Xd2_hist','U2_hist','Q_hist','Qd_hist','Qtil_hist')
% 
% % create the video writer with 1 fps
%   writerObj = VideoWriter('Desvio_Formação_2obs.mp4','MPEG-4');
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