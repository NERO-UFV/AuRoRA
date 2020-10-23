clear all
close all
clc

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

%% Load Class
try
    % Load Classes
    setenv('ROS_MASTER_URI','http://192.168.0.102:11311')
    setenv('ROS_IP','192.168.0.105')
    RI = RosInterface;
    RI.rConnect('192.168.0.102');
    
%     P = RPioneer(1,'RosAria',1);
    P = RPioneer(1,'RosAria',2);
    P2 = Pioneer3DX(1);
%     P3 = Pioneer3DX(2);
    
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
fig1 = figure;
fig2 = figure;

nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Parent',fig2,'Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

%% Variable initialization
data = [];

idP = 1;
idT1 = 2;
% idT2 = 3;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%
rb = OPT.RigidBody;          % read optitrack data
P = getOptData_rmma(rb(idP),P);   % get pioneer data
P2 = getOptData_rmma(rb(idT1),P2);   % get pioneer data

% Variáveis do Robô
velocidade_real = zeros(4,1); % Velocidades controle do Robô real
U_robo_real = zeros(6,1); % Comandos do Robô
P_robo = zeros(6,1); % Comandos do Robô
P_centro_trailer1 = zeros(6,1);
U1_robo_real_anterior = 0;
U3_robo_real_anterior = 0;
itae_anterior = 0;
iae_anterior = 0;
itae_v_linear_anterior = 0;
iae_v_linear_anterior = 0;
itae_v_angular_anterior = 0;
iae_v_angular_anterior = 0;

% Cinemática estendida do pioneer
alpha = pi;
sat_v = 0.75;
sat_w = 100*pi/180;
b = 0.2;

% Variaveis para plot corpo
raio = 0.15;
circ = 0:0.01:2*pi;

% Variaveis de Ganho
K_1 = 0.8*diag([1 1]);
K_2 = 0.8*diag([1 1]);
k_u = 4;
k_w = 4;

% Variaveis do caminho
RaioX = 2;           % [m]
RaioY = 1.5;           % [m]
CentroX = 0;
CentroY = 0;
inc = 0.1; % Resolução do Caminho
nCaminho = 355; % Número de pontos total no caminho
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
Ve = 0.35; % Velocidade desejada no caminho

% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_sim = tic; % Temporizador de simulação
t_derivada_real = tic;
t_derivada_sim = tic;
T_CONTROL = 1/10;
t_control = tic;
tempo_erro = tic;
tempo_erro_v = tic;

% thetas primeira identificacao
% t_1 = 0.23025;
% t_2 = 0.22615;
% t_3 = 0.00028953;
% t_4 = 0.95282;
% t_5 = 0.021357;
% t_6 = 0.95282;

% thetas primeira identificacao
% t_1 = 0.21591;
% t_2 = 0.26903;
% t_3 = -0.012686;
% t_4 = 0.89723;
% t_5 = 0.56243;
% t_6 = 0.25359;

t_1=0.23882;
t_2=0.23938;
t_3=0.0038418;
t_4=0.9435;
t_5=-0.007849;
t_6=0.92249;

Theta_original = [t_1;t_2;t_3;t_4;t_5;t_6];

% Variáveis de histórico
t_hist = [];
P_real_X_hist = [];
P2_real_Xtil_hist = [];
P2_real_X_hist = [];
P2_real_Xd_hist = [];
Theta_est_hist_real = [];
theta_1_hist = [];
theta_1_pot_hist = [];
sinal_controle_hist = [];
sinal_controle_trailer_hist = [];
erro_hist = [];
erro_v_linear_hist = [];
erro_v_angular_hist = [];
V_desejada_caminho_hist = [];
V_real_trailer1_hist = [];

% Variáveis de curvatura
lim_curvatura = 0.6;
K_curv = 1;

%%% dimensoes trailers
L0 = 0.3;
L1 = 0.455;
%%% dimensoes trailers simulacao
ret_lg = 0.415;
ret_lp = 0.285;

U_trailer1_real(3) = 0;

k = 0;

fprintf('\nStart..............\n\n');

%Video
indVideo = 1;

try
while dist_final > dist_fim
    if toc(t_control) > T_CONTROL
        
        k = k+1;
        Vd = Ve;

%         if abs(U_trailer1_real(3)) > lim_curvatura
%         Vd = Vd/(1+K_curv*abs(U_trailer1_real(3)));
%         end
        
        t_control = tic;
        
        P.rGetPotentiometerData;
        rb = OPT.RigidBody; 
        P = getOptData_rmma(rb(idP),P);   % get pioneer data
        P2 = getOptData_rmma(rb(idT1),P2);   % get pioneer data
                        
        x_r = P2.pPos.Xc(1) + (L1+b)*cos(P2.pPos.Xc(6)) + L0*cos(P.pPos.Xc(6));
        y_r = P2.pPos.Xc(2) + (L1+b)*sin(P2.pPos.Xc(6)) + L0*sin(P.pPos.Xc(6));
        
        x_c_t1 = P2.pPos.Xc(1) + b*cos(P2.pPos.Xc(6));
        y_c_t1 = P2.pPos.Xc(2) + b*sin(P2.pPos.Xc(6));
        
        P_robo([1 2]) = ([x_r y_r]);
        P_centro_trailer1([1 2]) = ([x_c_t1 y_c_t1]);
        
        % Distancia do robô para o final do caminho
        dist_final = norm(C_normal(:,end) - P2.pPos.Xc(1:3));
        
        % Calcula o ponto do caminho mais próximo do robô
        [dist, ind] = calcula_ponto_proximo(C_normal(1:2,:),P2.pPos.Xc(1:2));
        
        % Define o ponto desejado
        P2.pPos.Xd(1:2) = C_normal(1:2,ind);
        
        % Calcula o angulo do vetor tangente ao caminho
        theta_caminho = atan2(C_normal(2,ind+1*sign(Vd)) - C_normal(2,ind),C_normal(1,ind+1*sign(Vd)) - C_normal(1,ind));
        
        % Calcula as projeções nos eixos x e y do vetor velocidade tangente
        % ao caminho
        Vx = abs(Vd)*cos(theta_caminho);
        Vy = abs(Vd)*sin(theta_caminho);
        
        % Define a velocidade desejada no referencial do mundo. Isto é
        % dividido nos casos onde o robô esta fora do caminho e onde ele
        % está sobre o caminho
        
        P2.pPos.Xd(7) = Vx;
        P2.pPos.Xd(8) = Vy;
          
        %%% erros
        P2.pPos.Xtil = P2.pPos.Xd - P2.pPos.Xc;
        
        %%% Velocidades - lei de controle real
        v_real = P2.pPos.Xd([7 8]) + K_1*tanh(K_2*P2.pPos.Xtil([1 2]));
        vx_real = v_real([1]);
        vy_real = v_real([2]);
        v_fi_real = (-vx_real/(b*cos(alpha)))*sin(P2.pPos.Xc([6])) +  (vy_real/(b*cos(alpha)))*cos(P2.pPos.Xc([6]));
        
       velocidade_real([1 2 4]) = [vx_real vy_real v_fi_real];
        
       K_real = [ cos(P2.pPos.Xc([6])), sin(P2.pPos.Xc([6])), b*sin(alpha); ...
            -sin(P2.pPos.Xc([6])), cos(P2.pPos.Xc([6])), -b*cos(alpha); ...
            0, 0, 1];
        
       %%%% sinal de controle cinematico real
       U_trailer1_real = K_real*velocidade_real([1 2 4]);
       U_trailer1_real(2) = 0;
       
       theta_1 = P.pPos.Xc([6]) - P2.pPos.Xc([6]);
       theta_1_pot = P.pPot1.Data;

        if abs(theta_1) > pi
           theta_1 = theta_1 - 2*pi*sign(theta_1);
        end
       
       v0_c = U_trailer1_real(1)*cos(theta_1) + (U_trailer1_real(3))*(L1)*sin(theta_1);
       w0_c = U_trailer1_real(1)*sin(theta_1)/L0 - (U_trailer1_real(3))*(L1)*cos(theta_1)/L0;
                
       U_robo_real([1 2 3]) = [v0_c 0 w0_c];   %%%% sinal de controle cinematico para o robo1
       
        %%%%% calculo velocidade linear real do robo (no espaço do robo e dos trailers)       
        K_roboz = [ cos(P.pPos.Xc([6])), sin(P.pPos.Xc([6])), 0; ...
            -sin(P.pPos.Xc([6])), cos(P.pPos.Xc([6])), 0; ...
            0, 0, 1];
        
        velocidades_robo = K_roboz*([P.pPos.Xc(7);P.pPos.Xc(8);P.pPos.Xc(12)]);
       
         %%%% -------------   compensador dinâmico real ---------------
          U1_robo_real_ref = (U_robo_real(1) - U1_robo_real_anterior)/toc(t_derivada_real);
          U3_robo_real_ref = (U_robo_real(3) - U3_robo_real_anterior)/toc(t_derivada_real);
          t_derivada_real = tic;
          U1_robo_real_anterior = U_robo_real(1);
          U3_robo_real_anterior = U_robo_real(3);
          
          G_real = [0 0 -velocidades_robo(3)^2 velocidades_robo(1) 0 0; ...
               0 0 0 0 velocidades_robo(3)*velocidades_robo(1) velocidades_robo(3)];
          
          delta_1_real = U1_robo_real_ref + k_u*(U_robo_real(1) - velocidades_robo(1));
          delta_2_real = U3_robo_real_ref + k_w*(U_robo_real(3) - velocidades_robo(3));
          
          A_din = [Theta_original(1) 0; 0 Theta_original(2)];
          
          dados = [U1_robo_real_ref;U3_robo_real_ref;delta_1_real;delta_2_real];
          
          %%%% dinamico / cinematico
%           P.pSC.Ud(1:2) = A_din*[delta_1_real;delta_2_real] + G_real*Theta_original;
        P.pSC.Ud(1:2) = U_robo_real([1 3]);  %%% cinematico

        %Saturação
        if abs(P.pSC.Ud(1)) > sat_v
            P.pSC.Ud(1) = sat_v*sign(P.pSC.Ud(1));
        end
        
        if abs(P.pSC.Ud(2)) > sat_w
            P.pSC.Ud(2) = sat_w*sign(P.pSC.Ud(2));
        end
        
%         P.pSC.Ud(1:2)
        
        P = J.mControl(P);  
        P.rCommand;
        
        if abs(theta_1) > 90*pi/180     
            disp('jacknife')
            P.rCmdStop;
            break;
        end
%              
        
      %%%%%%%%%%% ---------------------Calculo erro posicao ---------------------
      erro = sqrt((P2.pPos.Xd(1) - P2.pPos.Xc(1))^2 + (P2.pPos.Xd(2) - P2.pPos.Xc(2))^2);
      time_erro = toc(tempo_erro);

      itae = (time_erro)*erro;
      itae = itae + itae_anterior;
      itae_anterior = itae;

      iae = erro;
      iae = iae + iae_anterior;
      iae_anterior = iae;
      
      erros = [iae itae];
      %%%%% ------------------------------------------------

      %%%%% calculo velocidade linear real do trailer em relacao ao robo (no espaço do robo e dos trailers)     
      velocidades_trailer1 = K_real*([P2.pPos.Xc(7);P2.pPos.Xc(8);P2.pPos.Xc(12)]);
      
      %%%%%%%%%%% ---------------------Calculo erro velocidade ---------------------
      erro_v_linear = abs(-Vd - velocidades_trailer1(1));
      erro_v_angular = abs(v_fi_real - velocidades_trailer1(3));
      
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
      
      Error = [erros;erros_v_linear;erros_v_angular];
      
      %%%%% ------------------------------------------------

        t_hist = [t_hist toc(t)];
        P_real_X_hist = [P_real_X_hist P.pPos.Xc];
        P2_real_Xtil_hist = [P2_real_Xtil_hist P2.pPos.Xtil(1:2)];
        P2_real_X_hist = [P2_real_X_hist P2.pPos.Xc];
        P2_real_Xd_hist = [P2_real_Xd_hist P2.pPos.Xd(1:2)];
        theta_1_hist = [theta_1_hist theta_1];
        theta_1_pot_hist = [theta_1_pot_hist theta_1_pot];
        erro_hist = [erro_hist erros'];
        sinal_controle_hist = [sinal_controle_hist P.pSC.Ud(1:2)];
        sinal_controle_trailer_hist = [sinal_controle_trailer_hist U_trailer1_real([1 3])];
        erro_v_linear_hist = [erro_v_linear_hist erros_v_linear'];
        erro_v_angular_hist = [erro_v_angular_hist erros_v_angular'];
        V_desejada_caminho_hist = [V_desejada_caminho_hist Vd];
        V_real_trailer1_hist = [V_real_trailer1_hist velocidades_trailer1];
        
        %%
        figure(fig1)
        
        Corpo1_real = [raio*cos(circ);raio*sin(circ)] + P_robo(1:2);
        Corpo_frente = P_robo(1:2) + [(raio+0.15)*cos(P.pPos.Xc(6));(raio+0.15)*sin(P.pPos.Xc(6))];
        
        Corpo2_real = [cos(P2.pPos.Xc(6)) -sin(P2.pPos.Xc(6));sin(P2.pPos.Xc(6)) cos(P2.pPos.Xc(6))]*[-ret_lg/2 ret_lg/2 ret_lg/2 -ret_lg/2 -ret_lg/2;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + P_centro_trailer1(1:2);
        haste_L0 = L0*[cos(P.pPos.Xc(6)) -sin(P.pPos.Xc(6));sin(P.pPos.Xc(6)) cos(P.pPos.Xc(6))]*[-1;0] + P_robo(1:2);
        haste_L1 = L1*[cos(P2.pPos.Xc(6)) -sin(P2.pPos.Xc(6));sin(P2.pPos.Xc(6)) cos(P2.pPos.Xc(6))]*[1;0] + P_centro_trailer1(1:2);
        
        plot(P2.pPos.Xc(1),P2.pPos.Xc(2),'h')
        hold on
        grid on
%         plot(P2_real_Xd_hist(1,:),P2_real_Xd_hist(2,:),'b-')
        plot(C_normal(1,:),C_normal(2,:),'-')
        plot(Corpo1_real(1,:),Corpo1_real(2,:),'k','LineWidth',2)
        plot(P2_real_X_hist(1,:),P2_real_X_hist(2,:),'r--','LineWidth',2)
        plot(Corpo2_real(1,:),Corpo2_real(2,:),'k','LineWidth',2)
        plot([P_robo(1) Corpo_frente(1)],[P_robo(2) Corpo_frente(2)],'k','LineWidth',2)
        plot([P_robo(1) haste_L0(1)],[P_robo(2) haste_L0(2)],'k','LineWidth',2)
        plot([P_centro_trailer1(1) haste_L1(1)],[P_centro_trailer1(2) haste_L1(2)],'k','LineWidth',2)
        plot([haste_L0(1)],[haste_L0(2)],'ko')
        axis([-2.5 2.5 -2 2])
%         axis([-3 3 -3 3])
        hold off
        
        drawnow
        
        Video(indVideo) = getframe(gcf);
        indVideo = indVideo + 1;
                
%         hold on
%         grid on
%         plot(t_hist(1,:),theta_1_hist(1,:))
%         axis([0 50 -5 5])
%         hold off

%         hold on
%         grid on
%         plot(t_hist(1,:),P_real_Xtil_hist(1,:))
%         plot(t_hist(1,:),P_real_Xtil_hist(2,:))
%         axis([0 20 -0.5 0.5])
%         hold off  

%         hold on
%         grid on
%         plot(t_hist(1,:),P_real_X_hist(6,:))
%         plot(t_hist(1,:),P2_real_X_hist(6,:))
%         axis([0 50 -10 10])
%         hold off 
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

%         figure(1);
%         hold on
%         grid on
%         plot(t_hist(1,:),-V_desejada_caminho_hist(1,:))
%         plot(t_hist(1,:),V_real_trailer1_hist(1,:))
%         axis([0 65 -2 2])
%         legend('u_{desejado}','u_{real}')
%         hold off
% %         
%         figure(2);
%         hold on
%         grid on
%         plot(t_hist(1,:),sinal_controle_trailer_hist(2,:))
%         plot(t_hist(1,:),V_real_trailer1_hist(3,:))
%         axis([0 65 -2 2])
%         legend('w_{desejado}','w_{real}')
%         hold off
%         
%         figure(3);
%         hold on
%         grid on
%         plot(t_hist(1,:),erro_hist(1,:))
%         plot(t_hist(1,:),erro_hist(2,:))
%         axis([0 65 0 700])
%         legend('iae','itae')
%         hold off
% % 
%         figure(4);
%         hold on
%         grid on
%         plot(t_hist(1,:),erro_v_linear_hist(1,:))
%         plot(t_hist(1,:),erro_v_linear_hist(2,:))
%         axis([0 65 0 10000])
%         legend('iae_{vlinear}','itae_{vlinear}')
%         hold off
% % %         
%         figure(5);
%         hold on
%         grid on
%         plot(t_hist(1,:),erro_v_angular_hist(1,:))
%         plot(t_hist(1,:),erro_v_angular_hist(2,:))
%         axis([0 65 0 10000])
%         legend('iae_{vangular}','itae_{vangular}')
%         hold off

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
    P.rCmdStop;
    
end

%Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

% % % %% 
% path = 'C:\Users\lai1u\Dropbox\AuRoRA 2018\ROS\Scripts\DiegoCBA\simulacoes\Testes_artigo1\experimentos\';
% filename = ['Diego_cinematico_filmado_leminiscata_dl_v_' num2str(Ve) '_Data_' datestr(now,30) '.mat'];
% fullname = [path filename];
% save(fullname,'t_hist','P_real_X_hist','P2_real_X_hist','P2_real_Xtil_hist','P2_real_Xd_hist','theta_1_hist','theta_1_pot_hist','erro_hist','sinal_controle_hist','sinal_controle_trailer_hist','erro_v_linear_hist','erro_v_angular_hist','V_desejada_caminho_hist','V_real_trailer1_hist','Error','Video');

%%
%%%%%%%%%%%%% video %%%%%%%%%%%%%%%%%%%
% 
%%%%%%%%%%%create the video writer with 1 fps
%   writerObj = VideoWriter(filename,'MPEG-4');
%   writerObj.FrameRate = 10;
%   %%%%%%%%%%%%%%  set the seconds per image
% 
% %%%%%%%%%%%%    open the video writer
%     open(writerObj);
% %%%%%%%%%%%%%%    write the frames to the video
% for i=1:length(Video)
%     %%%%%%%%%%%%%%  convert the image to a frame
%     frame = Video(i) ;
%     writeVideo(writerObj, frame);
% end
% %%%%%%%%%%%%%%%%%%    close the writer object
% close(writerObj);