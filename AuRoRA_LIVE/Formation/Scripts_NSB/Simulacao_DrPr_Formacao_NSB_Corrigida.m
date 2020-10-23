%% Controle de Formação Baseado em Espaço Nulo
% ICUAS 2019
% Mauro Sérgio Mafra e Sara Jorge e Silva

%
% Referência da formação: Pionner
%
% Tarefa prioritária sendo controle de forma    (subíndice f) 
% Tarefa secundária sendo o controle de posição (subíndice p)
%
% Tarefa 1: qp = [rhof alphaf betaf]'   Jf(x): 3 últimas linhas
% Tarefa 2: qf = [xf yf zf]'            Jp(x): 3 primeiras linhas
% Projeção no espaço nulo da primeira tarefa: (I-(invJf*Jf))
% Sendo J(x) a Jacobiana da transformação direta e invJ sua pseudo inversa
% 
% Lei de Controle 1: qRefPontoi = qPontoi + Ki*qTili
% Lei de Controle 2: qRefPontoi = qPontoi + Li*tanh(pinv(Li)*Ki*qTili)
% Para formação desejada constante -> qPontoi = 0
%

%% Resetar 
clear all;   close all;
warning off; clc;

%% Início do Código
% Carrega o diretório corrente e subdiretórios 
addpath(genpath(pwd))

% Define o tempo de amostragem 
sampleTime = 1/30;    % Ref Viçosa
%sampleTime = 0.1;

% Define o tempo
t = 0:sampleTime:20; 

%% Inicializa as variáveis 
a = .2;        % Distância do ponto de interesse ao eixo virtual do Pioneer 
zp = 0;        % Posição z do Pioneer

pos    = zeros(3,length(t));                % Condições iniciais do Pioneer
vD     = zeros(2,length(t));
vRefPion   = zeros(2,length(t));
vRobot = zeros(2,length(t));
vErro  = zeros(2,length(t));


% Controlador Dinâmico DESATIVADO
ku = 0.4; 
lu = 0.2; 
kw = 0.4; 
lw = 0.2; 
theta = [0.2604 0.2509 -0.000499 0.9965 0.00263 1.07680];
H  = [theta(1)   0    ; ...
         0   theta(2)];

%% Inicio impressão do Pioneer
[x,y,psi,~,~,~,~] = leerRobot();
scrsz             = get(0,'ScreenSize');
off1              = 0;
off2              = 10;
figpos            = [off1 off2 scrsz(3)-off1 scrsz(4)-off2];

f1   = figure(1);
set(f1,'Color','w','Position',figpos);
paso = 2; axis 'equal'
Robot_Dimension(2);
Ho = Robot_Plot_3D(x,y,psi,'g'); hold on
H1 = plot(x,y,'*m'); hold on;
H2 = plot(x,y,'--r','LineWidth',2); hold on;

%% Condições iniciais Drone
x =  0  * ones(length(t),1);
y =  1  * ones(length(t),1);
z =  1  * ones(length(t),1);
psi2 = 0;                  % Psi2 - Angulo de Rotação no eixo Z

%% Plot inicial drone
H1d = plot3(x(1),y(1),z(1),'yo','LineWidth',2,'MarkerEdgeColor','k',...
        'MarkerFaceColor','y','MarkerSize',10);
H2d = plot3(x(1),y(1),z(1),'-g','LineWidth',2); hold on;

%% Objetivo da formação 
%  qDes = [xd yd zd rhod betad alphad];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qDes    = [0 0 0 1 pi/2 0]';    % O drone pára a 1 metro acima do Pioneer 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

qDesAnt = qDes;                 % Alterar quando trajetória variar no tempo
qDesp   = qDes(1:3,1);          
qDesf   = qDes(4:6,1);

% PI para plot
xRefPonto01(:,1) = zeros(6,1);
xRefPonto02(:,1) = zeros(6,1);
xTil(:,1)        = zeros(6,1);
for i = 2:length(t) 
    %% Leitura dos dados do pioneer
    % x1   = pos(1,i);
    % y1   = pos(2,i);
    % psi1 = pos(3,i);
    [pos(1,i),pos(2,i),pos(3,i),vRobot(1,i),vRobot(2,i),~,~] = leerRobot();    
    %% Transformaçao Direta - Variáveis de posição da formação
    % Variéveis de posição da formação
    xf = pos(1,i);      % xf = x1
    yf = pos(2,i);      % yf = y1     
    zf = zp;            % zf = z1 = 0 para Pioneer  
    
    % Variéveis de forma da formaçao P = atan2(Y,X)
    rhof   = sqrt ((x(i) - pos(1,i))^2 + (y(i) - pos(2,i))^2 + (z(i) - zp)^2);
    betaf  = atan2((z(i) - zp),sqrt((x(i) - pos(1,i))^2 + (y(i) - pos(2,i))^2));
    alphaf = atan2((y(i) - pos(2,i)),(x(i) - pos(1,i)));
    
    % Variáveis da formação
    q  = [xf yf zf rhof betaf alphaf]';
    qp = q(1:3,:);          % Controle de Posição
    qf = q(4:6,:);          % Controle de Forma
                   
    %% Cálculo do Erro - q Til
    % qTil Tarefa 01 
    qTilT01(:,i) = qDesf - qf;        % Vetor para plotagem no tempo
    qTilf = qDesf - qf;               % Salva valor atual
    
    % qTil Tarefa 02 
    qTilT02(:,i) = qDesp - qp;
    qTilp = qDesp - qp; 
    
    % Utilizado para Plot do erro das variaveis 
    qTil = [qTilT02;qTilT01];    
     
    %% Velocidade da formação ( qRef derivada no tempo ), há dois casos:
    % =  0 Quando qd é fixo 
    % <> 0 Quando qd varia no tempo
    qPonto  = (qDes - qDesAnt)./ sampleTime;    
    qDesAnt = qDes;
    
    qPontof = qPonto(4:6,:);
    qPontop = qPonto(1:3,:);
    
    %% Lei Controle da Formação
    % Saturação - Lei de controle quando usar tanh(.)
    Lp = 0.4 * eye(3);  
    Lf = 0.4 * eye(3);


    % Ganhos da Tarefa de posçao 
    Kp = 0.1 * eye(3);
%     Kp = [ 0.1  0   0   ; ...    % kx
%             0  0.1  0   ; ...    % ky
%             0   0  0.1 ]; ...   % kz
        
    % Ganhos da Tarefa de formação
    Kf = 0.1 * eye(3);
%     Kf = [ 0.1  0   0   ; ...    % krho
%             0  0.1  0   ; ...    % kbeta
%             0   0  0.01 ]; ...   % kalpha
          
    % Lei de controle para tarefa principal        
    % qpontoi = 0 para formação desejada constante no tempo
    qRefPontof = qPontof + Lf*tanh(pinv(Lf)*Kf*qTilf); 
%     qRefPontof = qPontof + Kf*qTilf; 
    
    % Lei de controle para tarefa secundaria
    qRefPontop = qPontop + Lp*tanh(pinv(Lp)*Kp*qTilp); 
%    qRefPontop = qPontop + Kp*qTilp; 

    %% Matriz Jacobiana Direta           
 
    % Linha 4 da Matriz Jacobiana
J41 = (pos(1,i) - x(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J42 = (pos(2,i) - y(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J43 = (zp - z(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J44 = -(pos(1,i) - x(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J45 = -(pos(2,i) - y(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J46 = -(zp - z(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
    % Linha 5 da Matriz Jacobiana
J51 = -(pos(2,i) - y(i)) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2);
J52 =  (pos(1,i) - x(i)) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2);
J53 =  0;
J54 =  (pos(2,i) - y(i)) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2);
J55 = -(pos(1,i) - x(i)) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2);
J56 =  0;
    % Linha 6 da Matriz Jacobiana
J61 =  ((2*pos(1,i) - 2*x(i))*(zp - z(i))) / sqrt(2*((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2)) * ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J62 =  ((2*pos(2,i) - 2*y(i))*(zp - z(i))) / sqrt(2*((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2)) * ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J63 = -sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J64 = -((2*pos(1,i) - 2*x(i))*(zp - z(i))) / sqrt(2*((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2)) * ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J65 = -((2*pos(1,i) - 2*x(i))*(zp - z(i))) / sqrt(2*((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2)) * ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
J66 =  sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);

Jacob = [ 1  ,  0  , 0  , 0  , 0  , 0 ;...
          0  ,  1  , 0  , 0  , 0  , 0 ;...
          0  ,  0  , 1  , 0  , 0  , 0 ;...
          J41, J42, J43, J44, J45, J46;...
          J51, J52, J53, J54, J55, J56;...
          J61, J62, J63, J64, J65, J66 ];
      
    
    %% NSB                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%     NSB    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Obtendo a Matriz identidade 6x6
    I = eye(6); 
     
    % Dividindo a Jacobiana para realização em duas tarefas 
    Jp = Jacob(1:3,:);
    Jf = Jacob(4:6,:);
       
    % Obtendo as Pseudo-inversas das Jacobianas de posição e de forma
    invJp = pinv(Jp);
    invJf = pinv(Jf);
 
      
    % Projetando a tarefa de menor prioridade no espaço nulo da tarefa de
    % maior prioridade    
    xRefPontof = invJf*qRefPontof;                          % Tarefa 1 f
    xRefPontop = ((I-(invJf*Jf))*invJp)*(qRefPontop);       % Tarefa 2 p  
    
    xRefPonto01(:,i) = invJf*qRefPontof;                      
    xRefPonto02(:,i) = ((I-(invJf*Jf))*invJp)*(qRefPontop);        
%     
%     xRefPontop = invJp*qRefPontop;                          % Tarefa 1 p
%     xRefPontof = ((I-(invJp*Jp))*invJf)*(qRefPontof);       % Tarefa 2 f  
%     
%     xRefPonto01(:,i) = invJf*qRefPontof;                      
%     xRefPonto02(:,i) = ((I-(invJf*Jf))*invJp)*(qRefPontop);        
 
    xRefPonto = xRefPontop + xRefPontof;
       
    %%%%%%%%%%%%%%%%%%%%%%%%%%%     NSB    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Matriz Cinemática Inversa
    % Cinemática Inversa do Pioneer 
    K1 = [  cos(pos(3,i))     sin(pos(3,i))      0  ; ...
           -sin(pos(3,i))/a   cos(pos(3,i))/a    0  ; ...
               0                    0            0 ];
    % Cinemática Inversa do Drone         
    K2 = [  cos(psi2)        -sin(psi2)          0  ; ...
            sin(psi2)         cos(psi2)          0  ; ...
               0                 0               1 ];
    
    K = [ K1        zeros(3);...
          zeros(3)  K2     ];       
    
    %% Controle de Velocidade 
        % Implementar controle com a cinemática inversa do drone
        % obter a posição desejada do drone
        % fazer controlador  K
%        Kpionn = K(1:3,i); 
        dxRefpion  = xRefPonto(1:3,1);
        
%        Kdrone = K(4:6,i);
        dxRefdron  = xRefPonto(4:6,1);
        
        % Pioneer
        x1des = qDes(1,1); 
        y1des = qDes(2,1); 
        z1des = qDes(3,1);
        xDespion = [x1des; y1des; z1des];
        
        % Posição atual
        x1 = pos(1,i);
        y1 = pos(2,i);
        z1 = 0;
        xpion = [x1 ; y1 ; z1];
        
        % Erro de Posição do Pioneer
        xTilpion = xDespion - xpion;
        
        % Drone
        %  qDes = [xd yd zd rhod betad alphad];
        % X2 = Xd + (rho*cos(beta)*cos(alpha)
        % Y2 = Yd + (rho*cos(beta)*cos(alpha))
        % Z2 = Zd + (rho*sen(beta))
        x2des = qDes(1,1) + ( qDes(4,1) * cos (qDes(5,1)) * cos (qDes(6,1)) ); 
        y2des = qDes(2,1) + ( qDes(4,1) * cos (qDes(5,1)) * sin (qDes(6,1)) ); 
        z2des = qDes(3,1) + ( qDes(4,1) * sin (qDes(5,1)) );
        
        xDesdron = [x2des ; y2des ; z2des];
        
        % Posição atual
        x2 = x(i);
        y2 = y(i);
        z2 = z(i);

        xdron = [x2 ; y2 ; z2];
        
        % Erro de Posição do Pioneer
        xTildron = xDesdron - xdron;

        % Erro Conjunto para Plot
        xTil(:,i) = [xTilpion ; xTildron] ; 
        
        % Velocidade controlada
        % Granho do Controlador ciemático
        kcp  = 0.3 * eye (3);
        kcd  = 0.5 * eye (3);
        
        % Lei de Controle                    ##### Orientação de Sarcinelli        
        % Velocidade desejada é proporcional à:
        %  1 - velocidade de referência +
        %  2 - ponderação do erro de posição 
        upion = dxRefpion + kcp * xTilpion ; 
        udron = dxRefdron + kcd * xTildron ; 
        
        % Velocidade para Pioneer e Drone
        v{i} = K * [upion ; udron];
    
          
    %% Plot em tempo real
    delete(Ho)
    axis([-1 6 -1 6 0 4]);  
    view([-40.0,20.0]);    
    Ho=Robot_Plot_3D(pos(1,i),pos(2,i),pos(3,i),'g');hold on
    
    H1.XData = pos(1,i);
    H1.YData = pos(2,i);
    H2.XData = pos(1,2:i);
    H2.YData = pos(2,2:i);hold on
    
    H1d.XData = x(i);
    H1d.YData = y(i);
    H1d.ZData = z(i);hold on
    H2d.XData = x(1:i);
    H2d.YData = y(1:i);
    H2d.ZData = z(1:i);
    grid on;
    
    %% Compensador Dinâmico - Status Atual: Desativado
    % % Pionner
    % Velocidades do pioneer a serem ajustadas no compensador dinamico
    vD(1,i) = v{i}(1);
    vD(2,i) = v{i}(2);
    
    Vd_ponto(:,i) = (vD(:,i)- vD(:,i-1)) ./ sampleTime;  
    
    vErr(:,i)     = vD(:,i) - vRobot(:,i);               % Cálculo de erro de velocidade do Drone
    
    C  = [        0                  -theta(3)*vRobot(2,i);...
          theta(3)*vRobot(2,i)                0          ];
      
    F  = [theta(4)                     0                     ;...
              0     theta(6)+(theta(5)-theta(3)*vRobot(1,i))];
          
    Tv = [lu   0 ;...
           0  lw]; 
    Tv = Tv * [tanh(vErr(1,i)*ku/lu);       tanh(vErr(2,i)*kw/lw)];
    
    %Lei Controle dinâmica
    vRefPion(:,i)= Tv + H*Vd_ponto(:,i) + C*vD(:,i) + F*vD(:,i);  
 
    % Envia a velocidade para o robô
%    escribirRobot(vRefPion(1,i),vRefPion(2,i));        % Para CD ativado
     escribirRobot(vD(1,i),vD(2,i));                    % Para CD desativado
    
    % % Drone
    % Velocidades do Drone a serem ajustadas no compensador dinamico
    vD(4,i) = v{i}(4);
    vD(5,i) = v{i}(5);
    vD(6,i) = v{i}(6);
  
    Vd_ponto(:,i) = (vD(:,i)- vD(:,i-1)) ./ sampleTime;  
    
    vErr(:,i)     = vD(:,i) - vRobot(:,i);               % Cálculo de erro de velocidade do Drone
    
    C  = [        0                  -theta(3)*vRobot(2,i);...
          theta(3)*vRobot(2,i)                0          ];
      
    F  = [theta(4)                     0                     ;...
              0     theta(6)+(theta(5)-theta(3)*vRobot(1,i))];
          
    Tv = [lu   0 ;...
           0  lw]; 
    Tv = Tv * [tanh(vErr(1,i)*ku/lu);       tanh(vErr(2,i)*kw/lw)];
    
    %Lei Controle dinâmica
    vRefPion(:,i)= Tv + H*Vd_ponto(:,i) + C*vD(:,i) + F*vD(:,i);  
 
    
    % Integra a velocidade do drone - Definindo Posição do Drone     
    x(i+1) = x(i) + sampleTime * vRefPion(:,i)(4);
    y(i+1) = y(i) + sampleTime * v{i}(5);
    z(i+1) = z(i) + sampleTime * v{i}(6);
    psi2   = 0;  

    %% Fim do loop
    %(tempo de amostragem)
    pause(sampleTime);
    % (Sara) Estabelecer critério de parada    
end

%% Comando de Parada do Pioneer
escribirRobot(0,0);

    %% Performance Criterion (Sugestão)
    % Erro de Formação          % trapz -> Trapezoidal numerical integration 

%% Plot do Erro de Formação
figure(2)
subplot(2,1,1); plot(t(2:end),qTil(:,2:end),'LineWidth',2);
legend('x_f','y_f','z_f', '\rho_f', '\beta_f','\alpha_f');
title('Erros das variáveis da formação');
xlabel('Tempo [s]'); ylabel('Erro');
subplot(2,1,2); plot(t(2:end),xTil(:,2:end),'LineWidth',2);
legend('x_1','y_1','z_1', 'x_2','y_2','z_2');
title('Erros das Posições 1: Pioneer e 2: Drone');
xlabel('Tempo [s]'); ylabel('Erro');
grid on

%% Plot xRefPonto1 e xRefPonto2  - Análise do NSB
figure(3)
hold on;  grid on;
subplot(2,1,1); plot(t(2:end),xRefPonto01(:,2:end),'LineWidth',2);
legend('dx1_r_e_f','dy1_r_e_f','dz1_r_e_f','dx2_r_e_f','dy2_r_e_f','dz2_r_e_f');
title('Análise dos sinais do NSB - Tarefa 1');
xlabel('Tempo [s]'); ylabel('Sinal de Controle');

subplot(2,1,2); plot(t(2:end),xRefPonto02(:,2:end),'LineWidth',2);
legend('x1ref_ponto','y1ref_ponto','z1ref_ponto','x2ref_ponto','y2ref_ponto','z2ref_ponto');
legend('dx1_r_e_f','dy1_r_e_f','dz1_r_e_f','dx2_r_e_f','dy2_r_e_f','dz2_r_e_f');
title('Análise dos sinais do NSB - Tarefa 2');
xlabel('Tempo [s]'); ylabel('Sinal de Controle');

%% Fim algoritmo