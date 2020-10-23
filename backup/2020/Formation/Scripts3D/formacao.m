%% Tarefa 1

% Limpa as variávies
clear all; 
close all;
warning off;
clc;

% Carrega o diretório corrente e subdiretórios 
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))
addpath(genpath(pwd))

% teste plotar modelo drone bonitão
A = ArDrone;
figure;

% Define o tempo de amostragem (100ms)
sampleTime = 0.1;

% Define o tempo
t = 0:sampleTime:20; 

% Inicializa as variáveis 
a = .2;
ku=0.4; %Dinamica
lu=0.2; %Dinamica
kw=0.4; %Dinamica
lw=0.2; %Dinamica
theta=[0.2604 0.2509 -0.000499 0.9965 0.00263 1.07680];
H=[theta(1) 0;0 theta(2)];

pos = zeros(3,length(t));
vD = zeros(2,length(t));
vRef = zeros(2,length(t));
vRobot = zeros(2,length(t));
vErro = zeros(2,length(t));
aux = 1;

u_robot=[0 0];
u_ref=[0 0];
velantX=0;velantY=0;

%inicio impressão do pioneer
[x,y,psi,~,~,~,~] = leerRobot();
scrsz=get(0,'ScreenSize');
off1=0;
off2=10;
figpos=[off1 off2 scrsz(3)-off1 scrsz(4)-off2];

f1=figure(1);
set(f1,'Color','w','Position',figpos);
paso=2; axis 'equal'
Robot_Dimension(2);
Ho=Robot_Plot_3D(x,y,psi,'g'); hold on
H1 = plot(x,y,'*m'); hold on;
H2 = plot(x,y,'--r','LineWidth',2); hold on;

%Condições iniciais
x = -2*ones(length(t),1);
y = 1*ones(length(t),1);
z = 1*ones(length(t),1);
phi = zeros(length(t),1);
phid = 0;

%plot inicial drone
H1d = plot3(x(1),y(1),z(1),'yo','LineWidth',2,'MarkerEdgeColor','k',...
        'MarkerFaceColor','y','MarkerSize',10);
H2d = plot3(x(1),y(1),z(1),'-g','LineWidth',2); hold on;

%Objetivo da formacao
qDes = [2 0 0 0.5 pi/2 0]';

for i = 2:length(t)
    
    % Leitura dos dados do pioneer
    [pos(1,i),pos(2,i),pos(3,i),vRobot(1,i),vRobot(2,i),~,~] = leerRobot();
    
    %controlador da formacao
    xf = pos(1,i);
    yf = pos(2,i);
    zf = 0;
    rhof = sqrt((x(i) - pos(1,i))^2 + (y(i) - pos(2,i))^2 + (z(i)- 0)^2);
    alphaf = atan2((y(i) - pos(2,i)),(x(i) - pos(1,i)));
    betaf = atan2((z(i) - 0),sqrt((x(i) - pos(1,i))^2 + (y(i) - pos(2,i))^2));
    q = [xf yf zf rhof betaf alphaf];
    qTil(:,i) = qDes - q';
    
    %ganhos
    L  = .2*eye(6);
    kp = .4*eye(6);
    
    qRefPonto = L*tanh(inv(L)*kp*qTil(:,i));
    
    jacob = [ 1, 0, 0, 0, 0, 0; ...
              0, 1, 0, 0, 0, 0; ...
              0, 0, 1, 0, 0, 0; ...
              1, 0, 0, cos(alphaf)*cos(betaf), -rhof*cos(alphaf)*sin(betaf), -rhof*cos(betaf)*sin(alphaf); ...
              0, 1, 0, cos(betaf)*sin(alphaf), -rhof*sin(alphaf)*sin(betaf),  rhof*cos(alphaf)*cos(betaf); ...
              0, 0, 1, sin(betaf), rhof*cos(betaf), 0];
          
    xRefPonto = jacob*qRefPonto;
    
    K = [cos(pos(3,i)) sin(pos(3,i)) 0 0 0 0; ...
         -sin(pos(3,i))/a cos(pos(3,i))/a 0 0 0 0; ...
         0 0 1 0 0 0; ...
         0 0 0 cos(phid) -sin(phid) 0; ...
         0 0 0 sin(phid) cos(phid) 0; ...
         0 0 0 0 0 1];
     
    v{i} = K*xRefPonto;
       
    
    % teste plot drone
    A.pPos.X(1:3) = [x(i);y(i);z(i)];  % popula posições drone
    A.mCADplot;
    
    %impressão atualizada
    delete(Ho)
    axis([-4 4 -4 4 0 4]);  
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
    
    %velocidades do pioneer a serem ajustadas no compensador dinamico
    vD(1,i) = v{i}(1);
    vD(2,i) = v{i}(2);
    
    %Integra a velocidade do drone
    x(i+1) = x(i)+sampleTime*v{i}(4);
    y(i+1) = y(i)+sampleTime*v{i}(5);
    z(i+1) = z(i)+sampleTime*v{i}(6);
    phid = 0;
    
    %Controlador dinâmico do Pioneer
    Vd_ponto(:,i)=(vD(:,i)-vD(:,i-1))/sampleTime;
    
    vErr(:,i) = vD(:,i) - vRobot(:,i); % cálculo erros de velocidade
    
    C=[0 -theta(3)*vRobot(2,i);theta(3)*vRobot(2,i) 0];
    F=[theta(4) 0;0 theta(6)+(theta(5)-theta(3)*vRobot(1,i))];
    T_v=[lu 0;0 lw]*[tanh(vErr(1,i)*ku/lu);tanh(vErr(2,i)*kw/lw)];
    
    vRef(:,i)= T_v + H*Vd_ponto(:,i) + C*vD(:,i) + F*vD(:,i);  %Lei Controle dinâmica
    
  % Envia a velocidade para o robô
    escribirRobot(vRef(1,i),vRef(2,i));
    
    %(tempo de amostragem)
    pause(sampleTime);
end

% Ao final, para o robô
escribirRobot(0,0);

figure(2)
plot(t(2:end),qTil(1:2,2:end),'LineWidth',2);hold on;
plot(t(2:end),qTil(4:5,2:end),'LineWidth',2)
grid on
legend('Xf','Yf', 'rhof', 'Ângulo');
title('Erros na formação');
xlabel('tempo [s]');ylabel('Erro');

