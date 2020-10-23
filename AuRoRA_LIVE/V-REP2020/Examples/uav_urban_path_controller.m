%% Simulação do planejamento de trajetória para bambolês (wayposes)
% - Implementar seguimento de caminho
% - Distorcer curva para proteger bambolês passados

% Boas práticas
close all
clear
clc

% CONTROLAR TEMPO TOTAL DE SIMULAÇÃO: (min = 100s)
T = 90;

try
    % Rotina para buscar pasta raiz
    PastaAtual = pwd;
    PastaRaiz = 'AuRoRA 2018';
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
    
    disp('Diretório raiz e caminho bem definidos!')
    
catch
    disp('Diretório raiz errado!')
end

% =========================================================================
% Definição da cena:
figure(1)
axis([-3 4 -4 3 0 3])
view(100,42)
grid on

%Diametro do drone real (63 cm)
dD = 1; 

% Bambolês:
b1 =[1;1;1];     n1 = [-1;-0.5;1]; 
v1 = [1;0;1];    u1 = [0;1;0.5];


b2 = [3;-1;1];     n2 = [-1;-1;1].*-0.25;
v2 = [0.5;0;0.5];   u2 = [0;0.5;0.5];

b3 = [-2;-2;1.5];   n3 = [1;0;0];
v3 = [0;-1;0];      u3 = [0;0;1];


% Conjunto dos bambolês:
Bmb = [b1,b2,b3]; Nb = [n1,n2,n3];
Vb = [v1,v2,v3];  Ub = [u1,u2,u3];

%% Plots ==================================================================
% Pontos:
ps = [0;0;0]; % ponto inicial
CB = [];

for i = 1:size(Nb,2)
    if i>1
        ControlPoints = [pfd,pfd-2*dD.*n];
        CB = [CB, DBezierCurve(ControlPoints)];
             
        ps =pf-sign(pf).*dD.*n;        
    end
    
    pf = Bmb(:,i);
    n  = Nb(:,i);
    V = Vb(:,i)/norm(Vb(:,i));
    U = Ub(:,i)/norm(Ub(:,i));

    % Deslocamento do ponto:
    pfd = pf + dD.*n; % ponto final deslocado por segurança

    dp = pfd-ps; % ponto inicial ao ponto deslocado

    % Definição Retas:
    t = -1.5:0.1:1.5;
    L = t.*n; % passa pelo bambolê
    R = t.*dp; % do drone ao bambolê
    z = t.*cross(R,L)+pfd; % normal ao plano

    % Ponto de controle:
    idx = find(abs(n)==max(abs(n)));
    pc = pfd + n.*dD;

    % Desenho dos pontos e retas ==========================================
    % Ponto inicial:                  (PRETO)
    hold on
    plot3(ps(1),ps(2),ps(3),'ko')
    hold on
    plot3(ps(1),ps(2),ps(3),'k*')

    % Ponto final:                  (VERMELHO)
    hold on
    plot3(pf(1),pf(2),pf(3),'ro')
    hold on
    plot3(pf(1),pf(2),pf(3),'r*')


    % Ponto final deslocado:        (VERMELHO)
    hold on
    plot3(pfd(1),pfd(2),pfd(3),'ro')
    hold on
    plot3(pfd(1),pfd(2),pfd(3),'r*')

    % Ponto de controle:              (AZUL)
    hold on
    plot3(pc(1),pc(2),pc(3),'b*')
    hold on
    plot3(pc(1),pc(2),pc(3),'bo')

    % Normal ao bambolê:
    hold on
    plot3(L(1,:)+pfd(1),L(2,:)+pfd(2),L(3,:)+pfd(3),'b-.','LineWidth',1)

    % Do drone ao bambolê:
    hold on
    plot3([pfd(1) ps(1)],[pfd(2) ps(2)],[pfd(3) ps(3)],'r:.','LineWidth',1.2)

    % Bambolê 1: 
    u = 0:0.01:2*pi;
    XB = pf(1) +0.63.*cos(u).*V(1) +0.63*sin(u).*U(1);
    YB = pf(2) +0.63.*cos(u).*V(2) +0.63*sin(u).*U(2);
    ZB = pf(3) +0.63.*cos(u).*V(3) +0.63*sin(u).*U(3);
    hold on
    plot3(XB,YB,ZB,'LineWidth',2)
    grid on

    % Calcular e desenhar curva de Bèzier:
    ControlPoints = [ps,pc,pfd];
    CB = [CB, DBezierCurve(ControlPoints)];

    % Pontos de Controle:
    hold on
    plot3(ControlPoints(1,:),ControlPoints(2,:),...
          ControlPoints(3,:),'r:.','LineWidth',1.2)
      
    if i==size(Nb,2)
        ControlPoints = [pfd,pfd-2*dD.*n];
        CB = [CB, DBezierCurve(ControlPoints)];
    end
end

hold on
plot3(CB(1,:),CB(2,:),CB(3,:),'b--','LineWidth',2.5)
%% Simulação ==============================================================
A = ArDrone;

A.pPos.X = zeros(12,1);
XX = [];
flag1=1;
tmax = T; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação
ControlPoints = [];
ps = [0;0;0]; % ponto inicial

k = size(CB,2);

Curva.X = CB;
Curva.dX = [[0;0;0],diff(CB,1,2)];
Curva.Pos = 1;
Curva.Size = k;
Curva.Vmax = 10;
Path.dXr = zeros(3,k);

t = tic;
tc = tic;
tp = tic;
TT = 0;

while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        tt = toc(t);
        TT = (tt/tmax);
        itT = floor(k*TT)+1;     
        
        clc
        fprintf('Calculando: %0.2g%% \n',TT*100);
        
%         A.pPos.Xd(1) = CB(1,itT);  
%         A.pPos.Xd(2) = CB(2,itT);
%         A.pPos.Xd(3) = CB(3,itT);
%         A.pPos.Xd(6) = atan2(CB(2,itT),CB(1,itT));

        %             12        12      1
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];

        A.pPos.Xd(3)
        % Controlador
        A.rGetSensorData
        [A,Curva] = cPathFollowing(A,Curva);  
        A.pPos.Xd(3)
        A.rSendControlSignals;
    end
end
%% Resimulando ============================================================
clc
disp('Calculado!')
disp('Desenhando cena...')
disp('Reproduzindo simulação...')
hold on
plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',2)
grid on
for tr = 1:12:size(XX,2)
    A.pPos.X = XX([13:24],tr);
    
    A.mCADplot;
    drawnow
end
% =========================================================================


