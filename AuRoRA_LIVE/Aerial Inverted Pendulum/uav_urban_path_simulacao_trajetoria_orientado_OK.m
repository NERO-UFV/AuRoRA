%% Simula��o do planejamento de trajet�ria para bambol�s (wayposes)
% - Implementar seguimento de caminho
% - 
%       - Destorcer curva para proteger bambol�s passados

% Boas pr�ticas
close all
clear
clc

% CONTROLAR TEMPO TOTAL DE SIMULA��O: (min = 100s)
T = 90;

try
    fclose(instrfindall);
    
    % Rotina para buscar pasta raiz
    PastaAtual = pwd;
    PastaRaiz = 'AuRoRA';
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
    
    disp('Diret�rio raiz e caminho bem definidos!')
    
catch
    disp('Diret�rio raiz errado!')
end

% =========================================================================
% Defini��o da cena:
%Diametro do drone real (63 cm)
dD = .63; 

% Bambol�s:
b1 =[1;1;1];     n1 = [-1;-0.5;1];
v1 = [1;0;1];    u1 = [0;1;0.5];


b2 = [3;-1;1];     n2 = [-1;-1;1].*-0.25;
v2 = [0.5;0;0.5];   u2 = [0;0.5;0.5];

b3 = [-2;-2;1.5];   n3 = [1;0;0];
v3 = [0;-1;0];      u3 = [0;0;1];


% Conjunto dos bambol�s:
Bmb = [b1,b2,b3]; Nb = [n1,n2,n3];
Vb = [v1,v2,v3];  Ub = [u1,u2,u3];

%% Plots ==================================================================
ps = [0;0;0]; % ponto inicial
CB = [];

figure(1)
axis([-3 4 -4 3 0 3])
view(100,42)
grid on

for i = 1:size(Nb,2)
    if i>1
%         alpha = atan2(pf(3),norm(pf(1:2)));
%         beta = atan2(pf(2),pf(1));      
%         ps = pfd -2*dD*n.*[cos(beta)*sin(alpha);sin(beta)*sin(alpha);cos(alpha)];
%         
        ps = [pfd-2*dD.*(n/norm(n)),pfd-3*dD.*(n/norm(n))]; 
        ControlPoints = [pfd,ps(:,1)];
        CB = [CB, DBezierCurve(ControlPoints)];
    end
    
    pf = Bmb(:,i);
    n  = Nb(:,i);
    V = Vb(:,i)/norm(Vb(:,i));
    U = Ub(:,i)/norm(Ub(:,i));
    alpha = atan2(pf(3),norm(pf(1:2)));
    beta = atan2(pf(2),pf(1));   
    
    % Deslocamento do ponto final:
    pfd = pf +dD.*(n/norm(n)); % ponto final deslocado por seguran�a   
%     n.*[cos(beta)*sin(alpha);sin(beta)*sin(alpha);cos(alpha)];
 
    dp = pfd-ps(:,1); % ponto inicial ao ponto deslocado

    % Defini��o Retas:
    t = -1.5:0.1:1.5;
    L = t.*n; % passa pelo bambol�
    R = t.*dp; % do drone ao bambol�
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

    % Normal ao bambol�:
    hold on
    plot3(L(1,:)+pfd(1),L(2,:)+pfd(2),L(3,:)+pfd(3),'b-.','LineWidth',1)

    % Do drone ao bambol�:
    hold on
    plot3([pfd(1) ps(1)],[pfd(2) ps(2)],[pfd(3) ps(3)],'r:.','LineWidth',1.2)

    % Bambol� 1: 
    u = 0:0.01:2*pi;
    XB = pf(1) +0.63.*cos(u).*V(1) +0.63*sin(u).*U(1);
    YB = pf(2) +0.63.*cos(u).*V(2) +0.63*sin(u).*U(2);
    ZB = pf(3) +0.63.*cos(u).*V(3) +0.63*sin(u).*U(3);
    hold on
    plot3(XB,YB,ZB,'LineWidth',2)
    grid on

    % Calcular e desenhar curva de B�zier:
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
%% Simula��o ==============================================================
A = ArDrone;

A.pPos.X = zeros(12,1);
XX = [];
flag1=1;
tmax = T; % Tempo Simula��o em segundos
X = zeros(1,19); % Dados correntes da simula��o
ControlPoints = [];
ps = [0;0;0]; % ponto inicial

k = size(CB,2);

dCB = [[0;0;0],diff(CB,1,2)];

Curva.psi = atan2(CB(2,:),CB(1,:));
Curva.dpsi = [0,diff(Curva.psi,1,2)];
Curva.X = CB;
Curva.dX = [[0;0;0],diff(CB,1,2)];
Curva.Pos = 1;
Curva.Size = k;
Curva.Vmax = 10;
Path.dXr = zeros(3,k);

psitil = [];

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
%         (Curva.Pos/Curva.Size)*100);
        
        A.pPos.Xd(1) = CB(1,itT);  
        A.pPos.Xd(2) = CB(2,itT);
        A.pPos.Xd(3) = CB(3,itT);
        A.pPos.Xd(6) = Curva.psi(itT);
        A.pPos.Xd(7) = dCB(1,itT);  
        A.pPos.Xd(8) = dCB(2,itT);
        A.pPos.Xd(9) = dCB(3,itT);
        A.pPos.Xd(6) = Curva.dpsi(itT);
            

        %             12        12      1   
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];

        % Controlador
        A.rGetSensorData
%         [A,Curva] = cPathFollowingSIM(A,Curva);
        A = cInverseDynamicController_Compensador_ArDrone(A);
        
%         if TT*100>80&&(Curva.Pos<600)
%             tmax=tmax+5;
%         end
        A.rSendControlSignals;
    end
end

clc
disp('Calculado!')
disp('Desenhando cena...')

%% Resimulando ============================================================
disp('Pronto para reproduzir.')
disp('Reproduzindo simula��o...')
hold on
plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',2)
grid on
for tr = 1:5:size(XX,2)
    A.pPos.X = XX([13:24],tr);
    
    A.mCADplot;
    drawnow
end
% =========================================================================
% tmax =   540
% tmax =   445
