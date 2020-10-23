%% Simulação do planejamento de trajetória para bambolês (wayposes)
% - Implementar seguimento de caminho
% - 
% - Destorcer curva para proteger bambolês passados

% Boas práticas
close all
clear
clc

% CONTROLAR TEMPO TOTAL DE SIMULAÇÃO: (min = 100s)
T = 150;

% Rotina para buscar pasta raiz
try
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
%Diametro do A real (63 cm)
dD = .63; 

% Bambolês:
% b1 =[1;1;1];     n1 = [-1;-0.5;1];
% v1 = [1;0;1];    u1 = [0;1;0.5];
b1 =[-2;-2;1];     n1 = [-1;0;0];
v1 = [0;0;1];    u1 = [0;1;0];

% b2 = [3;-1;1];     n2 = [-1;-1;1].*-0.25;
% v2 = [0.5;0;0.5];   u2 = [0;0.5;0.5];
b2 = [2;-2;1];     n2 = [-1;0;0];
v2 = [0.5;0;0.5];   u2 = [0;0.5;0.5];

b3 = [1;0;1]; 

% Conjunto dos bambolês:
% Bmb = [b1,b2,b3]; Nb = [n1,n2,n3];
% Vb = [v1,v2,v3];  Ub = [u1,u2,u3];

Bmb = [b1,b2,b3]; Nb = [n1,n2,-n1];
Vb = [v1,v1,b1];  Ub = [u1,u1,u1];

%% Plots ==================================================================
ps = [-2;0;1]; % ponto inicial
CB = [];

figure(1)
axis([-4 4 -4 4 0 3])
view(100,42)
grid on

N = 500;
for i = 1:size(Nb,2)
    if i>1    
        ps = [pfd-2*dD.*(n/norm(n)),pfd-3*dD.*(n/norm(n))]; 
        ControlPoints = [pfd,ps(:,1)];
        
        time = linspace(0,1,N);
        
        CB = [CB, DBezierCurve(ControlPoints,time)];
    end
    
    pf = Bmb(:,i);
    n  = Nb(:,i);
    V = Vb(:,i)/norm(Vb(:,i));
    U = Ub(:,i)/norm(Ub(:,i));
     
    % Deslocamento do ponto final:
    pfd = pf +dD.*(n/norm(n)); % ponto final deslocado por segurança   
    dp = pfd-ps(:,1); % ponto inicial ao ponto deslocado

    % Definição Retas:
    t = -1.5:0.1:1.5;
    L = t.*n; % passa pelo bambolê
    R = t.*dp; % do A ao bambolê
    z = t.*cross(R,L)+pfd; % normal ao plano

    % Ponto de controle:
    idx = find(abs(n)==max(abs(n)));
    pc = pfd + n.*dD;
    
    psd = ps - dD.*(ps./norm(n));

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

    % Do A ao bambolê:
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
    ControlPoints = [ps,psd,pc,pfd];
    time = linspace(0,1,N);
    CB = [CB, DBezierCurve(ControlPoints,time)];

    % Pontos de Controle:
    hold on
    plot3(ControlPoints(1,:),ControlPoints(2,:),...
          ControlPoints(3,:),'r:.','LineWidth',1.2)
      
    if i==size(Nb,2)
        ControlPoints = [pfd,pfd-2*dD.*n];
        time = linspace(0,1,N);
        CB = [CB, DBezierCurve(ControlPoints,time)];
    end
end

% ControlPoints = [CB(:,end),CB(:,end),CB(:,1)];
% time = linspace(0,1,N);
% CB = [CB, DBezierCurve(ControlPoints,time)];

hold on
plot3(CB(1,:),CB(2,:),CB(3,:),'b--','LineWidth',2.5)
%% Simulação ==============================================================
A = ArDrone;

A.pPos.X = zeros(12,1);
XX = [];
tmax = T; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação
ControlPoints = [];
ps = [0;0;0]; % ponto inicial

k = size(CB,2);

dCB = [[0;0;0],diff(CB,1,2)];

Curva.rho = 0;
Curva.psi = atan2(CB(2,:),CB(1,:));
Curva.dpsi = [0,diff(Curva.psi,1,2)];
Curva.X = CB;
Curva.dX = [[0;0;0],diff(CB,1,2)];
Curva.Pos = 1;
Curva.Size = k;
Path.dXr = zeros(3,k);

% Curva.Vmax = 1/10; % => 120 seg
Curva.Vmax = 0.07;

t = tic;
tc = tic;
tp = tic;
TT = 0;

lap = 0;
c = 0;

while lap <= 1
% while Curva.Pos < Curva.Size    
    if toc(tc) > A.pPar.Ts
        tc = tic;
        tt = toc(t);
        TT = (tt/tmax);
        itT = floor(k*TT)+1;     
        
        clc
        fprintf('Percorrendo: %0.2g%% \n',(Curva.Pos/Curva.Size)*100);
        
        %             12        12      1   
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];

        A.rGetSensorData

        % Controlador
        [A,Curva] = cPathFollowing(A,Curva);
        Curva.Pos
        A.pPos.Xr(1:3) = A.pPos.Xd(1:3);
        A.pPos.Xr(6) = A.pPos.Xd(6);    
        A.pPos.Xr(7:9) = A.pPos.Xd(7:9);


        if norm(A.pPos.X(1:3)-Curva.X(1:3,Curva.Pos))>0.2
            A.pSC.Kinematics_control = 0;
            A = cInverseDynamicController_Compensador_ArDrone(A);
            c= c+1;
        else
            A.pSC.Kinematics_control = 1;
            A = cInverseDynamicController_Compensador_ArDrone(A);
        end
                
        norm(Curva.X(1:3,Curva.Pos)-A.pPos.X(1:3))

        if Curva.Pos==Curva.Size
%             Curva.Pos = 1;
            lap = lap+1; disp(lap)
        end

        A.rSendControlSignals
    end
end

tmax = toc(t);

clc
disp('Calculado!')
disp('Desenhando cena...')

%% Resimulando ============================================================
disp('Pronto para reproduzir.')
disp('Reproduzindo simulação...')
hold on
plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',2)
grid on
% g1 = gca;
% figure(2)
% g2 = gca;

for tr = 1:5:size(XX,2)
    A.pPos.X = XX([13:24],tr);

%     axes(g2)
%         hold on
%         u=sqrt(XX(19,1:tr).^2 +XX(20,1:tr).^2 +XX(21,1:tr).^2);
%         plot(XX(25,1:tr),u,'k')
%     axes(g1)

    A.mCADplot;
    drawnow
end
% =========================================================================
% tmax = 540;
% tmax = 445;
% tmax = 139;

%% original
% for i = 1:size(Nb,2)
%     if i>1    
%         ps = [pfd-2*dD.*(n/norm(n)),pfd-3*dD.*(n/norm(n))]; 
%         ControlPoints = [pfd,ps(:,2)];
%         N = 100;
%         time = linspace(0,1,N);
%         CB = [CB, DBezierCurve(ControlPoints,time)];
%     end
%     
%     pf = Bmb(:,i);
%     n  = Nb(:,i);
%     V = Vb(:,i)/norm(Vb(:,i));
%     U = Ub(:,i)/norm(Ub(:,i));
%      
%     % Deslocamento do ponto final:
%     pfd = pf +dD.*(n/norm(n)); % ponto final deslocado por segurança   
%     dp = pfd-ps(:,1); % ponto inicial ao ponto deslocado
%     
%     psd = ps - dD.*(ps/norm(ps));
% 
%     % Definição Retas:
%     t = -1.5:0.1:1.5;
%     L = t.*n; % passa pelo bambolê
%     R = t.*dp; % do A ao bambolê
%     z = t.*cross(R,L)+pfd; % normal ao plano
% 
%     % Ponto de controle:
%     idx = find(abs(n)==max(abs(n)));
%     pc = pfd + n.*dD;
% 
%     % Desenho dos pontos e retas ==========================================
%     % Ponto inicial:                  (PRETO)
%     hold on
%     plot3(ps(1),ps(2),ps(3),'ko')
%     hold on
%     plot3(ps(1),ps(2),ps(3),'k*')
% 
%     % Ponto final:                  (VERMELHO)
%     hold on
%     plot3(pf(1),pf(2),pf(3),'ro')
%     hold on
%     plot3(pf(1),pf(2),pf(3),'r*')
% 
% 
%     % Ponto final deslocado:        (VERMELHO)
%     hold on
%     plot3(pfd(1),pfd(2),pfd(3),'ro')
%     hold on
%     plot3(pfd(1),pfd(2),pfd(3),'r*')
% 
%     % Ponto de controle:              (AZUL)
%     hold on
%     plot3(pc(1),pc(2),pc(3),'b*')
%     hold on
%     plot3(pc(1),pc(2),pc(3),'bo')
% 
%     % Normal ao bambolê:
%     hold on
%     plot3(L(1,:)+pfd(1),L(2,:)+pfd(2),L(3,:)+pfd(3),'b-.','LineWidth',1)
% 
%     % Do drone ao bambolê:
%     hold on
%     plot3([pfd(1) ps(1)],[pfd(2) ps(2)],[pfd(3) ps(3)],'r:.','LineWidth',1.2)
% 
%     % Bambolê 1: 
%     u = 0:0.01:2*pi;
%     XB = pf(1) +0.63.*cos(u).*V(1) +0.63*sin(u).*U(1);
%     YB = pf(2) +0.63.*cos(u).*V(2) +0.63*sin(u).*U(2);
%     ZB = pf(3) +0.63.*cos(u).*V(3) +0.63*sin(u).*U(3);
%     hold on
%     plot3(XB,YB,ZB,'LineWidth',2)
%     grid on
% 
%     % Calcular e desenhar curva de Bèzier:
%     ControlPoints = [ps,psd,pc,pfd];
%     time = linspace(0,1,N);
%     CB = [CB, DBezierCurve(ControlPoints,time)];
% 
%     % Pontos de Controle:
%     hold on
%     plot3(ControlPoints(1,:),ControlPoints(2,:),...
%           ControlPoints(3,:),'r:.','LineWidth',1.2)
%       
%     if i==size(Nb,2)
%         ControlPoints = [pfd,pfd-2*dD.*n];
%         N = 100;
%         time = linspace(0,1,N);
%         CB = [CB, DBezierCurve(ControlPoints,time)];
%     end
% end
% 

figure
title('Xtil vs t')
u1=sqrt((XX(1,:)-XX(13,:)).^2 +(XX(2,:)-XX(14,:)).^2 +(XX(3,:)-XX(15,:)).^2);
plot(XX(25,:),u1,'k-')
hold on
plot(XX(25,:),0.08.*ones(1,size(XX,2)),'r--')

% figure
% title('Vel X')
% plot(XX(25,:),XX(13,:),'g')
% hold on
% plot(XX(25,:),XX(1,:),'r')
% hold on
% % plot(XX(25,:),XX(24,:),'r--')
% hold on
% u=sqrt(XX(19,:).^2 +XX(20,:).^2 +XX(21,:).^2);
% plot(XX(25,:),u,'k')
% legend('X','Xd','||u||')
% xlabel('t [s]')
% ylabel('u [m/s]')

%         A.pSC.U = A.pSC.Ud;
%         A.pPos.X ([4 5 9 12]) = A.pSC.U;
%         % ArDrone pose - Numerical integration
%         for ii = 1:6
%             A.pPos.X(ii) = A.pPos.X(ii+6)*A.pPar.Ts + A.pPos.X(ii);
%             if ii > 3
%                 if A.pPos.X(ii) > pi
%                     A.pPos.X(ii) = -2*pi + A.pPos.X(ii);
%                 end
%                 if A.pPos.X(ii) < -pi
%                     A.pPos.X(ii) = 2*pi + A.pPos.X(ii);
%                 end
%             end
%         end

