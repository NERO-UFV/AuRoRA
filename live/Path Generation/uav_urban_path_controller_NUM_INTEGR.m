%% Simulação do planejamento de trajetória para bambolês (wayposes)
% - Implementar seguimento de caminho
% - Distorcer curva para proteger bambolês passados

% Boas práticas
close all
clear
clc

% CONTROLAR TEMPO TOTAL DE SIMULAÇÃO: (min = 100s)
T = 150;

try
%     fclose(instrfindall);
    
    % Rotina para buscar pasta raiz
    PastaAtual = pwd;
    PastaRaiz = 'AuRoRA 2018';
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
    
    disp('Diretório raiz e caminho bem definidos!')
    
catch ME
    disp('Diretório raiz errado!')
    disp(ME)
end

% =========================================================================
% Definição da cena:
%Diametro do A real (63 cm)
dD = .63; 

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
ps = [0;0;0]; % ponto inicial
CB = [];
dCB = [];
CP2 =[];

figure(1)
axis([-4 4 -4 3 0 3])
view(100,42)
grid on

N = 250;

for i = 1:size(Nb,2)
    if i>1
%         alpha = atan2(pf(3),norm(pf(1:2)));
%         beta = atan2(pf(2),pf(1));      
%         ps = pfd -2*dD*n.*[cos(beta)*sin(alpha);sin(beta)*sin(alpha);cos(alpha)];
%         

        % Reta que atravessa o bambolê
        ps = [pfd-2*dD.*(n/norm(n)),pfd-3*dD.*(n/norm(n))]; 
        ControlPoints = [pfd,ps(:,1)];
        CP2 = [CP2,ControlPoints];
        time = linspace(0,1,N);
        CB = [CB, DBezierCurve(ControlPoints,time)];
        dCB = [dCB, DerivBezierCurve(ControlPoints,time)];
    end
    
    pf = Bmb(:,i);
    n  = Nb(:,i);
    V = Vb(:,i)/norm(Vb(:,i));
    U = Ub(:,i)/norm(Ub(:,i));
%     alpha = atan2(pf(3),norm(pf(1:2)));
%     beta = atan2(pf(2),pf(1));   
     
    % Deslocamento do ponto final:
    pfd = pf +dD.*(n/norm(n)); % ponto final deslocado por segurança   
%     n.*[cos(beta)*sin(alpha);sin(beta)*sin(alpha);cos(alpha)];
 
    dp = pfd-ps(:,1); % ponto inicial ao ponto deslocado

    % Definição Retas:
    t = -1.5:0.1:1.5;
    L = t.*n; % passa pelo bambolê
    R = t.*dp; % do A ao bambolê
    z = t.*cross(R,L)+pfd; % normal ao plano

    % Ponto de controle:
    idx = find(abs(n)==max(abs(n)));
    pc = pfd + n.*dD;

    % Desenho dos pontos e retas ==========================================
    % Ponto inicial:                  (PRETO)
    hold on
    plot3(ps(1,:),ps(2,:),ps(3,:),'ko')
    hold on
    plot3(ps(1,:),ps(2,:),ps(3,:),'k*')

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
%     hold on
%     plot3(pc2(1),pc2(2),pc2(3),'b*')
%     hold on
%     plot3(pc2(1),pc2(2),pc2(3),'bo')

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
    ControlPoints = [ps,pc,pfd];
    CP2 = [CP2,ControlPoints];
    time = linspace(0,1,N);
    CB = [CB, DBezierCurve(ControlPoints,time)];
    dCB = [dCB, DerivBezierCurve(ControlPoints,time)];

    % Pontos de Controle:
    hold on
    plot3(ControlPoints(1,:),ControlPoints(2,:),...
          ControlPoints(3,:),'r:.','LineWidth',1.2)
      
    if i==size(Nb,2)
        ControlPoints = [pfd,pfd-2*dD.*n];
        CP2 = [CP2,ControlPoints];
        time = linspace(0,1,N);
        CB = [CB, DBezierCurve(ControlPoints,time)];
        dCB = [dCB, DerivBezierCurve(ControlPoints,time)];
    end
end
time= linspace(0,1,size(CB,2));
CB2 = DBezierCurve(CP2,time);
dCB2 = DerivBezierCurve(CP2,time);

hold on
plot3(CB(1,:),CB(2,:),CB(3,:),'b--','LineWidth',2.5)
hold on
plot3(CB2(1,:),CB2(2,:),CB2(3,:),'r--','LineWidth',2.5)
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

dCB = [[0;0;0],diff(CB,1,2)];

Curva.rho = 0;
Curva.psi = atan2(CB(2,:),CB(1,:));
Curva.dpsi = [0,diff(Curva.psi,1,2)];
Curva.X = CB;
Curva.dX = dCB2;
% [[0;0;0],diff(CB,1,2)];
Curva.ddX = [[0;0;0],diff(Curva.dX,1,2)];
Curva.Pos = 1;
Curva.Size = k;
Path.dXr = zeros(3,k);

krv = abs(cross(Curva.dX,Curva.ddX))./(abs(Curva.dX).^3);

% Curva.Vmax = 1/10; % => 120 seg
Curva.Vmax = 1/15;

t = tic;
tc = tic;
tp = tic;
TT = 0;

while (toc(t) < tmax) && Curva.Pos ~= Curva.Size
    if toc(tc) > 1/30
        tc = tic;
        tt = toc(t);
        TT = (tt/tmax);
        itT = floor(k*TT)+1;     
        
        clc
        fprintf('Calculando: %0.2g%% \n',(Curva.Pos/Curva.Size)*100);
% 
%         A.pPos.Xd(1) = CB(1,itT);     
%         A.pPos.Xd(2) = CB(2,itT);
%         A.pPos.Xd(3) = CB(3,itT);
%         A.pPos.Xd(6) = Curva.psi(itT);
%         A.pPos.Xd(7) = dCB(1,itT);  
%         A.pPos.Xd(8) = dCB(2,itT);
%         A.pPos.Xd(9) = dCB(3,itT);
%         A.pPos.Xd(6) = Curva.dpsi(itT);
        
        %             12        12      1   
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];

        % Controlador
        A.rGetSensorData
        [A,Curva] = cPathFollowing(A,Curva);
%         A.pSC.Kinematics_control = 1;
        Curva.Pos
        A.pPos.Xr(1:3) = A.pPos.Xd(1:3);
        A.pPos.Xr(6) = A.pPos.Xd(6);
        A.pPos.Xr(7:9) = A.pPos.Xd(7:9);
        
        A = cInverseDynamicController_Compensador_ArDrone(A);
        
        A.rSendControlSignals

%         ArDrone pose - Numerical integration
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

    if toc(tp) > 0.3
        tp = tic;
        A.mCADplot;
        drawnow
    end
    end
end

clc
disp('Calculado!')
disp('Desenhando cena...')

%% Resimulando ============================================================
disp('Pronto para reproduzir.')
disp('Reproduzindo simulação...')
hold on
plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',2)
grid on
for tr = 1:5:size(XX,2)
    A.pPos.X = XX([13:24],tr);
    
    A.mCADplot;
    drawnow
end
% =========================================================================
% tmax = 540;
% tmax = 445;
% tmax = 139;

% figure
% plot(1:size(Curva.dX,2)-1,Curva.dX(1,1:end-1)-XX(19,:),'r--')
% title('Erro dX')

%%

psi1 = atan2(dCB(2,:),dCB(1,:));
psi2 = atan2(dCB2(2,:),dCB2(1,:));
figure
plot(1:size(psi1,2),psi1,'b-')
hold on
plot(1:size(psi2,2),psi2,'r--')

% figure
% plot(1:size(CB2,2),CB2(1,:),'r--')
% hold on
% plot(1:size(CB,2),CB(1,:),'b-')
% title('CB 1 e 2 em X')
% 
% figure
% plot(1:size(CB2,2),CB2(2,:),'r--')
% hold on
% plot(1:size(CB,2),CB(2,:),'b-')
% title('CB 1 e 2 em Y')
% 
% figure
% plot(1:size(CB2,2),CB2(3,:),'r--')
% hold on
% plot(1:size(CB,2),CB(3,:),'b-')
% title('CB 1 e 2 em Z')
% 
% figure
% plot(1:size(dCB2,2),CB2(1,:),'r--')
% hold on
% plot(1:size(dCB,2),CB(1,:),'b-')
% title('dCB 1 e 2 em X')
% 
% figure
% plot(1:size(dCB2,2),dCB2(2,:),'r--')
% hold on
% plot(1:size(dCB,2),dCB(2,:),'b-')
% title('dCB 1 e 2 em Y')
% 
% figure
% plot(1:size(dCB2,2),dCB2(3,:),'r--')
% hold on
% plot(1:size(dCB,2),dCB(3,:),'b-')
% title('dCB 1 e 2 em Z')


