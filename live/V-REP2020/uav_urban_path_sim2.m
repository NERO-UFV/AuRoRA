%% Simulação do planejamento de trajetória para bambolês (wayposes)
% - Implementar seguimento de caminho
% - Analisar velocidade/curvatura no tempo, se não tiver ok corrigir
% - Distorcer curva para proteger bambolês passados

% Boas práticas
close all
clear
clc

% CONTROLAR TEMPO TOTAL DE SIMULAÇÃO: (min = 100s)
T = 100;
% 150;

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
%Diametro do A real (63 cm)
dD = 1;
% .63; 

% Bambolês:
b1 =[1,1,1.2];     n1 = [-1,-0.5,1];
v1 = [1,0,1];    u1 = [0,1,0.5];


b2 = [3,-1,1.2];     n2 = [-1,-1,1].*-0.25;
v2 = [0.5,0,0.5];   u2 = [0,0.5,0.5];

b3 = [-2,-2,1.7];   n3 = [1,0,0];
v3 = [0,-1,0];      u3 = [0,0,1];

% Sem optitrack
% b4 = [0;0;1.35]; n4 = [0;1;0];
% v4 = [1;0;0];   u4 = [0;0;1];

% Conjunto dos bambolês:
Bmb = [b1;b2;b3]; Nb = [n1;n2;n3];
Vb = [v1;v2;v3];  Ub = [u1;u2;u3];

%% Plots ==================================================================
p0 = [0,0,.75]; % ponto inicial
n0 = [1,0,0];
ps = p0;
CB = [];
na =[];

figure(1)
axis([-4 4 -4 4 -0.5 3])
view(100,42)
grid on
xlabel('X [m]')
ylabel('Y [m]')

N = 250;
time = linspace(0,1,N);

for i = 1:size(Nb,2)
    if i>1    % Reseta ponto de partida[
        na = n;
        ps = pfd;
        psd = ps -dD.*(na/norm(na));
        hold on
        plot3(psd(1),psd(2),psd(3),'^k','MarkerSize',5,'LineWidth',2)
    else
        psd = p0 +dD.*(n0/norm(n0));
        hold on
        plot3(psd(1),psd(2),psd(3),'^k','MarkerSize',5,'LineWidth',2)
    end 
    
    pf = Bmb(i,:);
    n  = Nb(i,:);
    V = Vb(i,:)/norm(Vb(i,:));
    U = Ub(i,:)/norm(Ub(i,:));

    pfd = pf +dD.*(n/norm(n)); % ponto final deslocado por segurança   
    pc = pfd +2*dD.*(n/norm(n)); % ponto de controle
    dp = pfd-ps(1,:); % ponto inicial ao ponto deslocado

    % Definição Retas:
    t = -1.5:0.1:1.5;
    L = t.*n'; % passa pelo bambolê
    R = t.*dp'; % do A ao bambolê
    z = t.*cross(R,L)+pfd'; % normal ao plano
    
    % Desenho dos pontos e retas ==========================================
    % Ponto inicial:                     (PRETO)
    hold on
    plot3(ps(1),ps(2),ps(3),'ko')
    hold on
    plot3(ps(1),ps(2),ps(3),'k*')

    % Ponto final:                     (VERMELHO)
    hold on
    plot3(pf(1),pf(2),pf(3),'ro')
    hold on
    plot3(pf(1),pf(2),pf(3),'r*')

    % Ponto final deslocado:        (PRETO/VERMELHO)
    hold on
    plot3(pfd(1),pfd(2),pfd(3),'ko')
    hold on
    plot3(pfd(1),pfd(2),pfd(3),'r*')

    % Ponto de controle:                 (AZUL)
    hold on
    plot3(pc(1),pc(2),pc(3),'b*')
    hold on
    plot3(pc(1),pc(2),pc(3),'bo')

    % Normal do bambolê:
%     hold on
%     plot3(L(1,:)+pfd(1),L(2,:)+pfd(2),L(3,:)+pfd(3),'b-.','LineWidth',1)

    % Do A ao bambolê:
%     hold on
%     plot3([pfd(1) ps(1)],[pfd(2) ps(2)],[pfd(3) ps(3)],'r:.','LineWidth',1.2)

    % Bambolê 1: 
%     u = 0:0.01:2*pi;
%     XB = pf(1) +0.63.*cos(u).*V(1) +0.63*sin(u).*U(1);
%     YB = pf(2) +0.63.*cos(u).*V(2) +0.63*sin(u).*U(2);
%     ZB = pf(3) +0.63.*cos(u).*V(3) +0.63*sin(u).*U(3);
%     hold on
%     plot3(XB,YB,ZB,'LineWidth',2)
    grid on
    
    % Normal do bambolê:
%     hold on
%     plot3(L(1,:)+pfd(1),L(2,:)+pfd(2),L(3,:)+pfd(3),'b-.','LineWidth',1)

    % Do ArDrone ao bambolê:
%     hold on
%     plot3([pfd(1) ps(1)],[pfd(2) ps(2)],[pfd(3) ps(3)],'r:.','LineWidth',1.2)
    
    % Calcular e desenhar curva de Bèzier:
    ControlPoints = [ps;psd;pc;pfd];      PlotControlPoints(ControlPoints);
       
    B = bernsteinMatrix(3, time);
    bezierCurve = (B*ControlPoints);
    CB = [CB; bezierCurve];
    hold on
    plot3(bezierCurve(:,1),bezierCurve(:,2),bezierCurve(:,3),'LineWidth',1.8)
    
    % Atravessar o bambolê:
    ps = pfd; psd = ps-dD.*(n/norm(n));  pfd = pf-dD.*(n/norm(n)); 
    ControlPoints = [ps;psd;pfd];         PlotControlPoints(ControlPoints);
        
    B = bernsteinMatrix(2, time);
    bezierCurve = (B*ControlPoints);
    CB = [CB; bezierCurve];
    hold on
    plot3(bezierCurve(:,1),bezierCurve(:,2),bezierCurve(:,3),'LineWidth',1.8)
    
    if i==size(Nb,2) % Fechar curva:
        ControlPoints = [pfd;pfd-dD.*(n/norm(n));p0-dD.*(n0/norm(n0));p0];
        PlotControlPoints(ControlPoints);
        
        hold on
        plot3(pfd(1),pfd(2),pfd(3),'^k','MarkerSize',5,'LineWidth',2)

        B = bernsteinMatrix(3, time);
        bezierCurve = (B*ControlPoints);
        CB = [CB; bezierCurve];
        hold on
        plot3(bezierCurve(:,1),bezierCurve(:,2),bezierCurve(:,3),'LineWidth',1.8)
    end
end

% hold on
% plot3(CB(1,:),CB(2,:),CB(3,:),'b--','LineWidth',2.5)
% figure
% plot3(CB(1,:),CB(2,:),CB(3,:),'b-')
%% Experimento ============================================================
% Create Drone 
A = ArDrone(1);
A.rGetSensorData
A.pPos.X = zeros(12,1);

XX = [];
flag1=1;
tmax = T; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação
ControlPoints = [];
ps = [0;0;0]; % ponto inicial

k = size(CB,1);      
dCB = [[0;0;0],diff(CB,1,1)'./A.pPar.Ts];
Curva.rho = 0;       Curva.psi = atan2(dCB(2,:),dCB(1,:));
Curva.X = CB';        
Curva.dX = [[0;0;0],diff(CB,1,1)'./A.pPar.Ts];
Curva.Pos = 1;       Curva.Size = k;
Curva.dpsi = [0,diff(Curva.psi,1,2)];
Curva.dXr = zeros(3,k);
Curva.Vmax = 0.5; % Curva.Vmax = 1/10; % => 120 seg

t = tic;
tc = tic;
tp = tic;

% while toc(t) < tmax
while Curva.Pos < Curva.Size
    if toc(tc) > A.pPar.Ts
        tc = tic;
        tt = toc(t);
        TT = (tt/tmax);
        itT = floor(k*TT)+1;     
        
        clc
        fprintf('Percorrendo: %0.2g%% \n',(Curva.Pos/Curva.Size)*100);
        
        %             12        12      1   
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];

        % Get optitrack data        
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
        else
            A.pSC.Kinematics_control = 1;
            A = cInverseDynamicController_Compensador_ArDrone(A);
        end
                
        A.rSendControlSignals

        flag=0;
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

%%
figure
subplot(311)
plot(Curva.dX(1,:))
title('dX')
subplot(312)
plot(Curva.X(1,:))
title('X')
subplot(313)
plot(Curva.psi)
title('Psi')

%         A.pPos.Xd(1) = CB(1,itT);     
%         A.pPos.Xd(2) = CB(2,itT);
%         A.pPos.Xd(3) = CB(3,itT);
%         A.pPos.Xd(6) = Curva.psi(itT);
%         A.pPos.Xd(7) = dCB(1,itT);  
%         A.pPos.Xd(8) = dCB(2,itT);
%         A.pPos.Xd(9) = dCB(3,itT);
%         A.pPos.Xd(6) = Curva.dpsi(itT);
