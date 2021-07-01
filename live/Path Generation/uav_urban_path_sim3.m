%% Simulação do planejamento de trajetória para bambolês (wayposes)
% - Implementar seguimento de caminho
% - Analisar velocidade/curvatura no tempo, se não tiver ok corrigir
% - Distorcer curva para proteger bambolês passados

% Boas práticas
close all
clear
clc


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

%% ========================================================================
% Definição da cena:
%Diametro do A real (63 cm)
dD = .7;

% Bambolês:
% b1 =[1,1,1.2];     n1 = [-1,-0.5,1];
% v1 = [1,0,1];    u1 = [0,1,0.5];

b1 = [0,-2,1.35]; n1 = [1,0,0];
v1 = [0,1,0];    u1 = [0,0,1];

% Vários:
b1 =[1,1.3,1.9];     n1 = [-1,-0.5,1];
v1 = [1,0,1];    u1 = [0,1,0.5];


b2 = [2.5,-1,1.9];     n2 = [-1,-1,1].*-0.25;
v2 = [0.5,0,0.5];   u2 = [0,0.5,0.5];

b3 = [-2.5,-2,2.4];   n3 = [1,0,0];
v3 = [0,-1,0];      u3 = [0,0,1];

% Sem optitrack
% b4 = [0;0;1.35]; n4 = [0;1;0];
% v4 = [1;0;0];   u4 = [0;0;1];

% Conjunto dos bambolês:
Bmb = [b1;b2;b3]; Nb = [n1;n2;n3];
Vb = [v1;v2;v3];  Ub = [u1;u2;u3];

% Só um:
Bmb = [b2]; Nb = [n2];
Vb = [v2];  Ub = [u2];
Curva =[];        Curva.Vmax = .15;
% 0.07;
%% Plots ==================================================================
p0 = [0,0,.75]; n0 = [1,0,0];% ponto inicial
[CB,dCB,Curva] = CurvaBmb(p0,n0,Bmb,Nb,Vb,Ub,Curva);
xlim([-4 4]); ylim([-3 2]); zlim([0 4]);

vert = [-4       -3     0;
        4        -3     0;
        4         2     0;
        -4        2     0;
        4        -3     4;
        4         2     4;
        -4       -3     4];

fac = [1 2 3 4;2 3 6 5;2 1 7 5];
patch('Vertices',vert,'Faces',fac,...
    'FaceColor',0.8*[1 1 1],'FaceAlpha',0.3,'LineWidth',1.6)

plot3(b1(1),b1(2),0,'b+','MarkerSize',10,'LineWidth',2);
plot3([b1(1),b1(1)],[b1(2),b1(2)],[b1(3),0],'b--')

plot3(b2(1),b2(2),0,'b+','MarkerSize',10,'LineWidth',2);
plot3([b2(1),b2(1)],[b2(2),b2(2)],[b2(3),0],'b--')

plot3(b3(1),b3(2),0,'b+','MarkerSize',10,'LineWidth',2);
plot3([b3(1),b3(1)],[b3(2),b3(2)],[b3(3),0],'b--')

plot3([0,0],[0,0],[0.75,0],'k--')

%% Experimento ============================================================
A = ArDrone(1);
A.rGetSensorData
A.pPos.X = zeros(12,1);

figure(1)
hold on
p1 = plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'k+','MarkerSize',10,'LineWidth',2);
XX = [];
ps = [0;0;0.75]; % ponto inicial



t = tic;
tc = tic;
tp = tic;
rho = 0;
Curva.Vmax = .3;
% E = 2*V
% while toc(t) < tmax
while Curva.Pos < Curva.Size
    if toc(tc) > A.pPar.Ts
        % Atualiza tempo:
        tc = tic;  tt = toc(t); 
        
        clc % Mostrar progresso:
        fprintf('Posição no caminho: %i \n',Curva.Pos)
        fprintf('Percorrendo: %0.2g%% \n',(Curva.Pos/Curva.Size)*100)
        
        A.rGetSensorData % Get data 
        
        % Controlador
        [A,Curva,rho] = cPathFollowing(A,Curva,.5);
        A.pPos.Xr([1:3,6:9]) = [A.pPos.Xd(1:3);A.pPos.Xd(6);A.pPos.Xd(7:9)];
        
%         if rho<0.35
%             A.pSC.Kinematics_control = 1;
%         else
%             A.pSC.Kinematics_control = 0; 
%         end
             
        A = cInverseDynamicController_Compensador_ArDrone(A); 
        A.rSendControlSignals

        % Histórico:  12        12      1   
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];
        Curva.rho = [Curva.rho,rho];
                

        
%         figure(1)
%         delete(p1)
%         hold on
%         p1 = plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'k^','MarkerSize',10);
    end
end

clc
disp('Calculado!')
disp('Desenhando cena...')

%% Resimulando ============================================================
disp('Pronto para reproduzir.')
disp('Reproduzindo simulação...')
figure(1)
hold on
plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',2)
grid on
for tr = 1:7:size(XX,2)
    A.pPos.X = XX([13:24],tr);
    A.mCADplot; 
    drawnow
end
% =========================================================================
% tmax = 540;
% tmax = 445;
% tmax = 139;

%% Gráficos:
figure
subplot(311)
plot(Curva.dX(1,:)) % Velocidade dXd
title('dX')
subplot(312)
plot(Curva.X(1,:)) % Posição Xd
title('X')
subplot(313)
plot(Curva.psi) % Orientação Psid
title('Psi')

%%
figure
subplot(311)
plot(Curva.dX(1,1:Curva.Pos)) % Velocidade dXd
title('dX')
subplot(312)
plot(Curva.dX(2,1:Curva.Pos)) % Posição Xd
title('dY')
subplot(313)
plot(Curva.dX(3,1:Curva.Pos)) % Orientação Psid
title('dZ')
%%
figure
% plot(Curva.psi(1:Curva.Pos),'r--') % Orientação Psid
plot(XX(end,:),XX(6,:)*(180/pi),'r--','LineWidth',1.6) % Orientação Psid
hold on
plot(XX(end,:),XX(18,:)*(180/pi),'b-','LineWidth',1.6)
plot(XX(end,:),(XX(6,:)-XX(18,:))*(180/pi),'k-')
title('Psi')

grid on

%%
u = []; ud = u;
for kk = 1:size(XX,2)
    u = [u,norm(XX(19:21,kk))];
    ud = [ud,norm(XX(7:9,kk))];
end
figure
subplot(211)
plot(XX(end,:),ud,'r--','LineWidth',1.6)
hold on
plot(XX(end,:),u,'b-','LineWidth',1.6)
plot(XX(end,:),0.07.*ones(1,size(XX,2)),'g-','LineWidth',1.6)
grid on
title('Ud vs U')
subplot(212)
plot(XX(end,:),ud-u,'k-','LineWidth',1.6)
grid on
title('Util')
%%
% Erro de caminho
figure(2)
plot(Curva.rho)
hold on
plot(1:size(Curva.rho,2),(0.2).*ones(1,size(Curva.rho,2)),'r--')
title('Erro de caminho')
xlabel('Pos','Font','16')
ylabel('Til','Font','16')

% figure
% plot(XX(:,end))

% Velocidade
% figure
% plot(XX(19,:))
% hold on
% plot(Curva.dX(1,:))
% grid on

%         A.pPos.Xd(1) = CB(1,itT);     
%         A.pPos.Xd(2) = CB(2,itT);
%         A.pPos.Xd(3) = CB(3,itT);
%         A.pPos.Xd(6) = Curva.psi(itT);
%         A.pPos.Xd(7) = dCB(1,itT);  
%         A.pPos.Xd(8) = dCB(2,itT);
%         A.pPos.Xd(9) = dCB(3,itT);
%         A.pPos.Xd(6) = Curva.dpsi(itT);
