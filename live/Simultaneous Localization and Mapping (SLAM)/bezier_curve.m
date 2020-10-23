%% Caminho de Bèzier - Iure Rosa 18/01/20
%   Implementando planejamento de caminho por 
% curva de Bèzier de 5º grau com 2 parâmetros
% "aproximados".

%% Boas práticas
close all
clearvars
clc
%% Abrir V-REP
% cd([pwd,'\V-Rep_estavel\Scene'])
% ! scene.ttt
% 
% disp('Inicie a simulação!')
% pause(10)

%% Carregando os objetos do cenário
V = VREP;
V.vConnect
V.vHandle('Pioneer_p3dx');


%% Definindo o Robô


%% Inicializando variáveis para o controle
%Declarando o trace:
dados=[];
Rastro.Xd = [];P = Pioneer3DX;
P.pPar.a = 0;
P.pPar.alpha = (0)*(pi/180);
% P.rConnect; % Descomentar para realiazação de experimento
P.rSetPose([0 0 0 0]);

disp('Stopping Pioneer')
P.pSC.Ud = [0; 0];
V.vSendControlSignals(P,1);

pause(1);
Rastro.X = [];
V.vGetSensorData(P,1);

%% Variáveis para o caminho:
%Definindo variaveis:
xs  = 0;                        ys = 0;               %Posição inicial
xf  = 2;                        yf = 3;               %Posição final
ths = 0;                        thf = (120)*(pi/180);   %Orientação inicial/final

%Pontos de controle:
b0 = [xs;ys];                   b5 = [xf;yf];   
c0 = 2;                         c5 = 2;
d0 = c0.*[cos(ths),sin(ths)];   d5 = c5.*[cos(thf),sin(thf)];

b1 = b0 + [(1/5).*d0]';         b4 = b5 - [(1/5).*d5]';

b2 = b1 + [(1/5).*d0]';         b3 = b4 - [(1/5).*d5]';        %Inicialização aleatória


ControlPoints = [b0,b1,b2,b3,b4,b5]; %=> N = 6;

%Plot da curva
t = 0:0.01:1;
N = 6;
Sum = 0;    

for i=0:N-1
    Sum = Sum + nchoosek(N-1,i)*(1-t).^(N-1-i).*(t.^i).*ControlPoints(:,i+1);
end
Curve = Sum;

plot(xs,ys,'k*')
hold on
plot(xf,yf,'r*')
hold on
plot(ControlPoints(1,:),ControlPoints(2,:),'bo')
hold on
plot(ControlPoints(1,:),ControlPoints(2,:),'k*')
hold on
plot(Curve(1,:),Curve(2,:),'b--')
hold on
plot(Curve(1,:),Curve(2,:),'b*')
axis([-1 4 -1 4])
grid on

        
%% Rotina da simulação:
t=tic;  ta=tic;   tp = tic;         tc=tic; tcmax=35;
        tmax=40;
k1=0.6; k2=0.4;   it = 0;

while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        
        P.pPos.Xda     = P.pPos.Xd;
        it=it+1;
        
       
        time = toc(tc)/(tcmax);
        if toc(tc)>tcmax
            time=1;
        end
        %Pegar informação da posição e velocidade real do robô
        V.vGetSensorData(P,1);
        
        %% Cálculo da curva:
        N = 6;
        Sum = 0;

        for i=0:N-1
            Sum = Sum + nchoosek(N-1,i)*(1-time).^(N-1-i).*(time.^i).*ControlPoints(:,i+1);
        end
        Curve = Sum;

        
        %% Robot control
        %Posição do ponto desejado:
        P.pPos.Xd(1)= Curve(1);
        P.pPos.Xd(2)= Curve(2);
        
        % Pegando os dados do robo
%         disp('.........................')
%         disp(P.pPos.X(1))
        
        V.vGetSensorData(P,1);
%         disp(P.pPos.X(1))
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        % Controlador Cinemático modelo extendido
        vx = P.pPos.Xd(7) + 0.6*P.pPos.Xtil(1);
        vy = P.pPos.Xd(8) + 0.6*P.pPos.Xtil(2);
                
        if abs(P.pPar.alpha) < pi/2 && P.pPar.a > 0
            vw = (-sin(P.pPos.X(6))*vx + cos(P.pPos.X(6))*vy)/(P.pPar.a*cos(P.pPar.alpha));
            P.pSC.Ud(2) = vw;
        else              
            P.pPos.Xd(6)  = atan2(vy,vx);
            P.pPos.Xd(12) = (P.pPos.Xd(6)-P.pPos.Xda(6))/0.1;            
            P.pPos.Xtil(6) = P.pPos.Xd(6) - P.pPos.X(6);
            if abs(P.pPos.Xtil(6)) > pi
                if P.pPos.Xtil(6) > 0
                    P.pPos.Xtil(6) = - 2*pi + P.pPos.Xd(6) - P.pPos.X(6);
                else
                    P.pPos.Xtil(6) =   2*pi + P.pPos.Xd(6) - P.pPos.X(6);
                end
            end
            vw = 0*(P.pPos.Xd(12)) + 1.5*P.pPos.Xtil(6);
        end
        
        P.pSC.Ud(2) = vw;
        P.pSC.Ud(1) = vx*cos(P.pPos.X(6)) + vy*sin(P.pPos.X(6)) ...
                           + P.pPar.a*sin(P.pPar.alpha)*vw;

        
        % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % formação desejada
        Rastro.X  = [Rastro.X; P.pPos.X(1:2)'];    % formação real
        
        % Enviar sinais de controle para o robô
        V.vSendControlSignals(P,1);     
        
          %% Desenha o robô
            if toc(tp) > 0.1
                tp = tic;
                
                P.mCADdel
                P.mCADplot2D('r')   % Visualização 2D
         
                hold on
                plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'r-');
                hold on;
                plot(Rastro.X(:,1),Rastro.X(:,2),'black');
                
                
                drawnow
            end
    end
end

P.pSC.Ud([1 2]) = 0;
V.vSendControlSignals(P,1);

%% Desconecta Matlab e V-REP
V.vDisconnect;

%% Plot Charts

%Angular and Linear Velocity
figure
plot(dados(:,29),dados(:,25))
hold on
plot(dados(:,29),dados(:,26))
title('Velocidade Linear e Angular')
%Tracking
figure
plot(dados(:,13),dados(:,14))
title('Trajetória do Ponto de Controle')
%Error
figure
plot(dados(:,29),-dados(:,13)+dados(:,1))
title('Erro X')















