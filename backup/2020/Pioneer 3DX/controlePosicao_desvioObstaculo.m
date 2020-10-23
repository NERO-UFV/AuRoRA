clear 
close all
clc

%Classe Piorneer
P = Pioneer3DX;

%% Conectar ao MobileSim
P.rConnect
%% Iniciando Variaveis de Desempenho
% - Integral do erro absoluto
IAE=0;
IAEt=0;
% – Integral do erro absoluto vezes o tempo
ITAE=0;
% - Integral absoluta do sinal de controle
IASC=0;

%% Declarando a figura
figure(1)
hold on
grid on
axis([-5,5,-5,5])
title('Simulação')
%% Declarando o trace
Dados=[];
Desempenho = [];
%% Inicializando variáveis para o controle
it=0;
tmax=20;
tparcial = 15;
t=tic;ta=tic;
k2=0;

%% Parametros das funções
a1= 1.5; b = 1; T = 60; w = 2*pi/T;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CONTROLE DE TRAJETORIA
%  for k1 = 0.2:0.1:4.7
%     for k2 = 0.2:0.1:4.7
        t=tic;ta=tic;
k1=0.5;
k2=0.3;
Fk=0.05; %constante de respulsão
Fc =0.3; %constante de atração
        while toc(t)<tmax
            if toc(ta)>0.1
                it=it+1;
                ta=tic;
                
                %% Posição desejada para o Robô
                %Trajetoria da Leminiscata como lugar desejado para o robô
%               P.pPos.Xd(1)=a1*sin(w*toc(t));
%               P.pPos.Xd(2)=b*sin(2*w*toc(t));
%               P.pPos.Xd(7)= a1*w*cos(w*toc(t));
%               P.pPos.Xd(8)= 2*b*w*cos(2*w*toc(t));
%               P.pPos.Xc(6)=atan2(P.pPos.Xc(8),P.pPos.Xc(7));
%               P.pPos.Xc([1 2 6])=2*randn(1);

                P.pPos.Xd(1)=2;
                P.pPos.Xd(2)=0;
                P.pPos.Xd(7)=0;
                P.pPos.Xd(8)=0;
                K=[ cos(P.pPos.X(6)) -P.pPar.a*sin(P.pPos.X(6)); sin(P.pPos.X(6)) P.pPar.a*cos(P.pPos.X(6))];
        
                %% Pegar informação de onde o robô esta
                P.rGetSensorData;
                
                %% Controle e Desvio de Obstaculo
                %Erro de Posição
                P.pPos.Xtil=P.pPos.Xd-P.pPos.X;
                distanciaAlvoRobo = sqrt(P.pPos.Xtil([1])^2+P.pPos.Xtil([2])^2);

                %Matriz de Obstáculos
                Obst = [1 1; 0.1 -0.2];
                
              
                for i=1:length(Obst(1,:))
                    %distancia do robo pro obstaculo
                    d(i) = sqrt((Obst(1,i)-P.pPos.X([1]))^2+(Obst(2,i)-P.pPos.X([2]))^2);
                    Frep(:,i) = -Fk/d(i)^2*[Obst(:,i)-P.pPos.X([1 2])]./d(i);
                    if d(i)<=0.6
                        flag=1;
                    end

                end
                    FrepSoma = sum(Frep,2);
                    Fatrac = Fc*[P.pPos.Xd([1 2])-P.pPos.X([1 2])]./distanciaAlvoRobo;
                    Fr = FrepSoma + Fatrac;
%                 if -0.05< Fr(1)+Fr(2)<0.05
%                     theta = atan(P.pPos.Xtil(2)/P.pPos.Xtil(1));
%                     Fr = [cos(theta) sin(theta); sin(theta) cos(theta)]*[Fr(1);Fr(2)]
%                 end
%                 if Dados(it,25)-Dados(it-1,25)<
% 
%                 P.pPos.X([1 2])
                if flag == 1 
                    P.pSC.Ud=K\(Fr);
                    flag=0;
                else
                    P.pSC.Ud=K\(P.pPos.Xd([7 8])+k1*tanh(k2*P.pPos.Xtil([1 2])));
                end
                
%                 if P.pSC.Ud(1)>0.75
%                     IAE=100000;
%                     ITAE = 100000;
%                     IASC = 100000;
%                     break
%                 end
                P.rSendControlSignals;
                %% Trace
                Dados=[Dados ; [P.pPos.Xd' P.pPos.X' P.pPos.Xtil' P.pSC.Ud' P.pSC.Ur' toc(t)]];
                %Plotando Pioneer
                P.mCADdel
                P.mCADplot(1,'r')

                % Plotar Rastro
                try
                    delete(h(1))
                    delete(h(2))
                end
                h(1)=plot(Dados(:,[1]),Dados(:,[2]),'og');
                h(2)=plot(Dados(:,[13]),Dados(:,[14]),'-k');
                h(3)=plot(Obst(1,1),Obst(2,1),'or');
                h(4)=plot(Obst(1,2),Obst(2,2),'or');
                drawnow
                %% Calculo de desempenho do controlador
                % calculo do transitório
                if toc(t)<15
                    IAEt = IAEt + norm(P.pPos.Xtil(1:2));
                else
                    IAE = IAE + norm(P.pPos.Xtil(1:2));
                end

                ITAE = ITAE + Dados(end,end)*norm(P.pPos.Xtil(1:2));
                IASC = IASC + norm(P.pSC.Ud);

            end
        end
        IAEt= IAEt*0.1;
        IAE = IAE*0.1;
        ITAE = ITAE*0.1;
        IASC = IASC*0.1;
        Desempenho = [Desempenho; k1 k2 IAE ITAE IASC];
        P.pSC.Ud = [0; 0];
        P.rSetPose([0 0 0 0])
        P.rSendControlSignals;
        pause(2)
        
%     end
    
% end



% IAE = IAE*0.1;
% ITAE = ITAE*0.1;
% IASC = IASC*0.1;
% disp('Desempenho do controlador de posição:');
% disp(IAE);
% disp(ITAE);
% disp(IASC);
% 


%% Plotando resultados
hold off
% %% Erro ~x e ~y em funçao do tempo
% figure(2)
% plot(Dados(:,end),Dados(:,25),Dados(:,end),Dados(:,26));
% axis([-0.5, tmax,-2,2])
% grid on
% title('Erro');
% xlabel('Tempo (s)');
% ylabel('Metros');
% legend('Erro X','Erro Y');
% 
% %% Diferença Xd e Xr em funçao do tempo
% figure(3)
% plot(Dados(:,end),Dados(:,1),'--r',Dados(:,end),Dados(:,13),'-k');
% axis([-0.5, tmax,-2,2]);
% grid on
% title('Diferença em Xd e Xr');
% xlabel('Tempo (s)');
% ylabel('Metros');
% legend('X desejado','X real');
% 
% %% Diferença Yd e Yr em funçao do tempo
% figure(4)
% plot(Dados(:,end),Dados(:,2),'--r',Dados(:,end),Dados(:,14),'-k');
% axis([-0.5, tmax,-2,2]);
% grid on
% title('Diferença em Yd e Yr');
% xlabel('Tempo (s)');
% ylabel('Metros');
% legend('Y desejado','Y real');
% 
% %% Diferença Ud e Ur em funçao do tempo
vector = 0.75*ones(size(Dados(:,end)));
figure(5)
plot(Dados(:,end),Dados(:,37),'--r',Dados(:,end),Dados(:,39),'-k',Dados(:,end),vector,'--b');
grid on
axis([-0.5,tmax,-.5,3])
title('Controle de Velocidade Linear');
xlabel('Tempo (s)');
ylabel('m/s');
legend('U desejado','U real');
% 
%% Diferença Wd e Wr em funçao do tempo
% figure(6)
% plot(Dados(:,end),Dados(:,38),'--r',Dados(:,end),Dados(:,40),'-k');
% grid on
% axis([-0.5,tmax,-5,5])
% title('Controle de Velocidade Angular');
% xlabel('Tempo (s)');
% ylabel('Rad/s');
% legend('W desejado','W real');
% 
% %% Angulo Psi
% figure(7)
% plot(Dados(:,end),Dados(:,18));
% grid on
% axis([-0.5,tmax,-10,10])
% title('Angulo Psi');
% xlabel('Tempo (s)');
% ylabel('Rad');

%disp(it)