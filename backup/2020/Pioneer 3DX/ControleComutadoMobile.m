clear
close all
clc

try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Definindo o Robo
P = Pioneer3DX;
% P.rConnect;
P.rSetPose([0 0 0 0]);
pause(5)

%% Definindo a Figura que irá rodar a simulação
figure(1)
hold on
grid on
title('Simulação');
axis([-1 3 -1 3])

%% Definindo as matrizes de dados
Dados = [];
SinCont = [];

%% Constantes
Destino = 0;
it = 1;
Rep = 0;
Repc = 0;
Troca = 0;
% Constantes da trajetória
a = 1.5;
b = 1;

%% Definindo os indices de desempenho
IAE = 0;
ITAE = 0;
IASC = 0;

%% Tempo Real
tmax = 10;
w = 1/tmax;
ta = tic;
t = tic;
w = 2*pi/tmax;

%% Inicio da simulação
while (toc(t) <= tmax)
    if toc(ta) > 0.1
        %% Inicio da realimentação
        ta = tic;       
        it = it + 1;
        
        %% Definindo a Posição Desejada
        if toc(t) <= tmax/3
            P.pPos.Xd(1) = 2;
            P.pPos.Xd(2) = 2;
        elseif toc(t) <= tmax*2/3
            if Repc == 0
                Rep = 0;
                Repc = Repc + 1;
            end
            P.pPos.Xd(1) = 2;
            P.pPos.Xd(2) = 0;
        else
            if Repc == 1
                Rep = 0;
                Repc = Repc + 1;
            end
            P.pPos.Xd(1) = 0;
            P.pPos.Xd(2) = 2;
        end
        P.pPos.Xd(7) = 0;
        P.pPos.Xd(8) = 0;
        
        %% Definindo a Trajetória Desejada        
%         P.pPos.Xd(1) = a*sin(w*toc(t));
%         P.pPos.Xd(2) = b*sin(2*w*toc(t));
%         P.pPos.Xd(7) = w*a*cos(w*toc(t));
%         P.pPos.Xd(8) = 2*w*b*cos(2*w*toc(t));

        %% Pegando os dados do robo
        P.rGetSensorData;
        
        %% Erro
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        %% Controlador
        % Constantes do controlador
        Ka = 0.5; 
%         Ka (Robo) = 0.2;
%         Ka = 0.8;       
%         Ka = 0.65;
        Kb = Ka;
%         G1 = 0.5;   
%         G1 (Robo) = 0.2;
%         G1 = 0.2;
        G1 = 1.06;
        G2 = G1;
        G = [G1 0;
             0 G2];
        % Definindo a Função de Controle
        A = [P.pPos.Xtil(1)/sqrt(P.pPos.Xtil(1)^2 + Ka^2);
             P.pPos.Xtil(2)/sqrt(P.pPos.Xtil(2)^2 + Kb^2)];
        % Definindo o Controlador
        K = [cos(P.pPos.X(6)) -P.pPar.a*sin(P.pPos.X(6));
             sin(P.pPos.X(6)) P.pPar.a*cos(P.pPos.X(6))];
        P.pSC.Ud = K\(P.pPos.Xd([7,8]) + G*A);

        %% Controlador de Orientação
        % Calculando o Ângulo necessário
        Psir = atan2((P.pPos.Xd(2)-P.pPos.Xc(2)),(P.pPos.Xd(1)-P.pPos.Xc(1)));
        % Iniciando o Controle
        ANG = abs(P.pPos.Xc(6) - Psir)*180/pi;
%         disp(ANG)
        % Corrigindo a singularidade caso Psi desejado seja 180 graus      
        if ANG == 180
            P.pSC.Ud(1) = 0;
            P.pSC.Ud(2) = 1;
        end
        % O primeiro limite para o controlador sera de 5 graus
        if (ANG > 5 && abs(P.pSC.Ud(2)) > 0.1) && Rep == 0
            P.pSC.Ud(1) = 0;
        elseif Rep == 0
            Rep = 1;
            Troca = 1;
        end
        if Rep == 1
            P.pSC.Ud(2) = 0;
        end
        % Caso Psi desejado seja maior que 10 graus, ele ira corrigir        
        if ANG > 10 && Rep == 1
            Rep = 0;
            Troca = 1;
        end
        
        %% Outro controlador
%         P.pSC.Ud = [3.03 0]';
%         P.pSC.Ud = [0 100/180*pi*11/3.38]';

%         K = [cos(P.pPos.X(6)) -P.pPar.a*sin(P.pPos.X(6));
%             sin(P.pPos.X(6)) P.pPar.a*cos(P.pPos.X(6))];
%         P.pSC.Ud = K\(P.pPos.Xd([7,8]) + 10*P.pPos.Xtil([1,2]));

        %% Definindo alarme para limites físicos
%         if abs(P.pSC.Ud(1)/3.03) > 0.75
% %             P.pSC.Ud(1) = 0.75*P.pSC.Ud(1)/abs(P.pSC.Ud(1))*3.03;
%             ualarm = P.pSC.Ud(1)/3.03;
%             disp(ualarm)
%             disp('u')
%         end
%         if abs(P.pSC.Ud(2)*3.38/11) > 100/180*pi
% %             P.pSC.Ud(2) = 100/180*pi*P.pSC.Ud(2)/abs(P.pSC.Ud(2))*11/3.38;
%             walarm = P.pSC.Ud(2)*3.38/11*180/pi;
%             disp(walarm)
%             disp('w')
%         end
        
        %% Enviando sinais para o robo
        P.rSendControlSignals;
%         if Troca == 1
%             Troca = 0;
%             pause(1)
%         end
        
        %% Plot da simulação
        % Plot do robo
        P.mCADdel
        P.mCADplot(1,'r')
        try 
            delete(h)
        end 
        % Matriz de dados
        Dados = [Dados;
            [P.pPos.Xd' P.pPos.X' P.pPos.Xtil' P.pSC.Ud' P.pSC.Ur' P.pSC.U' toc(t)]];
        % Plot do destino
        h(1) = plot(Dados(:,1), Dados(:,2),'xr');
        h(2) = plot(Dados(:,13), Dados(:,14),'-k');
        drawnow
        
        %% Análise de Desempenho
        IAE = IAE + norm(P.pPos.Xtil(1:2));
        ITAE = ITAE + Dados(end,end)*norm(P.pPos.Xtil(1:2));
        IASC = IASC + norm(P.pSC.Ud);
        
    end
end

P.pSC.Ud([1 2]) =0;
P.rSendControlSignals;
%% Plotando resultados
hold off
%% Erro ~x e ~y em funçao do tempo
figure(2)
plot(Dados(:,end),Dados(:,25),Dados(:,end),Dados(:,26));
axis([0, tmax,-3,3])
grid on
title('Erro');
xlabel('Tempo (s)');
ylabel('Metros');
legend('Erro X','Erro Y');

%% Diferença Xd e Xr em funçao do tempo
figure(3)
plot(Dados(:,end),Dados(:,1),'--r',Dados(:,end),Dados(:,13),'-k');
axis([0, tmax,-3,3]);
grid on
title('Diferença em Xd e X');
xlabel('Tempo (s)');
ylabel('Metros');
legend('X desejado','X real');

%% Diferença Yd e Yr em funçao do tempo
figure(4)
plot(Dados(:,end),Dados(:,2),'--r',Dados(:,end),Dados(:,14),'-k');
axis([0, tmax,-3,3]);
grid on
title('Diferença em Yd e Y');
xlabel('Tempo (s)');
ylabel('Metros');
legend('Y desejado','Y real');

%% Diferença Ud e Ur em funçao do tempo
vector = 0.75*ones(size(Dados(:,end)));
figure(5)
plot(Dados(:,end),Dados(:,41)/3.03,'-r',Dados(:,end),Dados(:,39),'-k',Dados(:,end),vector,'--k');
grid on
axis([0,tmax,-.1,1])
title('Controle de Velocidade Linear');
xlabel('Tempo (s)');
ylabel('m/s');
legend('U desejado','U real');

%% Diferença Wd e Wr em funçao do tempo
vector2 = 100*ones(size(Dados(:,end)));
vector3 = -100*ones(size(Dados(:,end)));
figure(6)
plot(Dados(:,end),Dados(:,42)*180/pi/11*3.38,'-r',Dados(:,end),Dados(:,40),'-k',Dados(:,end),vector2,'--k',Dados(:,end),vector3,'--k');
grid on
axis([0 tmax -110 110])
title('Controle de Velocidade Angular');
xlabel('Tempo (s)');
ylabel('Graus/s');
legend('W desejado','W real');

%% Angulo Psi
figure(7)
plot(Dados(:,end),Dados(:,18)*180/pi);
grid on
axis([0 tmax -400 400])
title('Angulo Psi');
xlabel('Tempo (s)');
ylabel('Graus');

%% Velocidade Angular e Linear
figure(8)
plot(Dados(:,end),Dados(:,41)/3.03,'-b',Dados(:,end),Dados(:,42)*180/pi/11*3.38/100,'-r');
grid on
% axis([0 tmax -0.4 0.3])
title('Velocidade Angular e Linear');
xlabel('Tempo (s)');
ylabel('0.01Graus/s | m/s');
legend('Velocidade Linear','Velocidade Angular (W/100)');
