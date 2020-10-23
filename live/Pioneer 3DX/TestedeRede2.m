clear all
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

%% ID do robo
ID = input('Digite o ID do robô: ');
P(ID) = Pioneer3DX(ID);

%% Conectar o robô
P(ID).rConnect;

%% Definindo os valores das variaveis para o inicio
Tipo = 0;
N = 0;
Ang = 0;

%% Perguntas iniciais sobre a simulação
while N < 2 || N < ID
    N = input('Digite o número de robôs: ');
end
for ii=1:N
    if ii ~= ID
        P(ii) = Pioneer3DX(ii);
    end
end

%% Defininindo o local desejado da formação
Qd = [0 0 1 0]';
dQd = [0 0 0 0]';
% Angulo alfa da formação
alfa = pi*(N-2)/N;
% Definindo a posição desejada da formaçao
if N>2
    for ii=2:N-1
        Qd(:,ii) = [Qd(1,ii-1)+Qd(3,ii-1)*cos(Qd(4,ii-1));
                    Qd(2,ii-1)+Qd(3,ii-1)*sin(Qd(4,ii-1));
                               Qd(3,ii-1);
                           (pi-alfa)*(ii-1)];
        dQd(:,ii) = [0 0 0 0]';
    end
end
%% Definindo posição desejada dos robos a partir da formaçao
for ii=1:N-1
    Xd(:,ii) = [           Qd(1,ii); 
                           Qd(2,ii);
                Qd(1,ii) + Qd(3,ii)*cos(Qd(4,ii)); 
                Qd(2,ii) + Qd(3,ii)*sin(Qd(4,ii))];
end

%% Definindo a posiçao inicial da formaçao
Qi = [0 0 1 0]';
Qi(3,1) = Qd(3,1);

if N>1
    for ii=2:N-1
        Qi(:,ii) = [Qi(1,ii-1)+Qi(3,ii-1)*cos(Qi(4,ii-1));
                    Qi(2,ii-1)+Qi(3,ii-1)*sin(Qi(4,ii-1));
                               Qi(3,ii-1);
                           (pi-alfa)*(ii-1)];
    end
end

%% Definindo a posiçao inicial dos robos a partir da formaçao
for ii=1:N-1
    Xi(:,ii) = [           Qi(1,ii); 
                           Qi(2,ii);
                Qi(1,ii) + Qi(3,ii)*cos(Qi(4,ii)); 
                Qi(2,ii) + Qi(3,ii)*sin(Qi(4,ii))];
end

%% Definindo a posição do robô
if ID < N
    P(ID).rSetPose([Xi(1,ID) Xi(2,ID) 0 0]);
elseif ID == N %se o robô for o ultimo da formação, seus dados estaram na linha 3 e 4 da ultima coluna
    P(ID).rSetPose([Xi(3,ID-1) Xi(4,ID-1) 0 0]);
end

%% Forçando uma posição inicial "aleatoria"

% Xi = [0  1;
%       0 -1;
%       1  1;
%      -1  1];

% Xi = [0  1  1  3;
%       0 -1  1 -2;
%       1  1  3 -2;
%      -1  1 -2 -4];

%% Calculando o erro inicial
for ii=1:N
    if ii<N
        P(ii).pPos.X(1:2) = [Xi(1,ii);Xi(2,ii)];
        P(ii).pPos.Xd(1:2) = [Xd(1,ii);Xd(2,ii)];
        P(ii).pPos.Xtil = P(ii).pPos.Xd - P(ii).pPos.X;
    else
        P(ii).pPos.X(1:2) = [Xi(3,ii-1);Xi(4,ii-1)];
        P(ii).pPos.Xd(1:2) = [Xd(3,ii-1);Xd(4,ii-1)];
        P(ii).pPos.Xtil = P(ii).pPos.Xd - P(ii).pPos.X;
    end
end

%% Definindo a Rede
Rede = NetDataShare;

%% Testando a conexão da rede
tR = 0;
tRede = tic;
Rede.pMSG.getFrom{N} = [];

while true
    
%     if isempty(Rede.pMSG.getFrom)
%         Rede.mSendMsg(P(ID));
%         if toc(tR) > 1/30
%             tR = tic;
%             Rede.mReceiveMsg;
%             disp('Waiting for message......')
%         end
%         
%     elseif length(Rede.pMSG.getFrom) > 1
        contvazio = 0;
        conttime = 0;
        for ii = 1:size(Rede.pMSG.getFrom,2)
            if ~isempty(Rede.pMSG.getFrom{ii})
                contvazio = contvazio + 1;
            end
            if tR == 1 && Rede.pMSG.getFrom{ii}(end) >= 1
                conttime = conttime + 1;
            end
        end
        disp(contvazio)
        if contvazio < N
            Rede.mSendMsg(P(ID),tR);
            Rede.mReceiveMsg;
            disp('Waiting for message......')
            tRede = tic;
        elseif contvazio == N
            Rede.mSendMsg(P(ID),tR);
            Rede.mReceiveMsg;
            disp('Waiting other robots.....')
            tR = 1;
        end
        if conttime == N
%             pause(toc(tRede)
            break
        end
%     end
end
clc
disp('Data received. Continuing the program...');

%% Definindo as constantes
Qtil = Qd - Qi;
intQtil = 0;
% Ganho dos controladores
K1 = 1*diag([0.5, 0.5]);
K2 = 1*diag([0.5, 0.5]);
% K1 = diag([.7, .7]);
% K2 = diag([.8, .8]);
L = 0.1*diag([.5, .5, .5, .5]);
Linv = inv(L);
kp = diag([1, 1, 1, 1]);

%% Desvio com memória
% fator de esquecimento
fe = 0.95;
gamma = 0;

%% Exibição do gráfico
tm = 60;
w = 1/tm;
figure(1)
hold on
grid on
title('Simulação');
axis([-5 5 -5 5])
for ii = 1:N
    Dados{ii} = [];
end

%% Temporização
% tm = 50; % Tempo máximo
ts = 0.1; % Tempo de amostragem
texib = 0.1; % Tempo para exibição 
tS = 5; % Tempo de sincronização

t = tic; % Tempo corrente
tc = tic; % Tempo de controle
tp = tic; % Exibição da simulação ou experimento
tSi = tic; % Tempo corrente de sincronia
ct = 0;
ctS = 0;

cont = 1;
contf = 1;
regra = 1;
%% Sincronizando os robôs
Rede.mReceiveMsg;
pause(1)

%% Sincronização 1
% Sincronia em função do maior tempo
% while toc(tSi) < tS && ctS < tS
%    Rede.mSendMsg(P(ID),toc(tSi));
%    Rede.mReceiveMsg;
%    for ii = 1:N
%        if ii == 1
%            ctS = -1;
%        end
%        if Rede.pMSG.getFrom{ii}(end) > ctS
%            ctS = Rede.pMSG.getFrom{ii}(end);
%        end
%    end
%    disp(ctS)
% end
% fclose(instrfindall);
% Rede = 0;
% Rede = NetDataShare;
% Rede.pMSG.getFrom{N} = [];

%% Sincronização 2
% Sincronia em função do menor tempo
% while toc(tSi) < tS && ctS < tS
%    Rede.mSendMsg(P(ID),toc(tSi));
%    Rede.mReceiveMsg;
%    for ii = 1:N
%        if ii == 1
%            ctS = -1;
%        end
%        if Rede.pMSG.getFrom{ii}(end) < ctS
%            ctS = Rede.pMSG.getFrom{ii}(end);
%        end
%    end
%    disp(ctS)
% end

%% Conferindo se todos estao com o temporizador zerado
tW = tic;
disp('Iniciando a simulação');
while toc(tW) < 8
    Rede.mSendMsg(P(ID));
    Rede.mReceiveMsg; 
    pause(1)
    try
    if Rede.pMSG.getFrom{1}(end) == 0
        if Rede.pMSG.getFrom{2}(end) == 0
            if Rede.pMSG.getFrom{3}(end) == 0
                disp('Ok')
            end
        end
    end
    end
end

pause(1)
Rede.mReceiveMsg;

%% Temporização 2
% tm = 50; % Tempo máximo
ts = 0.1; % Tempo de amostragem
texib = 0.1; % Tempo para exibição 

t = tic; % Tempo corrente
tc = tic; % Tempo de controle
tp = tic; % Exibição da simulação ou experimento
ct = 0;


cont = 1;
contf = 1;
regra = 1;

%% Inicio da simulação
while toc(t) < tm && ct < tm
    if toc(tc) > ts
        tc = tic;
        tint = toc(tc);
        
        %% Trajetoria (Circunferencia)
        Qd(1,1) = 2*cos(2*pi*w*ct);
        Qd(2,1) = 2*sin(2*pi*w*ct);
        dQd(1,1) = -4*pi*w*sin(2*pi*w*ct);
        dQd(2,1) = 4*pi*w*cos(2*pi*w*ct);        
        
        %% Trajetoria (Lemniscata)
%         Qd(1,1) = 1.5*sin(2*pi*w*ct);
%         Qd(2,1) = 1*sin(2*2*pi*w*ct);
%         dQd(1,1) = 2*1.5*pi*w*cos(2*pi*w*ct);
%         dQd(2,1) = 2*2*pi*w*cos(2*2*pi*w*ct);        
        
        %% Posição desejada (Posição)
%         Qd(1:2,1) = [2 2]';
%         dQd(1:2,1) = [0 0]';

        %% Ler dados dos sensores
        P(ID).rGetSensorData;
        Rede.mSendMsg(P(ID),toc(t));

        
        %% Ler dados da rede
        Rede.mReceiveMsg;
        % Atribuindo valores para a rede
        for ii=1:N
            if ii ~= ID
                P(ii).pPos.Xd = Rede.pMSG.getFrom{ii}(2+(1:12));
                P(ii).pPos.X  = Rede.pMSG.getFrom{ii}(14+(1:12));
                P(ii).pSC.Ud  = Rede.pMSG.getFrom{ii}(26+(1:2));
                P(ii).pSC.U   = Rede.pMSG.getFrom{ii}(28+(1:2));
            end
        end
        
        %% Selecionar o maior tempo
        % Rotina para sincronizar os robôs com o maior tempo
        if toc(t) < 0
            ct = toc(t);
            tID = ID;
        else
            for ii = 1:N
                if ii == 1
                    ct = -1;
                end
                if Rede.pMSG.getFrom{ii}(end) > ct
                    ct = Rede.pMSG.getFrom{ii}(end);
                    tID = ii;
                    if ct > tm
                        break
                    end
                end
            end
        end
        disp(ct)
        disp(tID)
        
        %% Calculando a posiçao desejada para cada formaçao
        if N>2
            for ii=2:N-1
                Qd(:,ii) = [Qd(1,ii-1)+Qd(3,ii-1)*cos(Qd(4,ii-1));
                            Qd(2,ii-1)+Qd(3,ii-1)*sin(Qd(4,ii-1));
                                       Qd(3,ii-1);
                                   (pi-alfa)*(ii-1)];
                dQd(:,ii) = dQd(:,ii-1);
            end
        end
        %% Jacobiando inverso da posição desejada
        for ii=1:N-1
            Jinv1(:,(4*ii-3):(4*ii)) = [1 0 0 0;
                                        0 1 0 0;
                                        1 0 cos(Qd(4,ii)) -Qd(3,ii)*sin(Qd(4,ii));
                                        0 1 sin(Qd(4,ii)) Qd(3,ii)*cos(Qd(4,ii))];
        end
        
        %% Calculando a posição desejada dos robos usando a posição desejada da formação
        for ii=1:N-1
            Xd(:,ii) = [           Qd(1,ii); 
                                   Qd(2,ii);
                        Qd(1,ii) + Qd(3,ii)*cos(Qd(4,ii)); 
                        Qd(2,ii) + Qd(3,ii)*sin(Qd(4,ii))]; % Matriz [x1 y1 x2 y2]'
        end
        
        %% Calculando a velocidade desejada de cada robo
        for ii=1:N-1
            dXd(:,ii) = Jinv1(:,(4*ii-3):(4*ii))*dQd(:,ii);
        end
        
        %% Enviando para cada robo sua posição desejada
        for ii=1:N
            if ii<N
                P(ii).pPos.Xd(1:2) = [Xd(1,ii);Xd(2,ii)];
                P(ii).pPos.Xd(7:8) = [dXd(1,ii);dXd(2,ii)];
            else
                P(ii).pPos.Xd(1:2) = [Xd(3,ii-1);Xd(4,ii-1)];
                P(ii).pPos.Xd(7:8) = [dXd(3,ii-1);dXd(4,ii-1)];
            end
        end
        
        %% Escrever os dados recebidos dos sensores em forma matricial
        for ii=1:N-1
            X(:,ii) = [P(ii).pPos.X(1);
                       P(ii).pPos.X(2);
                       P(ii+1).pPos.X(1);
                       P(ii+1).pPos.X(2)];

            dX(:,ii) = [P(ii).pPos.X(7);
                        P(ii).pPos.X(8);
                        P(ii+1).pPos.X(7);
                        P(ii+1).pPos.X(8)];

            Q(:,ii) = [          (P(ii).pPos.X(1)+P(ii+1).pPos.X(1))/2;
                                 (P(ii).pPos.X(2)+P(ii+1).pPos.X(2))/2;
                 sqrt((P(ii+1).pPos.X(1)-P(ii).pPos.X(1))^2+(P(ii+1).pPos.X(2)-P(ii).pPos.X(2))^2)/2;
                  atan((P(ii+1).pPos.X(2)-P(ii).pPos.X(2))/(P(ii+1).pPos.X(1)-P(ii).pPos.X(1)))];
        end

        %% Calculando o erro do ponto de formação
        Qtil = Qd - Q;
                
        %% Controle da formação
        dQr = dQd + L*Qtil;
        
        intQtil = Qtil*.1;
        Qr = Qd + L*intQtil;
        
        %% Jacobiano inverso
        for ii=1:N-1
            Jinv(:,(4*ii-3):(4*ii)) = [1 0 0 0;
                                       0 1 0 0;
                                       1 0 cos(Qr(4,ii)) -Qr(3,ii)*sin(Qr(4,ii));
                                       0 1 sin(Qr(4,ii)) Qr(3,ii)*cos(Qr(4,ii))];
        end

        %% Aplicando o Jacobiano inverso na velocidade da formação para
        % achar a velocidade dos robos
        for ii=1:N-1
            dXd(:,ii) = Jinv(:,(4*ii-3):(4*ii))*dQr(:,ii);
        end
            
        %% Aplicando a função inversa na formação para achar a posição dos
        % robos
        for ii=1:N-1
            Xd(:,ii) = [           Qr(1,ii); 
                                   Qr(2,ii);
                        Qr(1,ii) + Qr(3,ii)*cos(Qr(4,ii)); 
                        Qr(2,ii) + Qr(3,ii)*sin(Qr(4,ii))];
        end
               
        %% Passando os dados para os robos
        for ii=1:N
            if ii<N
                P(ii).pPos.Xd(1:2) = Xd(1:2,ii);
            else
                P(ii).pPos.Xd(1:2) = Xd(3:4,ii-1);
            end
        end
        
        %% Calcular sinal de controle
        dXtil = dXd - dX;
        
        %% Aplicando o controlador no robô correspondente ao seu ID
        P(ID).pPos.Xtil = P(ID).pPos.Xd - P(ID).pPos.X;
        K = [cos(P(ID).pPos.X(6)) -P(ID).pPar.a*sin(P(ID).pPos.X(6));
            sin(P(ID).pPos.X(6)) P(ID).pPar.a*cos(P(ID).pPos.X(6))];
        A = tanh(K2*P(ID).pPos.Xtil(1:2));
        P(ID).pSC.Ud = K\(P(ii).pPos.Xd([7,8]) + K1*A);
        
%         for ii=1:N
%             P(ii).pPos.Xtil = P(ii).pPos.Xd - P(ii).pPos.X;
%             K{ii} = [cos(P(ii).pPos.X(6)) -P(ii).pPar.a*sin(P(ii).pPos.X(6));
%                      sin(P(ii).pPos.X(6)) P(ii).pPar.a*cos(P(ii).pPos.X(6))];
%             A{ii} = tanh(K2*P(ii).pPos.Xtil(1:2));
%             P(ii).pSC.Ud = K{ii}\(P(ii).pPos.Xd([7,8]) + K1*A{ii});
%         end
        
        for ii=1:N
            P(ii).rSendControlSignals;
        end
        Rede.mSendMsg(P(ID),toc(t));
        
        %% Plot da simulação
        % Plot do robo
        for ii = 1:N
            if ii == ID
                P(ii).mCADdel
                P(ii).mCADplot(1,'b')
            else
                P(ii).mCADdel
                P(ii).mCADplot(1,'r')
            end
        end
        try 
            delete(h)
        end 
        % Matriz de dados
        for ii = 1:N
            Dados{ii} = [Dados{ii};
                [P(ii).pPos.Xd' P(ii).pPos.X' P(ii).pPos.Xtil' P(ii).pSC.Ud' P(ii).pSC.Ur' P(ii).pSC.U' toc(t)]];
        end
        % Plot do destino
        for ii = 1:N
            h(2*ii-1) = plot(Dados{ii}(:,1), Dados{ii}(:,2),'--r');
            h(2*ii) = plot(Dados{ii}(:,13), Dados{ii}(:,14),'-k');
        end
        drawnow
    end
    %% Plotando a simulação
    if toc(tp) > texib
        
    end 
end
Rede.mSendMsg(P(ID),toc(t));

for ii=1:N
    P(ii).pSC.Ud = [0;0];
    P(ii).rSendControlSignals;
end
pause(2)
P(ID).rDisconnect;
