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

%% ID do robo
ID = 3;

%% Definindo a Rede
Rede = NetDataShare;
% while isempty(Rede.pMSG.getFrom)
%     Rede.mReceiveMsg;
%     pause(2)
%     N = size(Rede.pMSG.getFrom,2);
% end
%% Criando robo
P(ID) = Pioneer3DX(ID);

%% Conectando o robô
P(ID).rConnect;

%% Testando a conexão da rede
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P(ID));
        if toc(tm) > 1/30
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
        
    elseif length(Rede.pMSG.getFrom) > 1
        
        if isempty(Rede.pMSG.getFrom{1})
            Rede.mSendMsg(P(ID));
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        else
            break
        end
    end
end
clc
disp('Data received. Continuing the program...');

N = size(Rede.pMSG.getFrom,2);
for ii = 1:N
    if ii ~= ID
        P(ii) = Pioneer3DX(ii);
    end
end

%% Definindo a Figura que irá rodar a simulação
figure(1)
hold on
grid on
title('Simulação');
axis([-5 5 -5 5])

%% Recebendo dados e armazenando para teste
for i = 1:N
    Dados{i} = [];
end
Rede.mReceiveMsg;

%% Definindo a posição inicial
for ii = 1:N
    P(ii).pPos.Xd = Rede.pMSG.getFrom{ii}(2+(1:12));
    P(ii).pPos.X  = Rede.pMSG.getFrom{ii}(14+(1:12));
    P(ii).pSC.Ud  = Rede.pMSG.getFrom{ii}(26+(1:2));
    P(ii).pSC.U   = Rede.pMSG.getFrom{ii}(28+(1:2));
end
P(ID).rSetPose([P(ID).pPos.X(1) P(ID).pPos.X(2) 0 0]);
disp([P(ID).pPos.X(1) P(ID).pPos.X(2) 0 0])
%% Temporização
tmax = 60; %Tempo máximo da simulação
ts = 0.1; %Tempo de amostragem

t = tic; %Tempo corrente
tc = tic; %Tempo de controle

%% Inicio da simulação
% while ~isempty(Rede.pMSG.getFrom)
while toc(t) < tmax
    if toc(tc) > ts
        tc = tic;
        Rede.mReceiveMsg
        for ii = 1:N
            P(ii).pPos.Xd = Rede.pMSG.getFrom{ii}(2+(1:12));
            P(ii).pPos.X  = Rede.pMSG.getFrom{ii}(14+(1:12));
            P(ii).pSC.Ud  = Rede.pMSG.getFrom{ii}(26+(1:2));
            P(ii).pSC.U   = Rede.pMSG.getFrom{ii}(28+(1:2));
        end
        
        P(ID).rGetSensorData;
        Rede.mSendMsg(P(ID));
        
        for ii = 1:N
            P(ii).rSendControlSignals;
        end
        
        %% Plot da simulação
        % Plot do robo
        for ii = 1:N
            P(ii).mCADdel
            P(ii).mCADplot(1,'r')
        end
        try
            delete(h)
        end
        % Matriz de dados
        for ii = 1:N
            Dados{ii} = [Dados{ii};
                [P(ii).pPos.Xd' P(ii).pPos.X' P(ii).pPos.Xtil' P(ii).pSC.Ud' P(ii).pSC.Ur' P(ii).pSC.U']];
        end
        % Plot do destino
        for ii = 1:N
            h(2*ii-1) = plot(Dados{ii}(:,1), Dados{ii}(:,2),'--r');
            h(2*ii)   = plot(Dados{ii}(:,13), Dados{ii}(:,14),'-k');
        end
        drawnow
    end
end

for ii=1:N
    P(ii).pSC.Ud = [0;0];
    P(ii).rSendControlSignals;
end
pause(2)
% P(2).rDisconnect