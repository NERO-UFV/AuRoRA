%% CONTROLE DE TRAJETÓRIA PARA FORMAÇÃO DE DOIS ROBÔS EM LINHA

clear
close all
clc
% Fecha todas possíveis conexões abertas
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Definição da janela do gráfico
fig = figure(1);
axis([-4 4 -4 4])
pause(1)

%% Criação dos robôs
idA = input('Digite o ID do robô: ');
if idA == 1
    idB = 2;
elseif idA == 2
    idB = 1;
end
% PB = Pioneer3DX(idB);

PA = Pioneer3DX(idA);
PB = Pioneer3DX;

%% Posição inicial dos robôs
Xo = input('Digite a posição inicial do robô ([x y z psi]): ');

%% Gravar arquivos de txt (ou excel)
% Apenas o robô 1 irá gravar os dados para evitar redundância
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_FormacaoLinha2D')
Arq = fopen(['FL2dtrajLemniSim_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Conexão com robô/simulador
PA.rConnect;             % robô ou mobilesim
PA.rSetPose(Xo);         % define pose do robô
% pause(3)
disp('Início..............')

%% Definição da Formação
% centro de referência da formação:
% center = ponto médio entre os robôs
% robot  = referência no robô 1
% type = 'center';
type = 'center';
LF = LineFormation2D(type);       % carrega classe formação 2D
LF.pPos.Qd = [0 0 1 0]'; % define formação desejada

% LF.pPar.K1 = 1*diag([.4 .4 0.6 1.5]);          % ganho controlador cinemático
% LF.pPar.K2 = 1*diag([.5 .5 .5 .5]);              % ganho controlador cinemático

%% Criação da rede
Rede = NetDataShare;

%% Network communication check
disp('Begin network check......')
tm = tic;
while true
    Rede.mSendMsg(PA);
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(PA);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 1
        if isempty(Rede.pMSG.getFrom{idB})
            tm = tic;
            Rede.mSendMsg(PA);           
            Rede.mReceiveMsg;
            disp('Waiting for message......')
            
        else
            break
        end
    end
    Rede.mReceiveMsg;
end
clc
disp('Data received. Proceding program...');

%% Inicialização de variáveis
Xa = PA.pPos.X(1:6);    % postura anterior
data = [];
Rastro.Qd = [];
Rastro.Q = [];

%% Trajetória
a = 3;         % distância em x
b = 1.5;         % distância em y
w = 0.1;

nvoltas = 1.5;
tsim = 2*pi*nvoltas/w;
tap = .1;     % taxa de atualização do pioneer

%% Cálculo do erro inicial da formação
if length(Rede.pMSG.getFrom) >= 1
    if PA.pID > 1       % Caso robo seja ID = 2
        
        % pegar primeiro robo
        if ~isempty(Rede.pMSG.getFrom{1})
            
            Xa = Rede.pMSG.getFrom{1}(14+(1:6)); % robô 1
            Xb = PA.pPos.X((1:6));                % robô 2

%             PB.rSetPose(Xa([1 2 3 6]));        % define pose do robô B
            
            % Posição inicial dos robôs
            LF.pPos.X = [Xa; Xb];
            
        end
    else    % Caso o robô seja o ID = 1
        
        if length(Rede.pMSG.getFrom) == PA.pID+1   % caso haja dados dos dois robôs na rede
            
            Xa = PA.pPos.X(1:6);                         % robô 1
            Xb = Rede.pMSG.getFrom{PA.pID+1}(14+(1:6));  % robô 2
            
%             PB.rSetPose(Xb([1 2 3 6]));        % define pose do robô B

            % Posição inicial dos robôs
            LF.pPos.X = [Xa; Xb];
            
        end
    end
end

% Posição inicial da formação
LF.mDirTrans;

% Erro da formação
LF.pPos.Qtil = LF.pPos.Qd - LF.pPos.Q;
% Tratamento de quadrante
if abs(LF.pPos.Qtil(4)) > pi
    if LF.pPos.Qtil(4) > 0
        LF.pPos.Qtil(4) = -2*pi + LF.pPos.Qtil(4);
    else
        LF.pPos.Qtil(4) =  2*pi + LF.pPos.Qtil(4);
    end
end

%% Simulação

% Temporização
timeout = 120;   % tempo máximo de duração da simulação
t = tic;
tc = tic;
tp = tic;

while toc(t) < tsim
    
    if toc(tc) > tap
        
        tc = tic;
        
        % Calculo da trajetória
        %         % infinito (8')
                 ta = toc(t);
                LF.pPos.Qd(1)  = a*sin(w*ta);       % posição x
                LF.pPos.Qd(2)  = b*sin(2*w*ta);     % posição y
                LF.pPos.dQd(1) = a*w*cos(w*ta);     % velocidade em x
                LF.pPos.dQd(2) = 2*b*w*cos(2*w*ta); % velocidade em y
       
        % Angulo da formação seguindo a trajetoria
        LF.pPos.Qd(4) = atan2(LF.pPos.dQd(2),LF.pPos.dQd(1)) + pi/2;        
        LF.pPos.dQd(4) = 1/(1+(LF.pPos.dQd(2)/LF.pPos.dQd(1))^2)* ...
           ((LF.pPos.dQd(2)*LF.pPos.Qd(1)-LF.pPos.Qd(2)*LF.pPos.dQd(1))/LF.pPos.Qd(1)^2);
    
       %         círculo
%         ta = toc(t);
%         LF.pPos.Qd(1)  = a*cos(w*ta);    % posição x
%         LF.pPos.Qd(2)  = b*sin(w*ta);    % posição y
%         LF.pPos.dQd(1) = -a*w*sin(w*ta);  % velocidade em x
%         LF.pPos.dQd(2) = b*w*cos(w*ta); % velocidade em y
        
        % ângulo da formação seguindo a trajetória
%         LF.pPos.Qd(4) = w*ta; 
%         LF.pPos.dQd(4) = ta; 
        
        % salva variáveis para plotar no gráfico
        Rastro.Qd = [Rastro.Qd; LF.pPos.Qd(1:2)'];  % formação desejada
        Rastro.Q  = [Rastro.Q; LF.pPos.Q(1:2)'];    % formação real
        
        % Pega dados dos sensores
        PA.rGetSensorData;
        
        % Lê dados da rede
        Rede.mReceiveMsg;
        
        % Buscar SEMPRE robô com ID+1 para formação
        % Variável robôs da rede
        % Verifica se há mensagem
        %% Controle de formação
        if length(Rede.pMSG.getFrom) >= 1
            % Caso robo seja ID = 2 .....................................
            if  PA.pID > 1
                
                % pegar primeiro robo
                if ~isempty(Rede.pMSG.getFrom{1})
                    
                    Xa = Rede.pMSG.getFrom{1}(14+(1:6));    % robô 1
                    Xb = PA.pPos.X(1:6);                     % robô 2
                    
                    % Pose da formação
                    LF.pPos.X = [Xa; Xb];
                    
                    % Controlador da formação
                    LF.mFormationControl;
                    
                    % Posição desejada
                    LF.mInvTrans('d');  % obtem posições desejadas dos robôs
                    PA.pPos.Xd(1:2) = LF.pPos.Xd(3:4);
                    
                    % Atribui sinal de controle ao robô
                    PA.sInvKinematicModel(LF.pPos.dXr(3:4));
                    
                    % Atribuindo valores desejados do controle de formação
                    PB.pPos.Xd = Rede.pMSG.getFrom{1}(2+(1:12));
                    PB.pPos.X  = Rede.pMSG.getFrom{1}(14+(1:12));
                    PB.pSC.Ud  = Rede.pMSG.getFrom{1}(26+(1:2));
                    PB.pSC.U   = Rede.pMSG.getFrom{1}(28+(1:2));
                    
                    % Postura do centro do robô
                    PB.pPos.Xc([1 2 6]) = PB.pPos.X([1 2 6]) - ...
                        [PB.pPar.a*cos(PB.pPos.X(6)); PB.pPar.a*sin(PB.pPos.X(6)); 0];
                    
                    % Armazenar dados no arquivo de texto
                    %                     if ID==1
                    fprintf(Arq,'%6.6f\t',[PA.pPos.Xd' PA.pPos.X' PA.pSC.Ud' PA.pSC.U' ...
                        PB.pPos.Xd' PB.pPos.X' PB.pSC.Ud' PB.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
                    fprintf(Arq,'\n\r');
                    %                     end
                end
                
                % Caso o robô seja o ID = 1 ..................................
            else
                
                if length(Rede.pMSG.getFrom) == PA.pID+1   % caso haja dados dos dois robôs na rede
                    
                    Xa = PA.pPos.X(1:6);                         % robô 1
                    Xb = Rede.pMSG.getFrom{PA.pID+1}(14+(1:6));  % robô 2
                    
                    % Posição dos robôs
                    LF.pPos.X = [Xa; Xb];
                    
                    % Controlador da formação
                    LF.mFormationControl;
                    
                    % Posição desejada
                    LF.mInvTrans('d');  % obtem posições desejadas dos robôs
                    PA.pPos.Xd(1:2) = LF.pPos.Xd(1:2);
                    
                    % Atribui sinal de controle
                    PA.sInvKinematicModel(LF.pPos.dXr(1:2));
                    
                    % Atribuindo valores desejados do controle de formação
                    PB.pPos.Xd = Rede.pMSG.getFrom{PA.pID+1}(2+(1:12));
                    PB.pPos.X  = Rede.pMSG.getFrom{PA.pID+1}(14+(1:12));
                    PB.pSC.Ud  = Rede.pMSG.getFrom{PA.pID+1}(26+(1:2));
                    PB.pSC.U   = Rede.pMSG.getFrom{PA.pID+1}(28+(1:2));
                    
                    % Postura do centro do robô
                    PB.pPos.Xc([1 2 6]) = PB.pPos.X([1 2 6]) - ...
                        [PB.pPar.a*cos(PB.pPos.X(6)); PB.pPar.a*sin(PB.pPos.X(6)); 0];
                    
                    % Armazenar dados no arquivo de texto
                 
                    fprintf(Arq,'%6.6f\t',[PA.pPos.Xd' PA.pPos.X' PA.pSC.Ud' PA.pSC.U' ...
                        PB.pPos.Xd' PB.pPos.X' PB.pSC.Ud' PB.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
                    fprintf(Arq,'\n\r');
               
                end
            end
        end
        
        %% Compensação dinâmica
        % Compensador dinâmico
        PA = fCompensadorDinamico(PA);

      
        % variável para plotar gráficos
        data = [data; PA.pPos.Xd' PA.pPos.X' PA.pSC.Ud' PA.pSC.U' ...
            PB.pPos.Xd' PB.pPos.X' PB.pSC.Ud' PB.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)];
        
        % Publicar mensagem na rede
        Rede.mSendMsg(PA);
        
        %% Envia sinais de controle
        PA.rSendControlSignals;
        
    end
    
    % Desenha os robôs
    
    if toc(tp) > tap
        tp = tic;
        try
            PA.mCADdel
            PB.mCADdel
            delete(fig);
            delete(fig2);
        catch
        end
        PA.mCADplot2D('b')
        PB.mCADplot2D('r')
        hold on
        fig = plot(Rastro.Qd(:,1),Rastro.Qd(:,2),'k');
        fig2 = plot(Rastro.Q(:,1),Rastro.Q(:,2),'g');
        axis([-8 8 -8 8])
        grid on
        drawnow
    end
    
end

%% Fechar e Parar
% if ID==1
fclose(Arq);
% end

% Zera velocidades do robô
PA.pSC.Ud = [0 ; 0];
PA.rSendControlSignals;

%% Resultados
% Verifica qual robô é qual para que os gráficos fiquem iguais
%  nos dois computadores
if PA.pID == 1
    plotResultsTraj(PA,PB,LF,data);
elseif PA.pID == 2
    plotResultsTraj(PB,PA,LF,data);
end