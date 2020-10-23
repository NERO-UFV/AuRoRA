clear
close all
clc

try
    fclose(instrfindall)
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Gravar arquivos de txt (ou excel)
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_FormacaoLinha2D')
Arq = fopen(['FL2dposExp_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Exibição do gráfico
fig = figure(1);
axis([-8 8 -8 8])
pause(1)

%% Criação das classes
% Robos
ID = input('Digite o ID do robô: ');
PA = Pioneer3DX(ID);
PB = Pioneer3DX;

%% Posição inicial dos robôs
Xo = input('Digite a posição inicial do robô ([x y psi]): ');

%% Conexão com robô/simulador
% A.rConnect;           
PA.rSetPose(Xo');
shg
pause(7)
disp('Início..............')

%% Controle de Formaçao
LF = LineFormation2D('robot');
xsq = 1.2;
ysq = 1.24;
LF.pPos.Qd = [3*xsq 3*ysq xsq 0]';
% LF.pPos.Qd = [4 2 1 deg2rad(0)]';

% Criação da rede

tm = tic;
% Verifica se alguma informação da rede foi recebida
while isempty(Rede.pMSG.getFrom)
    Rede.mSendMsg(PA);
    if toc(tm) > 0.1
        tm = tic;
        Rede.mReceiveMsg;
        
    end
end
disp('Envio com sucesso.....');

%% Posições desejadas
Qd = [  1   0   0   1.5    0   pi/2;
       -1   1   0   2      0   pi/2;
       -1  -1   0   2      0   pi/2;
        0   0   0   1.5    0   pi/2];
cont = 0;     % counter to change desired position through simulation
time = 15;    % time to change desired positions [s]
% First desired position
LF.pPos.Qd = Qd(1,:)';

%% Cálculo do erro inicial da formação

if length(Rede.pMSG.getFrom) >= 1
    if PA.pID > 1       % Caso robo seja ID = 2
        
        % pegar primeiro robo
        if ~isempty(Rede.pMSG.getFrom{1})
            
            Xa = Rede.pMSG.getFrom{1}(14+(1:6)); % robô 1
            Xb = PA.pPos.X((1:6));                % robô 2
            
            % Posição inicial dos robôs
            LF.pPos.X = [Xa; Xb];
            
        end
    else    % Caso o robô seja o ID = 1
        
        if length(Rede.pMSG.getFrom) == PA.pID+1   % caso haja dados dos dois robôs na rede
            
            Xa = PA.pPos.X(1:6);                       % robô 1
            Xb = Rede.pMSG.getFrom{PA.pID+1}(14+(1:6));  % robô 2
            
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

erroMax = [.05 .05 .1 deg2rad(3)]; % vetor do erro de formação
% Temporização
t  = tic;
tc = tic;
tp = tic;
timeout = 60;   % tempo máximo de duração da simulação

while abs(LF.pPos.Qtil(1))>erroMax(1) || abs(LF.pPos.Qtil(2))>erroMax(2) || ...
        abs(LF.pPos.Qtil(3))>erroMax(3) || abs(LF.pPos.Qtil(4))>erroMax(4)  % erros de formação
           
    if toc(tc) > 0.1      
        tcc = tic;
        
        
        
        
        PA.rGetSensorData
        % Informações da Rede
        Rede.mReceiveMsg;
        
        % Buscar SEMPRE robô com ID+1 para formação
        % Variável robôs da rede
        % Verifica se há mensagem
        %% Controle de formação
        if length(Rede.pMSG.getFrom) >= 1
            % Caso robo seja ID = 2 .....................................
            if PA.pID > 1
                
                % pegar primeiro robo
                if ~isempty(Rede.pMSG.getFrom{1})
                    
                    Xa = Rede.pMSG.getFrom{1}(14+(1:6));    % robô 1
                    Xb = PA.pPos.X(1:6);                     % robô 2
                    
                    % Robot Position
                    LF.pPos.X = [Xa; Xb];
                    
                    % Formation Controller
                    LF.mFormationControl;
                    
                    %% ................................................
                    %% Robot Control Velocity
                    LF.mInvTrans('d');
                    PA.pPos.Xd(1:2) = LF.pPos.Xd(1:2); 
                    PA.sInvKinematicModel(LF.pPos.dXr(3:4));                  % Calculate control signals  
                                      
                    % Posição desejada
                    LF.mInvTrans;
                    PA.pPos.Xd(1:2) = LF.pPos.Xd(1:2);   % posições desejadas
                    PA.pPos.Xd(7:8) = LF.pPos.dXr(1:2);  % velocidades desejadas
                                       
                    % Atribuindo valores desejados do controle de formação
                    PB.pPos.Xd = Rede.pMSG.getFrom{1}(2+(1:12));
                    PB.pPos.X  = Rede.pMSG.getFrom{1}(14+(1:12));
                    PB.pSC.Ud  = Rede.pMSG.getFrom{1}(26+(1:2));
                    PB.pSC.U   = Rede.pMSG.getFrom{1}(28+(1:2));
                    
                    % Postura do centro do robô
                    PB.pPos.Xc([1 2 6]) = PB.pPos.X([1 2 6]) + ...
                        [PB.pPar.a*cos(PB.pPos.X(6)); PB.pPar.a*sin(PB.pPos.X(6)); 0];
                    
                    % Armazenar dados no arquivo de texto
                    fprintf(Arq,'%6.6f\t',[PA.pPos.Xd' PA.pPos.X' PA.pSC.Ur' PA.pSC.U' ...
                        PB.pPos.Xd' PB.pPos.X' PB.pSC.Ur' PB.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
                    fprintf(Arq,'\n\r');
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
                   
                    %% ................................................
                    %% Robot Control Velocity
                    LF.mInvTrans('d');
                    PA.pPos.Xd(1:2) = LF.pPos.Xd(1:2);   % posições desejadas
                    PA.pPos.Xd(7:8) = LF.pPos.dXr(1:2);  % velocidades desejadas
                    
                    % Atribuindo valores desejados do controle de formação                    
                    PB.pPos.Xd = Rede.pMSG.getFrom{PA.pID+1}(2+(1:12));
                    PB.pPos.X  = Rede.pMSG.getFrom{PA.pID+1}(14+(1:12));
                    PB.pSC.Ud  = Rede.pMSG.getFrom{PA.pID+1}(26+(1:2));
                    PB.pSC.U   = Rede.pMSG.getFrom{PA.pID+1}(28+(1:2));
                    
                    % Postura do centro do robô
                    PB.pPos.Xc([1 2 6]) = PB.pPos.X([1 2 6]) + ...
                        [PB.pPar.a*cos(PB.pPos.X(6)); PB.pPar.a*sin(PB.pPos.X(6)); 0];
                    
                    % Armazenar dados no arquivo de texto
                    fprintf(Arq,'%6.6f\t',[PA.pPos.Xd' PA.pPos.X' PA.pSC.Ud' PA.pSC.U' ...
                        PB.pPos.Xd' PB.pPos.X' PB.pSC.Ud' PB.pSC.U' LF.pPos.Qd' LF.pPos.Qtil' toc(t)]);
                    fprintf(Arq,'\n\r');
                end
            end
        end

        %% Controlador dinâmico
        PA = fDynamicController; 
        %% Envia sinais de controle
        PA.rSendControlSignals;
        
        % Publicar mensagem na rede
        Rede.mSendMsg(PA);
        
    end
    % Desenha os robôs
    if toc(tp) > 0.1
        tp = tic;
        
        PA.mCADdel
        PB.mCADdel
        
        PA.mCADplot2D('b')
        PB.mCADplot2D('r')
        
        grid on
        drawnow
    end
    
    % Limita tempo de simulação
    if toc(t)> timeout
        disp('Tempo limite atingido.');
        break
    end
    
end

%% Fechar e Parar
fclose(Arq);

% Envia velocidades 0 para o robô
PA.pSC.Ur = [0 ; 0];
PA.rSendControlSignals;
        
