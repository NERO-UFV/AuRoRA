clear
close all
clc

try
    fclose(instrfindall)
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Gravar arquivos de txt (ou excel)
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_FormacaoLinha2D')
Arq = fopen(['FL2d_' NomeArq '.txt'],'w');
cd(PastaAtual)

% Exibição do gráfico
fig = figure(1);
axis([-5 8 -8 8])
pause(1)

nRobos = 2;

ID = input('Digite o ID do robô: ');
A = Pioneer3DX(ID);
B = Pioneer3DX;

% A.mConectar
% A.mConectarMobileSim
% pause(5)

% Posição inicial dos robôs
X = input('Digite a posição inicial do robô: ');
A.pPos.X(1:2) = [X(1) ; X(2)];

% Controle de Formaçao
LF = LineFormation2D;
LF.pPos.Qd = [2 0 1 0]';

% Criação da rede
Rede = NetDataShare;
tm = tic;
while isempty(Rede.pMSG.getFrom)
    Rede.mSendMsg(A);
    if toc(tm) > 0.1
        tm = tic;
        Rede.mReceiveMsg;
        %         Rede.pMSG.getFrom
    end
end
disp('Envio com sucesso.....')
% Rede.pMSG.getFrom

if A.pID == nRobos
    %     Rede.pMSG.getFrom
    tm = tic;
    while isempty(Rede.pMSG.getFrom{1})
        Rede.mSendMsg(A);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            %             Rede.pMSG.getFrom
        end
    end
else
    %     Rede.pMSG.getFrom
    tm = tic;
    while length(Rede.pMSG.getFrom) < A.pID+1
        Rede.mSendMsg(A);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            %             Rede.pMSG.getFrom
        end
    end
    %     length(Rede.pMSG.getFrom)
end
disp('Resposta com sucesso.....')

Xa = A.pPos.X([1 2 7 8]);

U = [];
ke = 0;
kr = 0;

% Temporização
t = tic;
tc = tic;
tp = tic;
while toc(t) < 400
if toc(tc) > 0.1
        tcc = tic;
        
        % Informações da Rede
        Rede.mReceiveMsg;
        
        % Buscar SEMPRE robô com ID+1 para formação
        % Variável robôs da rede
        % Verifica se há mensagem
        
        if length(Rede.pMSG.getFrom) >= 1
            if A.pID > 1
                % pegar primeiro robo
                if ~isempty(Rede.pMSG.getFrom{1})
                    kr = kr + 1;
                    disp([ke kr])
                    
                    Xa = Rede.pMSG.getFrom{1}(14+[1 2 7 8]);
                    Xb = A.pPos.X([1 2 7 8]);
                    % Xb = Rede.pMSG.getFrom{A.pID}(14+[1 2 7 8]);
                    
                    [Xa Xb]
                    
                    [Xda,Xdb] = mFormationControl(LF,Xa,Xb);
                    
                    % Atribuindo valores desejados do controle de formação
                    A.pPos.Xd([1 2 7 8]) = Xdb;
                    B.pPos.Xc([1 2 7 8]) = Xa;
                    
                    B.pPos.Xd = Rede.pMSG.getFrom{1}(2+(1:12));
                    B.pPos.X  = Rede.pMSG.getFrom{1}(14+(1:12));
                    B.pSC.Ur  = Rede.pMSG.getFrom{1}(26+(1:2));
                    B.pSC.U   = Rede.pMSG.getFrom{1}(28+(1:2));
                    
                    B.pPos.Xc([1 2 6]) = B.pPos.X([1 2 6]) + ...
                        [B.pPar.a*cos(B.pPos.X(6)); B.pPar.a*sin(B.pPos.X(6)); 0];
                    
                    % Armazenar dados no arquivo de texto
                    fprintf(Arq,'%6.6f\t',[A.pPos.Xd' A.pPos.X' A.pSC.Ur' A.pSC.U' ...
                        B.pPos.Xd' B.pPos.X' B.pSC.Ur' B.pSC.U' toc(t)]);
                    fprintf(Arq,'\n\r');
                end
            else
                if length(Rede.pMSG.getFrom) == A.pID+1
                    % ~isempty(Rede.pMSG.getFrom{A.pID+1})
                    
                    disp([ke kr])
                    
                    Xa = A.pPos.X([1 2 7 8]);
                    % Xa = Rede.pMSG.getFrom{A.pID}(14+[1 2 7 8]);
                    Xb = Rede.pMSG.getFrom{A.pID+1}(14+[1 2 7 8]);
                    % Rede.pMSG.getFrom{2}'
                    
                    [Xa Xb]
                    
                    [Xda,Xdb] = mFormationControl(LF,Xa,Xb);
                    % Atribuindo valores desejados do controle de formação
                    A.pPos.Xd([1 2 7 8]) = Xda;
                    B.pPos.Xd = Rede.pMSG.getFrom{A.pID+1}(2+(1:12));
                    B.pPos.X  = Rede.pMSG.getFrom{A.pID+1}(14+(1:12));
                    B.pSC.Ur  = Rede.pMSG.getFrom{A.pID+1}(26+(1:2));
                    B.pSC.U   = Rede.pMSG.getFrom{A.pID+1}(28+(1:2));
                    
                    B.pPos.Xc([1 2 6]) = B.pPos.X([1 2 6]) + ...
                        [B.pPar.a*cos(B.pPos.X(6)); B.pPar.a*sin(B.pPos.X(6)); 0];
                    
                    % Armazenar dados no arquivo de texto
                    fprintf(Arq,'%6.6f\t',[A.pPos.Xd' A.pPos.X' A.pSC.Ur' A.pSC.U' ...
                        B.pPos.Xd' B.pPos.X' B.pSC.Ur' B.pSC.U' toc(t)]);
                    fprintf(Arq,'\n\r');
                end
            end
        end
        
        % Ler Sinais de Controle
        A.mLerDadosSensores;
        
        % Controle dos robôs
        A = fControladorCinematico(A);
        
        % Envia sinais de controel
        A.mEnviarSinaisControle;
        
        % Publicar mensagem na rede
        Rede.mSendMsg(A);
        ke = ke + 1;
        
    end
    if toc(tp) > 0.1
        tp = tic;
        
        A.mCADdel
        B.mCADdel
        
        A.mCADplot2D('b')
        B.mCADplot2D('r')
        
        grid on
        drawnow
    end
end

%% Fechar e Parar
fclose(Arq);

% Envia velocidades 0 para o robô
A.pSC.Ur = [0 ; 0];
A.mEnviarSinaisControle;