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
Arq = fopen(['FL2dDesvio_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Exibição do gráfico
fig = figure(1);
axis([-5 10 -5 5])
pause(1)

%% Criação dos robôs
nRobos = 2;

ID = input('Digite o ID do robô: ');
A = Pioneer3DX(ID);
B = Pioneer3DX;

A.mJoystick;  % Verifica se joystick está conectado

%% Posição inicial dos robôs
Xo = input('Digite a posição inicial do robô ([x y psi]): ');
% Xo = [0 -1.25 0];
% A.pPos.X([1 2 6]) = [X(1) ; X(2); X(3)];

%% Conexão com robô/simulador
A.mConectar;            % robô||simulador
A.mDefinirPosturaInicial(Xo');
shg
pause(3)
disp('Início..............')

%% Controle de Formaçao
LF = LineFormation2DCompensador('center');
xsq = .89;%1.2;
ysq = .89;%1.24;
LF.pPos.Qd = [9*xsq 0*ysq xsq pi/2]';
% LF.pPos.Qd = [4 2 1 deg2rad(0)]';

%% Criação da rede
Rede = NetDataShare;
tm = tic;
% Verifica se alguma informação da rede foi recebida
while isempty(Rede.pMSG.getFrom)
    Rede.mSendMsg(A);
    if toc(tm) > 0.1
        tm = tic;
        Rede.mReceiveMsg;
        
    end
end
disp('Envio com sucesso.....');

%% Inicialização de variáveis
Xa = A.pPos.X(1:6);

U = [];
ke = 0;
kr = 0;
acc = [];   % salva aceleração calculada

% Temporização
t = tic;
tc = tic;
tp = tic;
timeout = 120;   % tempo máximo de duração da simulação

%% Cálculo do erro inicial da formação

if length(Rede.pMSG.getFrom) >= 1
    if A.pID > 1       % Caso robo seja ID = 2
        
        % pegar primeiro robo
        if ~isempty(Rede.pMSG.getFrom{1})
            
            Xa = Rede.pMSG.getFrom{1}(14+(1:6)); % robô 1
            Xb = A.pPos.X((1:6));                % robô 2
            
            % Posição inicial dos robôs
            LF.pPos.X = [Xa; Xb];
            
        end
    else    % Caso o robô seja o ID = 1
        
        if length(Rede.pMSG.getFrom) == A.pID+1   % caso haja dados dos dois robôs na rede
            
            Xa = A.pPos.X(1:6);                       % robô 1
            Xb = Rede.pMSG.getFrom{A.pID+1}(14+(1:6));  % robô 2
            
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

erroMax = [.05 .05 .05 deg2rad(3)]; % vetor do erro de formação

while abs(LF.pPos.Qtil(1))>erroMax(1) || abs(LF.pPos.Qtil(2))>erroMax(2) || ...
        abs(LF.pPos.Qtil(3))>erroMax(3) || abs(LF.pPos.Qtil(4))>erroMax(4)  % erros de formação
      
    %% Desvio
    % Mudança do ângulo da formação para desvio de obstáculo
    if LF.pPos.Q(1)>=LF.pPos.Qd(1)*.65  % obstáculo a X% do caminho
        
        LF.pPos.Qd(4) = -pi/2;
        
    elseif LF.pPos.Q(1)>=LF.pPos.Qd(1)*.25 % 
        
        LF.pPos.Qd(4) = 0;    
        
    end
    %%
    
    if toc(tc) > 0.1
        
        tcc = tic;

        % Ler Sinais de Controle
        A.mLerDadosSensores;
        
        % Informações da Rede
        Rede.mReceiveMsg;
        
        % Buscar SEMPRE robô com ID+1 para formação
        % Variável robôs da rede
        % Verifica se há mensagem
        %% Controle de formação
        if length(Rede.pMSG.getFrom) >= 1
            % Caso robo seja ID = 2 .....................................
            if A.pID > 1
                
                % pegar primeiro robo
                if ~isempty(Rede.pMSG.getFrom{1})
                    %                     kr = kr + 1;
                    %                     disp([ke kr])
                    
                    Xa = Rede.pMSG.getFrom{1}(14+(1:6));    % robô 1
                    Xb = A.pPos.X(1:6);                     % robô 2
                    
                    % Posição dos robôs
                    LF.pPos.X = [Xa; Xb];
                    
                    % Controlador da formação
                    LF.mFormationControl;
                    
                    % Posição desejada
                    A.pPos.Xd(1:2) = LF.pPos.Xr(3:4);
                    
                    % Atribui sinal de controle ao robô                                     
                    A.pSC.Ud = LF.pSC.Ud(3:4);
                    
                    % Atribuindo valores desejados do controle de formação
                    B.pPos.Xd = Rede.pMSG.getFrom{1}(2+(1:12));
                    B.pPos.X  = Rede.pMSG.getFrom{1}(14+(1:12));
                    B.pSC.Ur  = Rede.pMSG.getFrom{1}(26+(1:2));
                    B.pSC.U   = Rede.pMSG.getFrom{1}(28+(1:2));
                    
                    % Postura do centro do robô
                    B.pPos.Xc([1 2 6]) = B.pPos.X([1 2 6]) + ...
                        [B.pPar.a*cos(B.pPos.X(6)); B.pPar.a*sin(B.pPos.X(6)); 0];
                    
                    % Armazenar dados no arquivo de texto
                    fprintf(Arq,'%6.6f\t',[A.pPos.Xd' A.pPos.X' A.pSC.Ur' A.pSC.U' ...
                        B.pPos.Xd' B.pPos.X' B.pSC.Ur' B.pSC.U' toc(t)]);
                    fprintf(Arq,'\n\r');
                end
                
                % Caso o robô seja o ID = 1 ..................................
            else
                
                if length(Rede.pMSG.getFrom) == A.pID+1   % caso haja dados dos dois robôs na rede
                    
                    Xa = A.pPos.X(1:6);                         % robô 1
                    Xb = Rede.pMSG.getFrom{A.pID+1}(14+(1:6));  % robô 2
                    
                    % Posição dos robôs
                    LF.pPos.X = [Xa; Xb];
                    
                    % Controlador da formação
                    LF.mFormationControl;
                    
                    % Posição desejada
                    A.pPos.Xd(1:2) = LF.pPos.Xr(1:2);
                   
                    % Atribui sinal de controle                 
                    A.pSC.Ud = LF.pSC.Ud(1:2);     
                    
                    % Atribuindo valores desejados do controle de formação                    
                    B.pPos.Xd = Rede.pMSG.getFrom{A.pID+1}(2+(1:12));
                    B.pPos.X  = Rede.pMSG.getFrom{A.pID+1}(14+(1:12));
                    B.pSC.Ur  = Rede.pMSG.getFrom{A.pID+1}(26+(1:2));
                    B.pSC.U   = Rede.pMSG.getFrom{A.pID+1}(28+(1:2));
                    
                    % Postura do centro do robô
                    B.pPos.Xc([1 2 6]) = B.pPos.X([1 2 6]) + ...
                        [B.pPar.a*cos(B.pPos.X(6)); B.pPar.a*sin(B.pPos.X(6)); 0];
                    
                    % Armazenar dados no arquivo de texto
                    fprintf(Arq,'%6.6f\t',[A.pPos.Xd' A.pPos.X' A.pSC.Ur' A.pSC.U' ...
                        B.pPos.Xd' B.pPos.X' B.pSC.Ur' B.pSC.U' toc(t)]);
                    fprintf(Arq,'\n\r');
                end
            end
        end
        
        U = [U [A.pSC.U; B.pSC.U]];  % velocidade dos robôs
        
        disp('UrA');
        display(A.pSC.Ur);
        
        disp('UrB');
        display(B.pSC.Ur);
        
        
        %% Compensação dinâmica
        % Cálculo da aceleração
        %         A.pSC.Ua = A.pSC.U(1:2);                % salva velocidade anterior para cálculo de aceleração
        %         A.mLerDadosSensores;                      % lê dados dos sensores do robô
%         A.pSC.dU = (A.pSC.Ua - A.pSC.U)/.1;   % calcula aceleração (a=dV/dt)       %
                
        acc = [acc, A.pSC.dU];   % aceleração
       
        % Compensador dinâmico
        A = fCompensadorDinamico(A);
        
        
        %% Envia sinais de controle
      
        A.mEnviarSinaisControle;
      
        % Publicar mensagem na rede
        Rede.mSendMsg(A);
        %         ke = ke + 1;
        
    end
    % Desenha os robôs
    if toc(tp) > 0.1
        tp = tic;
        
        A.mCADdel
        B.mCADdel
        
        A.mCADplot2D('b')
        B.mCADplot2D('r')
        
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
A.pSC.Ur = [0 ; 0];
A.mEnviarSinaisControle;
% A.mDesconectar;
% pause

% Para retornar pioneer pelo controle (evitar a fadiga)
A.mJoystick;  % Verifica se joystick está conectado
while button(A.pSC.Joystick.J,8)==0  
 
    A.mEnviarSinaisControleTeste       
   
end
