%% CONTROLE DINÂMICO DE FORMAÇÃO DE 2 ROBÔS POR ESTRUTURA VIRTUAL
% Programa Original: Marcos Felipe e Wagner Casagrande
% Adaptação para Matlab : Chacal   e Marcos Felipe
% NERO - 2018

clear all
close all
clc
try
    fclose(instrfindall)
end

%% Configuração da pasta
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))
cd(PastaAtual)

%% Inicialização das classes
Rede = InfoSharing;    % classe para troca de dados entre os robôs
% n = 2; %Numero de robôs
ID = input('Digite o ID do robo: ');

P{ID} = Pioneer3DX(ID);
P{ID}.pID = ID;

F = Formation_2Robots(ID);

%% Posição inicial dos robôs
% x = input('Digite a posição x do robo: ');
% y = input('Digite a posição y do robo: ');
% P{ID}.pPos.X = [x y 0 0 0 0 0 0 0 0 0 0]';  % robo
P{ID}.pPos.X = [0 0 0 0 0 0 0 0 0 0 0 0]';  % robo

%% Inicialização da comunicação

F.mIniciaComunicacao(P{ID},Rede);
n = length(Rede.pMSG.getFrom);  % verifica número de robôs

%% Inicializa variáveis para armazenar dados

% for ii = 1:n
%     RastrosR(ii).X = [];
%     RastrosR(ii).Xd = [];
%     RastrosR(ii).U = [];
% end

RastrosR.X = [];
RastrosR.Xd = [];
RastrosR.U = [];

% Rastros da formação
RastrosF.q = [];
RastrosF.qTil = [];
RastrosF.vC = [];

%% Critério para inicializar o programa
if length(Rede.pMSG.getFrom)>1
    
    try
        % Povoa variáveis do robo
        for ii = 1:n
            F.pPos.tempX{ii} = Rede.pMSG.getFrom{ii}(15:26);  % captura postura X de cada robô
        end
        %    Postura dos robôs da formação: [x1 y1 z1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
        F.pPos.X = [F.pPos.tempX{1}(1:6);F.pPos.tempX{2}(1:6)];
    catch
        disp('Dados da rede incompletos.');
    end
    %% Formação
    % Variáveis de formação desejadas
    F.pPos.Qd = [3; 5; 1; 0];        % [xf,yf,rof,alfaf]
    
    % Cálculo das Variáveis da formação
    F.mCalculaFormacao;
    
    
    %% Parâmetros dos controladores
    % Cinemático
    % ganhos do controlador cinemático
    F.pPar.l = [.7 .7 .7 .07];     %
    % l = ones(1,4);
    F.pPar.k = [.05 .05 .15 .0175];
    
    % Dinâmico
    % parâmetros dinâmicos do pioneer 3dx (tese Felipe Martins)
    
    P{ID}.pPar.th = [0.2604 0.2509 0.000499 0.9965 0.00263 1.0768]; % theta
    
    P{ID}.pPar.H = [P{ID}.pPar.th(1), 0 ; 0, P{ID}.pPar.th(2)];    % matriz do modelo dinâmico do robô
    P{ID}.pPar.I = 1;                        % fator para ajuste (?)
    
    
    % ganhos do controlador dinâmico
    P{ID}.pPar.Lu = 1;
    P{ID}.pPar.Lw = 1;
    P{ID}.pPar.ku = 1;
    P{ID}.pPar.kw = 1;
    
    %% Simulação
    
    parar = 0;         % flag de parada da simulação
    figure
    axis([-3,10,-2,15])
    hold on
    
    t_experimento = tic;
    t_plot = tic;
    t_controle = tic;
    
    RastroControle = [];
    RastroSimulacao = toc(t_experimento);
    
    %% Início do loop
    while parar == 0
        
        P{ID}.mLerDadosSensores;    % Obtém dados dos sensores do robô
        
        if toc(t_controle)>0.1
            %             disp('entrou no if');
            RastroControle = [RastroControle; toc(t_controle)];
            t_controle = tic;
            RastroSimulacao = [RastroSimulacao; toc(t_experimento)]; % salva tempo de simulação
            
            %% Calculo das variáveis da formação
            try
                % Povoa variáveis do robo
                for ii = 1:n
                    F.pPos.tempX{ii} = Rede.pMSG.getFrom{ii}(15:26);  % captura postura X de cada robô
                end
                %    Postura dos robôs da formação: [x1 y1 z1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
                F.pPos.X = [F.pPos.tempX{1}(1:6);F.pPos.tempX{2}(1:6)];
                disp('Dados recebidos.');
            catch
                disp('Dados da rede incompletos.');
            end
            
            
            F.mCalculaFormacao;
            
            %             % Tratamento de ângulo (?)
            %             if( abs(F.q(4))>pi)
            %                 if(F.q(4)>=0)
            %                     F.q(4) = -2*pi + F.q(4);
            %                 else
            %                     F.q(4) = 2*pi + F.q(4);
            %                 end
            %             end
            
            %% Calculo da velocidade cinemática
            F.mCalculaVcin;
            
            % Atribui velocidade cinemática ao robo
            P{ID}.pSC.Uc = F.pSC.tempVc{ID};
            
            P{ID}.pSC.Ur = P{ID}.pSC.Uc; % Caso não inclua controle
            %             dinâmico
            %
            
            %% Troca de informação com a rede
            %             Rede.mSendMsg(P{ID});       % envia dados para rede
            %             Rede.mReceiveMsg;           % le dados da rede
            %
            F.mIniciaComunicacao(P{ID},Rede);
            
            %% CONTROLE DINÂMICO  ------------------------------
            
            % Cálculo da aceleração
            obj.pSC.Ua = obj.pSC.U(1:2);                % salva velocidade anterior para cálculo de aceleração
            obj.mLerDadosSensores;                      % lê dados dos sensores do robô
            obj.pSC.dU = (obj.pSC.Ua - obj.pSC.U)/.1;   % calcula aceleração (a=dV/dt)
            
%             % Compensação dinâmica
%             A.mCompensadorDinamico;
            
        end
        %% Envia sinal de controle ao robô
        P{ID}.mEnviarSinaisControle;
        
        %% Salva dados
        RastrosF.q = [RastrosF.q;  F.pPos.Q'];
        RastrosF.qTil = [RastrosF.qTil; F.pPos.Qtil'];
        RastrosF.vC = [RastrosF.vC; F.pSC.Vc'];
        %                 for ii=1:n
        %
        %         RastrosR(ii).X = [RastrosR(ii).X; P{ii}.pPos.X(1:6)'];
        %         RastrosR(ii).Xd = [RastrosR(ii).Xd; P{ii}.pPos.Xd(1:6)'];
        %         RastrosR(ii).U = [RastrosR(ii).U; P{ii}.pSC.U(1:2)'];
        %                 end
        %
        
        RastrosR.X = [RastrosR.X; P{ID}.pPos.X(1:6)'];
        RastrosR.Xd = [RastrosR.Xd; P{ID}.pPos.Xd(1:6)'];
        RastrosR.U = [RastrosR.U; P{ID}.pSC.U(1:2)'];
        
        %         %% Troca de informação com a rede
        %         Rede.mSendMsg(P{ID});       % envia dados para rede
        %         Rede.mReceiveMsg;           % le dados da rede
        F.mIniciaComunicacao(P{ID},Rede);
        
        %% Desenha trajetória em tempo real
        if toc(t_plot)>0.5
            t_plot = tic;
            try
                %             delete(fig(1));
                %             delete(fig(2));
                delete(fig(3));
                delete(fig(4));
                %
            catch
            end
            % Obtém dados do outro robô da formação para plotar gráficos
            %             for ii = 1:n
            %                 if ii~=ID
            %                     P{ID}.pPos.X = Rede.pMSG.getFrom{ID}(15:26);  % captura postura X de cada robô
            %                 end
            %
            %             end
            
            %         fig(1) = plot(P{1}.pPos.X(1),P{1}.pPos.X(2),'s');
            %         fig(2) = plot(P{2}.pPos.X(1),P{2}.pPos.X(2),'s');
            %             fig(3) = plot([P{1}.pPos.X(1),P{2}.pPos.X(1)],[P{1}.pPos.X(2),P{2}.pPos.X(2)]);
            fig(4) = plot(F.pPos.Q(1),F.pPos.Q(2),'x');
            fig(5) = plot(F.pPos.Qd(1),F.pPos.Qd(2),'x');
            %
            
            P{ID}.mCADplot2D([1 0 0]);  % plota modelo do robô
            %             P{2}.mCADplot2D([0 1 0]);  % plota modelo do robô
            drawnow
            
        end
        
        % Critério de parada da simulação
        if (abs(F.pPos.Qtil(1))<0.5 && abs(F.pPos.Qtil(2))<0.05 && ...
                abs(F.pPos.Qtil(3))<0.1 && abs(F.pPos.Qtil(4))<pi/18)
            parar = 1;
        end
    end
    %% Fim do loop
    
    
    %% Envia sinal de parada para cada robô
    % if parar == 1
    %     for nmsg=1:20
    %         %enviar mensagem parar
    %         for ii=1:n
    %             P{ii}.mParar;
    %         end
    %     end
    
    
    %% Plota Resultados
    figure
    hold on
    plot(RastrosF.vC(:,1)),title('Velocidade Linear');
    plot(RastrosF.vC(:,3));
    figure
    hold on
    plot(RastrosF.vC(:,2));
    plot(RastrosF.vC(:,4)),title('Velocidade Angular');
    figure;
    
    plot(RastrosF.qTil(:,[1 2 3])),title('Erro Posição'),legend('X','Y','\rho');
    figure;
    plot(RastrosF.qTil(:,4)),title('Erro Angulo da formação');
end