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
PastaRaiz = 'AuRoRa 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))
cd(PastaAtual)

%% Clientes Roboserv

n = 2; %Numero de robôs

for ii=1:n
    
    P{ii} = Pioneer3DX(ii);
    P{ii}.pID = ii;
    %     P{ii}.mSetUDP('255.255.255.255');
    %     P{ii}.pStatus.Conectado = 1;
end

%Posição inicial dos robôs
P{1}.pPos.X = [0 0 0 0 0 0 0 0 0 0 0 0]';  % robo 1
P{2}.pPos.X = [1 -2 0 0 0 0 0 0 0 0 0 0]'; % robo 2

% posini = tic;
% while(toc(posini)<5)
%     for ii=1:n
%         P{ii}.mAtualizarServidor([0,0]);
%     end
% end

for ii = 1:n
    RastrosR(ii).X = [];
    RastrosR(ii).Xd = [];
    RastrosR(ii).U = [];
end

%% Formação

% Variáveis da formação
F.q = [0; 0; 0; 0];         %xf ; yf ; rof ; alfaf
F.q(1)    = (P{1}.pPos.X(1) + P{2}.pPos.X(1))/2;
F.q(2)    = (P{1}.pPos.X(2) + P{2}.pPos.X(2))/2;
F.q(3)   = sqrt((P{2}.pPos.X(1) - P{1}.pPos.X(1))^2 + (P{2}.pPos.X(2) - P{1}.pPos.X(2))^2);
F.q(4) = atan2((P{2}.pPos.X(2) - P{1}.pPos.X(2)),(P{2}.pPos.X(1) - P{1}.pPos.X(1)));

% Variáveis de formação desejadas
F.qd = [3; 5; 1; 0];        % [xf,yf,rof,alfaf]

% Erro de posição inicial
F.qTil = F.qd - F.q;

% Rastros da formação
RastrosF.q = [];
RastrosF.qTil = [];
RastrosF.vC = [];

%% Parâmetros dos controladores
% ganhos do controlador cinemático
l = [.7 .7 .7 1];     %
% l = ones(1,4);
k = [.2 .2 .6 .07];

% matrizes de ganho
L = [ l(1), 0,    0,    0;
    0,    l(2), 0,    0;
    0,    0,    l(3), 0;
    0,    0,    0,    l(4)];

Kappa = [k(1), 0,    0,    0;
    0,    k(2), 0,    0;
    0,    0,    k(3), 0;
    0,    0,    0,    k(4)];

% parâmetros dinâmicos do pioneer 3dx (tese Felipe Martins)
th = [0.2604 0.2509 0.000499 0.9965 0.00263 1.0768]; % theta

H = [th(1), 0 ; 0, th(2)];    % matriz do modelo dinâmico do robô
I = 1;                        % fator para ajuste (?)

% ganhos do controlador dinâmico
Lu = 1;
Lw = 1;
ku = 1;
kw = 1;

%% Simulação

parar = 0;         % flag de parada da simulação
figure
axis([-3,10,-2,15]);
grid on;
hold on

t_experimento = tic;
t_plot = tic;
t_controle = tic;

RastroControle = [];
RastroSimulacao = toc(t_experimento);

while parar == 0
    
    % Obtém dados dos sensores dos robôs
    P{1}.mLerDadosSensores;
    P{2}.mLerDadosSensores;
    
    if toc(t_controle)>0.1
        RastroControle = [RastroControle; toc(t_controle)];
        t_controle = tic;
        RastroSimulacao = [RastroSimulacao; toc(t_experimento)]; % salva tempo de simulação
        
        F.q(1)    = (P{1}.pPos.X(1) + P{2}.pPos.X(1))/2;
        F.q(2)    = (P{1}.pPos.X(2) + P{2}.pPos.X(2))/2;
        F.q(3)   = sqrt((P{2}.pPos.X(1) - P{1}.pPos.X(1))^2 + (P{2}.pPos.X(2) - P{1}.pPos.X(2))^2);
        F.q(4) = atan2((P{2}.pPos.X(2) - P{1}.pPos.X(2)),(P{2}.pPos.X(1) - P{1}.pPos.X(1)));
        
%         if( abs(F.q(4))>pi)
%             if(F.q(4)>=0)
%                 F.q(4) = -2*pi + F.q(4);
%             else
%                 F.q(4) = 2*pi + F.q(4);
%             end
%         end
%         
        % Matriz jacobiano inversa
        Jinv = [1, 0, -cos(F.q(4))/2, F.q(3)*sin(F.q(4))/2;
            0, 1, -sin(F.q(4))/2, -F.q(3)*cos(F.q(4))/2;
            1, 0, cos(F.q(4))/2, -F.q(3)*sin(F.q(4))/2;
            0, 1, sin(F.q(4))/2, F.q(3)*cos(F.q(4))/2];
        
        %  Matriz de cinemática inversa
        Kinv = [cos(P{1}.pPos.X(6)),     sin(P{1}.pPos.X(6)),      0,              0;
            -sin(P{1}.pPos.X(6))/P{1}.pPar.a, cos(P{1}.pPos.X(6))/P{1}.pPar.a,   0,              0;
            0,              0,      cos(P{2}.pPos.X(6)),     sin(P{2}.pPos.X(6));
            0,              0,      -sin(P{2}.pPos.X(6))/P{2}.pPar.a, cos(P{2}.pPos.X(6))/P{2}.pPar.a];
        
        % Erro de posição da formação
        F.qTil = F.qd - F.q;
        
        % Velocidades da formação
        F.dq_ref = L*tanh(L\Kappa*F.qTil);
        
        % Matriz de velocidades do robô
        F.dx_ref = Jinv*F.dq_ref;
        
        % Velocidades de controle cinemático
        F.vC = Kinv*F.dx_ref;
        
        P{1}.pSC.Uc = F.vC(1:2);
        P{2}.pSC.Uc = F.vC(3:4);
        
%         % Caso não use controle dinâmico -------------------------------
%         P{1}.pSC.Ur = F.vC(1:2);
%         P{2}.pSC.Ur = F.vC(3:4);
%         % -------------------------------------------------------------
        
        % CONTROLE DINAMICO ----------------------------------------------
        % Cálculo da aceleração
        for ii = 1:n
            
            P{ii}.pSC.uw_old = P{ii}.pSC.U(1:2);       % salva velocidade anterior para cálculo de aceleração
            P{ii}.mLerDadosSensores;         % lê dados dos sensores do robô
            P{ii}.pSC.uw_new = P{ii}.pSC.U(1:2);       % salva velocidade atual
            P{ii}.pSC.acc = (P{ii}.pSC.uw_new - P{ii}.pSC.uw_old)/.1;   % calcula aceleração (a=dV/dt)
        end
        %         P.mBroadcastPublicar;
        
        % Erros de velocidade
        for ii = 1:n
            P{ii}.pSC.U_til =[P{ii}.pSC.Uc(1) - P{ii}.pSC.U(1);
                P{ii}.pSC.Uc(2) - P{ii}.pSC.U(2)];
            
            % Matriz de Saturação Ts
            Ts = [Lu*tanh(P{ii}.pSC.U_til(1)*ku/Lu) ;
                Lw*tanh(P{ii}.pSC.U_til(2)*kw/Lw)];
            
            
            % Matrizes do modelo dinâmico do robô
            C = [0, -th(3); th(3), 0]*P{ii}.pSC.U(2);
            Fd = [th(4), 0; 0, th(6)+(th(5)-I*th(3))*P{ii}.pSC.U(1)];
            
            %Lei de controle dinâmico
            P{ii}.pSC.Ur = Ts + H*P{ii}.pSC.acc + C*[I*P{ii}.pSC.Uc(1); P{ii}.pSC.Uc(2)] + Fd*P{ii}.pSC.Uc(1:2);
            
        end
        % FIM DO CONTROLE DINAMICO ----------------------------------------
        
        % Envia sinal de controle aos robôs
        P{1}.mEnviarSinaisControle;
        P{2}.mEnviarSinaisControle;
        %         P{1}.mAtualizarServidor(F.vC(1:2));
        %         P{2}.mAtualizarServidor(F.vC(3:4));
        
        for ii=1:n
            RastrosR(ii).X = [RastrosR(ii).X; P{ii}.pPos.X(1:6)'];
            RastrosR(ii).Xd = [RastrosR(ii).Xd; P{ii}.pPos.Xd(1:6)'];
            RastrosR(ii).U = [RastrosR(ii).U; P{ii}.pSC.U(1:2)'];
        end
        
        % Salva dados
        RastrosF.q = [RastrosF.q;  F.q'];
        RastrosF.qTil = [RastrosF.qTil; F.qTil'];
        RastrosF.vC = [RastrosF.vC; F.vC'];
        
    end
    
    % Desenha trajetória em tempo real
    if toc(t_plot)>0.5
        t_plot = tic;
        try
            %             delete(fig(1));
            %             delete(fig(2));
            delete(fig(3));
            delete(fig(4));
            %
        end
        %         fig(1) = plot(P{1}.pPos.X(1),P{1}.pPos.X(2),'s');
        %         fig(2) = plot(P{2}.pPos.X(1),P{2}.pPos.X(2),'s');
        fig(3) = plot([P{1}.pPos.X(1),P{2}.pPos.X(1)],[P{1}.pPos.X(2),P{2}.pPos.X(2)]);
        fig(4) = plot(F.q(1),F.q(2),'x');
        fig(5) = plot(F.qd(1),F.qd(2),'x');
        %
        P{1}.mCADplot2D([1 0 0]);
        P{2}.mCADplot2D([1 1 0]);
        
        %         axis([-5, 5, -5, 5]);
        drawnow
    end
    
    % Critério de parada da simulação
    if (abs(F.qTil(1))<0.5 && abs(F.qTil(2))<0.05 && abs(F.qTil(3))<0.1 && abs(F.qTil(4))<pi/18)
        parar = 1;
    end
end
% Envia sinal de parada para cada robô
if parar == 1
    %     for nmsg=1:20
    %         %enviar mensagem parar
    %         for ii=1:n
    %             P{ii}.mParar;
    %         end
    %     end
    
    % Plota Resultados
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