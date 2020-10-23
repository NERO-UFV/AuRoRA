%% Instruções Iniciais
clear; close all; clc;
try close(instrfindall), catch, end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Gravar arquivos de txt (ou excel)
NomeArq = datestr(now,30);
cd('DataFiles\Log_Controladores')
Arq = fopen([NomeArq '.txt'],'w');
cd(PastaAtual)

% Criar objetos
P = Pioneer3DX(1);
P.pPos.Xd(1:2) = [0;0];

figure(1);
hold on; grid on; 
axis([-4 4 -2.5 2.5]);
rastroMax = 1000;
rastroAtu = diag(P.pPos.X(1:2))*ones(2,rastroMax);
rastroDes = diag(P.pPos.Xd(1:2))*ones(2,rastroMax);
rastroPio = diag(P.pPos.Xd(1:2))*ones(2,rastroMax);

% Conectar ao robôs
% P.mConectar;

% Conectar ao robo do simulador
P.mConectarMobileSim;

pause(5);



% Setar Postura inicial
P.pPos.Xso([1 2 6]) = [0 0 0];
P.mDefinirPosturaInicial(P.pPos.Xso([1 2 6]));

% Ler postura inicial (lï¿½ obj.pPos.Xs) e o erro
P.mLerDadosSensores;
P.pPos.Xtil = P.pPos.Xd - P.pPos.X;

%% Dados da Trajetória
a = 3;
b = 1;
wrobo = 0.75; % Dados do robô

wmax = wrobo/sqrt(a^2+4*b^2);

perc = 1;
w = perc*wmax;

nvoltas = 2;
tsim = 2*pi*nvoltas/w;

IAE = 0;
ITAE = 0;
IAE2 = 0;
ITAE2 = 0;

% Tempos
t = tic;
tp = tic;
tc = tic;
texib = 0.1;

while toc(t) < tsim
    
    % Coletar dados de 10 em 10ms
    if toc(tc) > 0.1
        tc = tic;
        
        P.pPos.Xd(1) = a*sin(w*toc(t));
        P.pPos.Xd(2) = b*sin(2*w*toc(t));
        P.pPos.Xd(7) = a*w*cos(w*toc(t));
        P.pPos.Xd(8) = 2*b*w*cos(2*w*toc(t));
        
        % Ler dados sensores
        P.mLerDadosSensores
             
        % Calcular sinal de controle
        P = fControladorDinamico(P);
        
        % Enviar sinais de controle
        P.mEnviarSinaisControle
        
        % Armazenar dados no arquivo de texto
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ur' P.pSC.U' toc(t)]);
        fprintf(Arq,'\n\r');
        
        % Atualização do rastro
        rastroAtu = [rastroAtu(:,2:end) P.pPos.X(1:2)];
        rastroDes = [rastroDes(:,2:end) P.pPos.Xd(1:2)];
        rastroPio = [rastroPio(:,2:end) P.pPos.Xp(1:2)];
        
        IAE = IAE + norm(P.pPos.Xtil(1:2));
        ITAE = ITAE + norm(P.pPos.Xtil(1:2))*toc(t);
        
        if toc(t) > tsim/2
            IAE2 = IAE2 + norm(P.pPos.Xtil(1:2));
            ITAE2 = ITAE2 + norm(P.pPos.Xtil(1:2))*(toc(t)-tsim/2);
        end
    end
    
    if toc(tp) > texib
        tp = tic;
        
        try
            delete(rastrofig)
        end
        
        P.mCADdel
        P.mCADplot2D('b')
        
        rastrofig(1) = plot(rastroDes(1,:),rastroDes(2,:),'r--');
        rastrofig(2) = plot(rastroAtu(1,:),rastroAtu(2,:),'k-');                
        rastrofig(3) = plot(rastroPio(1,:),rastroPio(2,:),'g-');        
        rastrofig(4) = plot(rastroDes(1,end),rastroDes(2,end),'r*');
        
        drawnow % Possibilita a exibição na tela
    end
    
end

%% Fechar e Parar
fclose(Arq);

display([IAE ITAE IAE2 ITAE2])

% Envia velocidades 0 para o robô
P.pSC.Ur = [0 ; 0];
P.mEnviarSinaisControle;

pause(2);

% P.mDesconectar;