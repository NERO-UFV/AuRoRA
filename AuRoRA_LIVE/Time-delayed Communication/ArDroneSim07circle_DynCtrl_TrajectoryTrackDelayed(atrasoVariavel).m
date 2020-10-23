%%%% Time-deleyad Multi-robot formation Control %%%%

% Testar erros involvidos na simulação do modelo

close all
clear
clc

try
    fclose(instrfindall);
end

% IAE_ITAE_IASC = []; % Matriz de índices de desempenho geral

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Ar = ArDrone;   % Real
idAtraso  = randi(34); % Atraso variável, de 0 a 1 segundo!

mkdir('C:\Users\leo_j\Dropbox\Dropbox\AuRoRA 2018\Time-delayed Communication\Dados Simu','Trajectory Tracking');
    
% Nome arquivo .mat
filename = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cDados Simu%cTrajectory Tracking%csimu_atraso_variavel(Circunferência).mat', '\', '\', '\', '\', '\', '\', '\', '\', '\');


A = ArDrone;    % Atrasado

% Conectar Joystick
J = JoyControl;

% Arquivo Texto
% ArqTxt = fopen('dataPosDelayed.txt','w');

% =========================================================================
% f1 = figure('Name','Simulação Posicionamento ArDrone','NumberTitle','off');
% % f1.Position = [435 2 930 682];
% f1.Position = [1367 11 831 634]; % Segunda tela em Sete Lagoas!
% figure(f1);
% 
% title('Task: Position Control')
% xlabel({'Eixo $$x$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
% ylabel({'Eixo $$y$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
% zlabel({'Eixo $$z$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
% axis equal
% axis([-4 2 -3 3 0 3])
% view(3)
% view(50,10)
% grid on
% hold on
% 
% % Time-delayed Drone Appearance
% A.mCADcolor([0;.4470; 0.5410]);
% A.pCAD.i3D.FaceAlpha = 1;
% A.mCADplot;
% 
% % Drone não atrasado:
% % Ar.mCADplot;
% 
% drawnow
pause(1)
disp('Start..........')

% =========================================================================
% Iniciar eta de controle - Decolar:
tmax = 40; % Tempo Simulação em segundos
t = tic;       % simulation current time
tc = tic;      % drone frequency
tp = tic;      % graph refresh rate
tt = tic;      % trajectory time

% =========================================================================
% Inicialização dos Índices de Desempenho (drone atrasado):
IAE = 0;
ITAE = 0;
IASC = 0;

% IAE_Ar = 0;
% ITAE_Ar = 0;
% IASC_Ar = 0;

% Variables Initialization
a = 1.5;        % "raio" do circulo em x
b = 1.5;          % "raio" do circulo em y
% w = 0.05;     % lemniscata
w = 0.2;     % circulo

% =========================================================================
% Dados robôs:
% XX = [Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' A.pPos.Xd' A.pPos.X' A.pSC.Ud' IAE_Ar ITAE_Ar IASC_Ar IAE ITAE IASC toc(t)];
XX = [A.pPos.Xd' A.pPos.X' A.pSC.Ud' idAtraso IAE ITAE IASC toc(t)];
kk = 1;

% xx = [A.pPos.X(1) Ar.pPos.X(1)];
% yy = [A.pPos.X(2) Ar.pPos.X(2)];
% zz = [A.pPos.X(3) Ar.pPos.X(3)]; 

xx = [A.pPos.X(1)];
yy = [A.pPos.X(2)];
zz = [A.pPos.X(3)];

% Parâmetros para integração numérica
dt = 30e-3;

ddX(1:3,1) = 0;
 dX(1:3,1) = A.pPos.X(7:9,1);
  X(1:3,1) = A.pPos.X(1:3,1);

% ddXAr(1:3,1) = 0;
%  dXAr(1:3,1) = 0;
%   XAr(1:3,1) = A.pPos.X(1:3,1);
% 
% Ar.pPos.Xd(1:3,1) = A.pPos.Xd(1:3,1);
% Ar.pPos.Xd(7:9,1) = zeros(3,1);

% =========================================================================
%- Obter informação com atraso
% A  informação da posição do robô está com atraso
% Informação a cada 30ms
% Atraso máximo de 1s
% idAtraso  = 10; %randi(34); Atraso variável, de 0 a 1 segundo!

% acc_max = 0.85; % m/s^2 (Tarefa de segmento de trajetória)
acc_max = 0; % Tarefa de posicionamento
       

while toc(t) < tmax
    if toc(tc) > 1/30
        % Inicio da realimentação:
        tc = tic;   % timer para controle

        % Trajetória desejada:
        % tt = toc(t);
        tt = toc(t);
        if toc(t)>50
            % Return to initial position
            A.pPos.Xd(1)  = 0;    % posição x
            A.pPos.Xd(2)  = 0;    % posição y
            A.pPos.Xd(7)  = 0;    % velocidade em x
            A.pPos.Xd(8)  = 0;    % velocidade em y
            A.pPos.Xd(3)  = 1.5;
            A.pPos.Xd(9)  = 0;
            
        else
            % circunferência:
            A.pPos.Xd(1)  = a*cos(w*tt);    % posição x
            A.pPos.Xd(2)  = b*sin(w*tt);    % posição y
            A.pPos.Xd(3)  = 1.5;                 % posição z
            A.pPos.Xd(7)  = -a*w*sin(w*tt); % velocidade em x
            A.pPos.Xd(8)  = b*w*cos(w*tt);  % velocidade em y
            A.pPos.Xd(9)  = 0;
        end               
     
        
        % Controlador:
        acc_des = acc_max;
        K1 = diag([0.5 0.6 2]);
        K2 = diag([3 3 3]);
        K3 = sqrt(4*K1);
        K4 = sqrt(4*K1*K2)/K3;
        
        % -------------------------- Drone não Atrasado
%         Ar.pPos.Xtil = Ar.pPos.Xd - Ar.pPos.X;
%         pos_tilAr = Ar.pPos.Xtil(1:3,1);                
%         vel_tilAr = Ar.pPos.Xtil(7:9,1);
% 
%         ddXAr = acc_des + K1*tanh(K2*vel_tilAr) + K3*tanh(K4*pos_tilAr);                                                     
% 
%         % Pegando os dados do robo:
%         dXAr = dXAr + ddXAr*dt; % Velocidade atual
%         XAr = XAr + dXAr*dt;    % Posição atual
% 
%         Ar.pPos.X(1:3,1) = XAr;
%         Ar.pPos.X(7:9,1) = dXAr;
        
        % -------------------------- Drone Atrasado
        
        %- Obter informação com atraso
        % A  informação da posição do robô está com atraso
        % Informação a cada 30ms
        % Atraso máximo de 1s
        idAtraso  = randi(34); % Atraso variável, de 0 a 1 segundo!
        
        if kk > idAtraso
            A.pPos.X = XX(kk-idAtraso,13:24)';
        end

        A.pPos.Xtil = A.pPos.Xd - A.pPos.X;
        pos_til = A.pPos.Xtil(1:3,1);                
        vel_til = A.pPos.Xtil(7:9,1);

        ddX = acc_des + K1*tanh(K2*vel_til) + K3*tanh(K4*pos_til);                                                     

        % Pegando os dados do robo:
        dX = dX + ddX*dt; % Velocidade atual
        X = X + dX*dt;    % Posição atual

        A.pPos.X(1:3,1) = X;
        A.pPos.X(7:9,1) = dX;
        % -------------------------- 
        % Índices de desempenho (Atrasado & Não-atrasado)
        IAE = IAE + (abs(A.pPos.Xtil(1)) + abs(A.pPos.Xtil(2)) + abs(A.pPos.Xtil(3)))*dt;
        ITAE = ITAE + (abs(A.pPos.Xtil(1)) + abs(A.pPos.Xtil(2)) + abs(A.pPos.Xtil(3)))*toc(t)*dt;
        IASC = IASC + abs(abs(ddX(1)) + abs(ddX(2)) + abs(ddX(3)))*dt;
        
%         IAE_Ar = IAE_Ar + (abs(Ar.pPos.Xtil(1)) + abs(Ar.pPos.Xtil(2)) + abs(Ar.pPos.Xtil(3)))*dt;
%         ITAE_Ar = ITAE_Ar + (abs(Ar.pPos.Xtil(1)) + abs(Ar.pPos.Xtil(2)) + abs(Ar.pPos.Xtil(3)))*toc(t)*dt;
%         IASC_Ar = IASC_Ar + (abs(ddXAr(1)) + abs(ddXAr(2)) + abs(ddXAr(3)))*dt;
        
        
        % Histórico de dados:
        XX = [XX; A.pPos.Xd' A.pPos.X' A.pSC.Ud' idAtraso IAE ITAE IASC toc(t)];
        kk = kk + 1;

%         % Rota feita pelos drones {
%         xx = [xx; A.pPos.X(1) Ar.pPos.X(1)];
%         yy = [yy; A.pPos.X(2) Ar.pPos.X(2)];
%         zz = [zz; A.pPos.X(3) Ar.pPos.X(3)];
%         %                       }
        
        % Rota feita pelos drones {
        xx = [xx; A.pPos.X(1)];
        yy = [yy; A.pPos.X(2)];
        zz = [zz; A.pPos.X(3)];
        %                       }

%         fprintf(ArqTxt,'%6.6f\t',[A.pPos.Xd' A.pPos.X' A.pSC.Ud' toc(t)]);
%         fprintf(ArqTxt,'\n');
    end
   
    if toc(tp) > inf
        tp = tic;
        A.mCADplot;
        Ar.mCADplot;
        plot3(xx(:,1),yy(:,1),zz(:,1),'-b',xx(:,2),yy(:,2),zz(:,2),'--r')
        drawnow
        hold on            
    end
    
end

pause(2)
disp('end..........')

% Salva dados da simulação
rSaveData(filename, XX, IAE, ITAE, IASC, idAtraso);



% fclose(ArqTxt);


%% ----- Posição para Atraso Específico: 
rPlotPositionVar(tmax);

% Variação do atraso
rPlotDelayedVariation(tmax);
%-----

%%----- IAE, ITAE e IASC para Atraso Específico: (adaptar!!!)
rPlotPerformanceAnalysisVar(tmax);
%-----
