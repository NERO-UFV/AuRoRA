%% Line reactive navigation v2 (03/02/2020)
%% Boas práticas

% //TODO: Orientação vem do mapa, velocidade maxima desejada é parametro
% livre. Logo, colocar na forma de seguimento de caminho.

close all
clearvars
clc

%% Carregando os objetos do cenário
V = VREP;
V.vConnect
V.vHandle('Pioneer_p3dx');
H = HandlePushObj;

%% Definindo o Robô
P = Pioneer3DX;
P.pPar.a = 0.3;
P.pPar.alpha = 0;

figure(2)
hold on
grid on
title('Simulação');
axis([-5 5 -5 5])
P.mCADplot2D('r')   % Visualização 2D
drawnow

disp('Stopping Pioneer')
P.pSC.Ud = [0; 0];
V.vSendControlSignals(P,1);

pause(1);
% [p,~] = V.vGetObjPosition('Disc');

%% Inicializando variáveis para o controle
%Declarando o trace:
dados=[];
V.vGetSensorData(P,1);

%Parâmetros Line
LinMap=zeros(8,2); Map=[]; hmed = []; Hist = [];
tp = tic;
%% Rotina da simulação:
t=tic;  ta=tic; tL = tic; tmax=120;
% tmax=80; %Percurso todo
k1=0.6; k2=0.4; it = 0;
J = JoyControl; J.pSNES = 1; Pilot = 0;
while toc(t)<tmax
    %% Get Laser Data
    if toc(tL)>0.5
        tL = tic;
        %Line:
        Map = V.vGetLaserData(P,1); 
    end
    
    %% Robot control
    if toc(ta)>0.1
        ta=tic;  it=it+1;
        
        
        if isempty(Map)
            % Decide later
        else
            %Subsampling:
            for kk = 20:20:size(Map(:,2),1)
                X = Map(kk,1);
                Y = Map(kk,2);

                %Angular coeff:
                b = X\Y;
                Ylinha = X*b;

                LinMap(kk/20,1) = X(1);
                LinMap(kk/20,2) = Ylinha(1);
            end
            
            XX = LinMap(:,1)+ P.pPos.X(1);
            YY = LinMap(:,2)+ P.pPos.X(2);
            b1 = XX\YY;
            Yreta = XX*b1;

            med = mean(LinMap);
 
            figure(2)
            hold on
            plus = plot(med(1),med(2),'r+','MarkerSize',7,'LineWidth',1.2);
            hmed = [hmed;med];
            
            %Posição do ponto médio:
            P.pPos.Xd(1)= med(1);
            P.pPos.Xd(2)= med(2);
            P.pPos.Xa(1:2) = med(1:2);
                
            figure(2)
            p1=plot(LinMap(:,1),LinMap(:,2),'b--');
            drawnow
        end

        if ~isempty(hmed) % Se o mapa estiver vazio
                P.pPos.Xd(1)= med(1);
                P.pPos.Xd(2)= med(2);
                P.pPos.Xa(1:2) = med(1:2);
        end

        %Pegar informação da posição e velocidade real do robô
        V.vGetSensorData(P,1);
        
        % Planejamento:
        if Pilot ~= 2
            P.pPos.X(6)= atan2(P.pPos.X(2)-P.pPos.Xc(2),P.pPos.X(1)-P.pPos.Xc(1));
            P.pPos.Xd(7:8)= (P.pPos.Xd(1:2)-P.pPos.Xa(1:2))/toc(ta);
        end
        
        % Controle:        
        [P,tmax,Pilot] = pSafetyControl(P,J,Pilot,tmax);
        P = fKinematicControllerExtended(P);
       
        % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];

        % Enviar sinais de controle para o robô
        V.vSendControlSignals(P,1);        
        Hist = [P.pPos.Xc,Hist];
        
        % Sub-rotina para plotar
        figure(2)
        if toc(tp) > 0.1
            if (Pilot == 1)
                delete(plus)
                plus = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);
            end
            
            tp = tic;
            % Plot da simulação
            P.mCADdel
            P.mCADplot2D('r')   % Visualização 2D
            drawnow
        end
        
    end
end

if Pilot == 0
    disp('Press START to quit.')
    J.pDigital(5) = 1;
    while ~J.pDigital(end)
        if toc(ta)>0.1
            ta=tic;
            V.vGetSensorData(P,1); 
            [P,tmax,Pilot] = pSafetyControl(P,J,Pilot,tmax);
            P = fKinematicControllerExtended(P);
            V.vSendControlSignals(P,1);  
        end
    end
end

P.pSC.Ud([1 2]) = 0;
V.vSendControlSignals(P,1);

%% Desconecta Matlab e V-REP
V.vDisconnect;

