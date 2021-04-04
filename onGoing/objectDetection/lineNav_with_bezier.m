%% Line reactive navigation
%% Boas práticas
close all
clearvars
clc

%% Look for root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
try
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
catch
    disp('Wrong root directory')
end

%% Abrir V-REP
% cd([pwd,'\V-Rep_estavel\Scene'])
% ! scene_laser_corridor.ttt
% 
% disp('Inicie a simulação!')
% pause(10)

%% Carregando os objetos do cenário
V = VREP;
V.vConnect
V.vHandle('Pioneer_p3dx');
H = HandlePushObj;
% V.vObject('Disc');

%% Definindo o Robô
P = Pioneer3DX;
P.pPar.a = 0;
P.pPar.alpha = (0)*(pi/180);
% P.rConnect; % Descomentar para realiazação de experimento
% P.rSetPose([0 0 0 0]);

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
k = []; % index
LinMap=zeros(8,2); % 
LiveMap =[];
Map=[];
hmed = [];
Hist = [];
Curve =[];

%% Rotina da simulação:
t=tic;  ta=tic; tL = tic; tmax=120;
% tmax=80; %Percurso todo
k1=0.6; k2=0.4; it = 0;
J = JoyControl; J.pSNES = 1; Pilot = 0;
while toc(t)<tmax
    if toc(tL)>0.5
        tL = tic;
        %Line:
%         ts = tic;
        Map = V.vGetLaserData(P,1); 
%         toc(ts)
    end
    if toc(ta)>0.1
        ta=tic;  it=it+1;
        %% Robot control
        %Disc following:
%         [p,~] = V.vGetObjPosition('Disc');
%         oldX = P.pPos.Xd(1);
%         oldY = P.pPos.Xd(2);
        
        
        
        if isempty(Map)
            %Disc(GOAL) postition:
%             Pos.Xd1(1)= p(1);
%             Pos.Xd1(2)= p(2);
        else
         if (mod(it,3)==0)
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
            
            hold on
            plot(med(1),med(2),'r*')
            hmed = [hmed;med];
            
%             Curve = H.BezierCurve(hmed',toc(t)/tmax);
            
            %Posição do ponto médio:
%             if toc(t)>10
                P.pPos.Xd(1)= med(1);
                P.pPos.Xd(2)= med(2);
%             else
%                 P.pPos.Xd(1)= Curve(1);
%                 P.pPos.Xd(2)= Curve(2);
%             end
            
            p1=plot(LinMap(:,1),LinMap(:,2),'b--');
%             p2=plot(Hist(1,:),Hist(2,:),'black');
%             hold on
%             p2 =plot(XX,Yreta,'--');
            drawnow
         end
        end
%         delete(p1)
%         delete(p2)

        if ~isempty(hmed)
%             Curve = H.BezierCurve(hmed',toc(t)/tmax);

            %Posição do ponto médio:
%             if toc(t)>10
                P.pPos.Xd(1)= med(1);
                P.pPos.Xd(2)= med(2);
%             else
%                 P.pPos.Xd(1)= Curve(1);
%                 P.pPos.Xd(2)= Curve(2);
%             end
        end

        %Pegar informação da posição e velocidade real do robô
        V.vGetSensorData(P,1);
        
        P.pPos.X(6)= atan2(P.pPos.X(2)-P.pPos.Xc(2),P.pPos.X(1)-P.pPos.Xc(1));
        P.pPos.Xd(7)= 0;
        P.pPos.Xd(8)= 0;
        
        K=[cos(P.pPos.X(6)) -0.15*sin(P.pPos.X(6));...
            sin(P.pPos.X(6)) 0.15*cos(P.pPos.X(6))];
               
        % Controle
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
%         Não estão em uso:
%         distanciaAlvoRobo = sqrt(Pos.Xtil([1])^2+Pos.Xtil([2])^2);
%         anguloAlvoRobo = atan(Pos.Xtil([2])/Pos.Xtil([1]))*180/pi;
        if Pilot == 0
            P.pSC.Ud=K\(P.pPos.Xd([7 8])+k1*tanh(k2*P.pPos.Xtil([1 2])));
        else
            [P,tmax,Pilot] = pSafetyControl(P,J,Pilot,tmax);
        end
% 
%         P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
%         
%         % Controlador Cinemático modelo extendido
%         vx = P.pPos.Xd(7) + 0.2*P.pPos.Xtil(1);
%         vy = P.pPos.Xd(8) + 0.2*P.pPos.Xtil(2);
%                 
%         if abs(P.pPar.alpha) < pi/2 && P.pPar.a > 0
%             vw = (-sin(P.pPos.X(6))*vx + cos(P.pPos.X(6))*vy)/(P.pPar.a*cos(P.pPar.alpha));
%             P.pSC.Ud(2) = vw;
%         else              
%             P.pPos.Xd(6)  = atan2(vy,vx);
%             P.pPos.Xd(12) = (P.pPos.Xd(6)-P.pPos.Xda(6))/0.1;            
%             P.pPos.Xtil(6) = P.pPos.Xd(6) - P.pPos.X(6);
%             if abs(P.pPos.Xtil(6)) > pi
%                 if P.pPos.Xtil(6) > 0
%                     P.pPos.Xtil(6) = - 2*pi + P.pPos.Xd(6) - P.pPos.X(6);
%                 else
%                     P.pPos.Xtil(6) =   2*pi + P.pPos.Xd(6) - P.pPos.X(6);
%                 end
%             end
%             vw = 0*(P.pPos.Xd(12)) + 1*P.pPos.Xtil(6);
%         end
%         
%         P.pSC.Ud(2) = vw;
%         P.pSC.Ud(1) = vx*cos(P.pPos.X(6)) + vy*sin(P.pPos.X(6)) ...
%                            + P.pPar.a*sin(P.pPar.alpha)*vw;

        % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];

        % Enviar sinais de controle para o robô
        V.vSendControlSignals(P,1);        
        Hist = [P.pPos.Xc,Hist];
    end
end

P.pSC.Ud([1 2]) = 0;
V.vSendControlSignals(P,1);

%% Desconecta Matlab e V-REP
V.vDisconnect;





