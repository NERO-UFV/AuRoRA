%% Line reactive navigation
%simExtRemoteApiStart(19999)
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
% V.vObject('Disc');    

disp('Stopping Pioneer')
Ud = [0; 0];
V.vSendControlSignals(Ud,1);

pause(1);
% [p,~] = V.vGetObjPosition('Disc');

%% Inicializando variáveis para o controle
%Declarando o trace:
Dados=[];
[Pos.Xc1,Pos.X1, Pos.U1] = V.vGetSensorData(1);
Pos.Xd1 = zeros(8,1);
Pos.Xd1(2)=Pos.Xc1(2);
Pos.Xd1(1) = Pos.Xc1(1) + 1;

%Parâmetros Line
alphas = 0; % angular coeffs
betas = []; % independent coeffs
k = []; % index
LinMap=zeros(8,2); % 
LiveMap =[];
minH = [];
Hist = [];
% path = [];
dist =0;

%% Rotina da simulação:
t=tic;  ta=tic; tmax=45;
% tmax=80; %Percurso todo
k1=0.6; k2=0.4; it = 0;

while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        it=it+1;
        %% Robot control
        %Disc following:
%         [p,~] = V.vGetObjPosition('Disc');
%         oldX = Pos.Xd1(1);
%         oldY = Pos.Xd1(2);
        
        %Line:
        Map = V.vGetLaserData(1); 
%         minH = [min(Map(:,3)),minH];
        if isempty(Map) || dist > 2
            %Disc(GOAL) postition:
%             Pos.Xd1(1)= p(1);
%             Pos.Xd1(2)= p(2);
        else
         if 1==1%(mod(it,6)==0)            
            for kk = 20:20:size(Map(:,2),1)
                %Subsampling X:
                X = Map(kk,1);
                 
                %Subsampling Y: 
                Y = Map(kk,2);

                %Angular coeff for each iteration:
                b = X\Y;
                Ylinha = X*b;

                LinMap(kk/20,1) = X(1);
                LinMap(kk/20,2) = Ylinha(1);
            end
            
            XX = LinMap(:,1);
            YY = LinMap(:,2);
            b1 = XX\YY;
            Yreta = XX*b1;

            med = mean(LinMap);
            hold on
            plot(med(1),med(2),'r*')
            
            %Posição do ponto médio:
%             dist = sqrt( (Pos.Xd1(1)^2) + (Pos.Xd1(2)^2) )

%             if dist > 1 && it>10
%                 %manter destino
%                 disp(path)
%             else
                Pos.Xd1(1)= med(1);
                Pos.Xd1(2)= med(2);
%             end
            
            p1=plot(LinMap(:,1),LinMap(:,2),'b--');
            p2=plot(Map(:,1),Map(:,2),'g');
%              hold on
%             p2 =plot(XX,Yreta,'--');
            drawnow
         end
        end
%         delete(p1)
%         delete(p2)

        

        Pos.theta1= atan2(Pos.X1(2)-Pos.Xc1(2),Pos.X1(1)-Pos.Xc1(1));
        Pos.Xd1(7)= 0;
        Pos.Xd1(8)= 0;
        
        K=[cos(Pos.theta1) -0.15*sin(Pos.theta1);...
            sin(Pos.theta1) 0.15*cos(Pos.theta1)];
       
        %Pegar informação da posição e velocidade real do robô
        [Pos.Xc1, Pos.X1, Pos.U1] = V.vGetSensorData(1);
               
        % Controle
        Pos.Xtil = Pos.Xd1([1 2])-Pos.X1;
        dist = sqrt( (Pos.Xtil(1)^2) + (Pos.Xtil(2)^2) );
        
%         Não estão em uso:
%         distanciaAlvoRobo = sqrt(Pos.Xtil([1])^2+Pos.Xtil([2])^2);
%         anguloAlvoRobo = atan(Pos.Xtil([2])/Pos.Xtil([1]))*180/pi;
        
        Pos.Ud1=K\(Pos.Xd1([7 8])+k1*tanh(k2*Pos.Xtil([1 2])));

        V.vSendControlSignals(Pos.Ud1,1);
%         temp = V.vGetLaserData(1);
        LiveMap=[LinMap,LiveMap];
        Hist = [Pos.Xc1,Hist];
    end
end

% plot(tpt,Hist(2,:))
% tpt = 1:size(Hist,2);

%% Comando STOP Robots
Ud = [0; 0];
V.vSendControlSignals(Ud,1);

%% Desconecta Matlab e V-REP
V.vDisconnect;

%% Select time-window and plot lin-reg:
% Points=[];
% for ii = 3:103:300
%     figure(ii)
%     [r,s,disc,Map]=laserSweeper(LiveMap(:,(ii-2):i));
% 
%     alphas = []; % angular coeffs
%     betas = []; % independent coeffs
%     k = []; % index
%     % regLine=zeros(size(Map(:,2),1)/3,2);
% 
%     for kk = 9:9:size(Map(:,2),1)
%         X = [Map(kk,1),Map(kk-1,1),Map(kk-2,1),...
%              Map(kk-3,1),Map(kk-4,1),Map(kk-5,1),...
%              Map(kk-6,1),Map(kk-7,1),Map(kk-8,1)];
%         Y = [Map(kk,2),Map(kk-1,2),Map(kk-2,2),...
%              Map(kk-3,2),Map(kk-4,2),Map(kk-5,2),...
%              Map(kk-6,2),Map(kk-7,2),Map(kk-8,2)];
%         poiX = [Map(kk,1),Map(kk-1,1),Map(kk-2,1)]; % 3 last X
%         poiY = [Map(kk,2),Map(kk-1,2),Map(kk-2,2)]; % 3 last Y
%         regLine = polyfit(X,Y,1); % linear regression
%         alphas(kk) = regLine(1);
%         betas(kk) = regLine(2);
%         if abs(alphas(kk))>100
%             alphas(kk)=0;
%     %         betas(kk)=0;
%         elseif abs(alphas(kk))<10^-3
%             alphas(kk)=0;
%         end
% 
%         k = [k,kk];
% 
%         figure(ii)
%         hold on
%         Ylinha=polyval(regLine,X);
%         p1 = plot(X,Ylinha,'b');
%         Points = [X(1),Ylinha(1);Points];
%         pause(0.1)
%     end
% end

% plot(Points(:,1),Points(:,2)) % Mapa visto em t=to, contínuo

%% Subsampling hist
    %n=12:
%             for kk = 15:15:size(Map(:,2),1)
%                 X = [Map(kk,1),Map(kk-1,1),Map(kk-2,1),...
%                      Map(kk-3,1),Map(kk-4,1),Map(kk-5,1),...
%                      Map(kk-6,1),Map(kk-7,1),Map(kk-8,1)];
%                 Y = [Map(kk,2),Map(kk-1,2),Map(kk-2,2),...
%                      Map(kk-3,2),Map(kk-4,2),Map(kk-5,2),...
%                      Map(kk-6,2),Map(kk-7,2),Map(kk-8,2)];

    %n=9
%             for kk = 30:30:size(Map(:,2),1)
%                 X = [Map(kk,1),Map(kk-1,1),Map(kk-2,1),...
%                      Map(kk-3,1),Map(kk-4,1),Map(kk-5,1)];
%                 Y = [Map(kk,2),Map(kk-1,2),Map(kk-2,2),...
%                      Map(kk-3,2),Map(kk-4,2),Map(kk-5,2),...
%                      Map(kk-6,2),Map(kk-7,2),Map(kk-8,2)];




