clear all
close all
clc
%% Versão Teste Classe V-REP

V = VREP;
V.vConnect;

% Handle Push Obj Class
H = HandlePushObj;
%% Carregando os objetos do cenário
V.vHandle('Pioneer_p3dx');
% V.vHandle('Pioneer_p3dx','0');
% V.vHandle('Pioneer_p3dx','1');

V.vObject('Disc');
pause(1);
%% Get Destination point and Robot Position
[Goal,~] = V.vGetObjPosition('Disc');


%% Declarando a figura
figure(1)
hold on
axis([-6,6,-6,6])
Map = V.vGetLaserData(1);
h(1)= plot(Map(:,1),Map(:,2),'.b');
hold on
[Min,Vert,Max]= H.ObjectSearch(Map);

[Xv,Yv] = H.ProjectionFace(Map,Min,Vert,Max);
clear Min Vert Max;
[ObjList] = H.Orthogonal(Map,Xv,Yv,Goal);

%% Declarando o trace
Dados=zeros(1,7);
[Pos.Xc,Pos.X, Pos.U] = V.vGetSensorData(1);

Pos.Xd = zeros(8,1);
%% Inicializando variáveis para o controle
it=0;
tmax=50;
t=tic;ta=tic;
flag = 0;
%% Parametros das funções
a= 3; b = 2; T = 70; w = 2*pi/T;
%% Rotina da lemniscata
while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        it=it+1;
        %% Robot 1
        %Pegar informação da posição e velocidade real do robô
        [Pos.Xc, Pos.X, Pos.U] = V.vGetSensorData(1);
        
        MidFace = ObjList([3 4]);
%         L = norm(Map(Min,1:2)-MidFace);
        
     
        Pos.Xd(1)= MidFace(1);
        Pos.Xd(2)= MidFace(2);
        Pos.Xd(7)= 0;
        Pos.Xd(8)= 0;
        
        Pos.theta= atan2(Pos.X(2)-Pos.Xc(2),Pos.X(1)-Pos.Xc(1));

        K=[cos(Pos.theta) -0.15*sin(Pos.theta); sin(Pos.theta) 0.15*cos(Pos.theta)];
          
       % Control System
        Pos.Xtil = Pos.Xd([1 2])-Pos.X;
       
        if norm(Pos.Xtil)<0.1 && flag==0
            flag=1;
            tl=tic;
        end
        
        if flag==1
            Pos.Xd(1)= a*sin(w*toc(tl));
            Pos.Xd(2)= b*sin(2*w*toc(tl));
            
            Map = V.vGetLaserData(1);
%            
%            
            Pos.theta= atan2(Pos.X(2)-Pos.Xc(2),Pos.X(1)-Pos.Xc(1));

            K=[cos(Pos.theta) -0.15*sin(Pos.theta); sin(Pos.theta) 0.15*cos(Pos.theta)];

            % Control System
            Pos.Xtil = Pos.Xd([1 2])-Pos.X;
            Pos.Ud = K\(Pos.Xd([7 8])+1*tanh(.7*Pos.Xtil([1 2])));
%              % Trace
            Dados=[Dados ; [Pos.Xd([1 2])' Pos.X' Pos.Ud' toc(t)]];
           
        else
            Pos.Ud = K\(Pos.Xd([7 8])+1.7*tanh(0.5*Pos.Xtil([1 2])));
        end

        % Send Control Signal to VREP
        V.vSendControlSignals(Pos.Ud,1);
        
     
    end
end

%% Comando STOP Robots
Ud = [0; 0];
V.vSendControlSignals(Ud,1);


%% Desconecta Matlab e V-REP
V.vDisconnect;