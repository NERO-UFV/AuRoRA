xclear all
close all
clc
%% Test Beta Version Class V-REP and HandlePushObj
% Handle Push Obj Class
H = HandlePushObj;

% V-REP Class
V = VREP;
V.vConnect;

%% Loading Pioneer Tags and Desired Position
V.vHandle('Pioneer_p3dx');  
V.vObject('Disc');
pause(2)
[Goal,~] = V.vGetObjPosition('Disc');

[Pos.Xc,Pos.X, Pos.U] = V.vGetSensorData(1);

% Get Laser Data to build the Map
Map = V.vGetLaserData(1); 

%% Find the objects vertices through the Map

[Min,Vert,Max]= H.ObjectSearch(Map);

%% Figure
figure(1)
hold on
axis([-6,6,-6,6])
h = H.PlotMap(Map,[],[],[]);
% h = H.PlotAnalysis(Map,Min,Vert,Max);

[Xv,Yv] = H.ProjectionFace(Map,Min,Vert,Max);
clear Min Vert Max;
[ObjList] = H.Orthogonal(Map,Xv,Yv,Goal);
clear Xv Yv;

[PathPoints, Path] =  H.BezierPath(Pos.X,ObjList,Goal);
[sp, spcost] = H.Dijkstra(Path, 1, size(ObjList,1)+2);

cell2mat(PathPoints(6,1))
%% Declarando o trace
Dados=[];

%% Parametros das funções
a1= 1.5; b = 1; T = 60; w = 2*pi/T;
%% Inicializando variáveis para o controle
it=0;
tmax=45;
tparcial=1;
t=tic;ta=tic; tp=tic;
flag = 0;

%% Rotina da lemniscata
while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        it=it+1;
        %% Robot 1
        %Trajetoria da Leminiscata como lugar desejado para o robô
        Pos.Xd1(1)= a*sin(w*toc(t));
        Pos.Xd1(2)= b*sin(2*w*toc(t));
        Pos.Xd1(7)= a*w*cos(w*toc(t));
        Pos.Xd1(8)= 2*b*w*cos(2*w*toc(t));
        Pos.theta1= atan2(Pos.X1(2)-Pos.Xc1(2),Pos.X1(1)-Pos.Xc1(1));
        
        K=[cos(Pos.theta1) -0.15*sin(Pos.theta1); sin(Pos.theta1) 0.15*cos(Pos.theta1)];

        % Get Position and Velocity from VREP
        [Pos.Xc1, Pos.X1, Pos.U1] = V.vGetSensorData(1);
        
        
        % Control System
        Pos.Xtil = Pos.Xd1([1 2])-Pos.X1;
        Pos.Ud1 = K\(Pos.Xd1([7 8])+0.5*tanh(1.5*Pos.Xtil([1 2])));
        
        % Send Control Signal to VREP
        V.vSendControlSignals(Pos.Ud1,1);
        
        % Trace
        Dados=[Dados ; [Pos.Xd1([1 2])' Pos.X1' Pos.Xtil' Pos.Ud1' toc(t)]];
        
        %% Try to delete PLot
        try
            delete(h)
        end
        
        % Get Laser Data to build the map
        Map = V.vGetLaserData(1);
        
        hold on
        %% Find the objects vertices through the Map
        [Min,Vert,Max] = H.ObjectSearch(Map);
        
        % Plot the Map and Object Tracking
        h = H.PlotMap(Map,Min,Vert,Max);
        drawnow
       
    end
end
%% Comando STOP Robots
Ud = [0; 0];
V.vSendControlSignals(Ud,1);

%% Desconecta Matlab e V-REP
V.vDisconnect;
