clear all 
close all
clc

%% Test Beta Version Class V-REP and HandlePushObj
% Handle Push Obj Class
H = HandlePushObj;

%% V-REP Class
V = VREP;
V.vConnect;
Variables;
%% Loading Pioneer Tags and Desired Position
V.vHandle('Pioneer_p3dx');  
V.vObject('Disc');
pause(2)

%% Get Destination point and Robot Position
[Goal,~] = V.vGetObjPosition('Disc');

[Pos.Xc,Pos.X, Pos.U] = V.vGetSensorData(1);

%% Get Laser Data to build the Map
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

%% Declarando o trace
Dados=[];
Save = [];

%% Main Rotine
while isempty(ObjList)==0
    % Calculate the paths
    [Bplot,PathPoints, Path] =  H.BezierPath(Map,Pos.X,ObjList,Goal);
    drawnow;

    % Find the Optimal Path
    [OptimalPath, ~] = H.Dijkstra(Path, 1, size(ObjList,1)+2);
    ObjList(OptimalPath(2)-1,:)=[];
    
    for i=1:2
        tmax = cell2mat(PathPoints(OptimalPath(2)-1,2*i-1));
        NavigationPath = cell2mat(PathPoints(OptimalPath(2)-1,2*i));
        t = tic;
        Track = [];
        while toc(t)<tmax
            %% Bezier Curve
            time = toc(t)/tmax;
            Curve = H.BezierCurve(NavigationPath,time);
            
            %% Set Desired Position
            Pos.Xd(1)= Curve(1);
            Pos.Xd(2)= Curve(2);
            Pos.Xd(7)= 0;
            Pos.Xd(8)= 0;
            
            % Get Position and Velocity from VREP
            [Pos.Xc, Pos.X, Pos.U] = V.vGetSensorData(1);
            
            Pos.theta= atan2(Pos.X(2)-Pos.Xc(2),Pos.X(1)-Pos.Xc(1));

            % Control System
            K=[cos(Pos.theta) -0.15*sin(Pos.theta); sin(Pos.theta) 0.15*cos(Pos.theta)];   

            Pos.Xtil = Pos.Xd([1 2])-Pos.X;
            Pos.Ud = K\(Pos.Xd([7 8])+1.4*tanh(1.2*Pos.Xtil([1 2])));

            
            
%             if i==2
%                 BoxMap = V.vGetLaserData(1);
%                 [Min,Vert,Max]= H.ObjectSearch(BoxMap);
%                 [~,I]= min(BoxMap(Min,3));
%                 Min = Min(I);
%                 [~,I]= min(BoxMap(Max,3));
%                 Max= Max(I);
%                 [~,I]= min(BoxMap(Vert,3));
%                 Vert= Vert(I);
%                 MidFace = (BoxMap(Min,1:2)+BoxMap(Max,1:2))/2;
%                 
%                 L = BoxMap(Min,1:2)-MidFace;
%                 Error = Pos.Xd([1 2])-MidFace';
%                 Save = [Save; [Error' MidFace]];
%                 
%                 if norm(Error)>0.15
%                    Pos.Ud = [0;0];
%                   
%                    V.vSendControlSignals(Pos.Ud,1);
%                    break;
%                 end
%             end
            
            % Send Control Signal to VREP
            V.vSendControlSignals(Pos.Ud,1);

            % Trace
            
            Dados=[Dados; [Pos.Xd([1 2])' Pos.X' Pos.Xtil' Pos.Ud' toc(t)]];
            
        end
    end
    delete(Bplot);
    Ud = [0; 0];
    V.vSendControlSignals(Ud,1);
end

%% Comando STOP Robots
% Ud = [0; 0];
% V.vSendControlSignals(Ud,1);

%% Desconecta Matlab e V-REP
V.vDisconnect;
