% clear all 
% close all
% clc
% 
% 
% %% Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
% addpath(genpath(pwd));
% 
% %% Test Beta Version Class V-REP and HandlePushObj
% % Handle Push Obj Class
% H = HandlePushObj;
% P = Pioneer3DX;
% OPT = OptiTrack;
% OPT.Initialize;



%% Get Destination point and Robot Position
Goal = [1. 1.5];
P.pPar.ti=tic;
rb = OPT.RigidBody;
P = getOptData(rb,P);


%% Get Laser Data to build the Map
map = sub.LatestMessage;
Map = [cos(P.pPos.X(6)) -sin(P.pPos.X(6)); sin(P.pPos.X(6)) cos(P.pPos.X(6))]*map.readCartesian' +P.pPos.X([1 2]); 

L = length(Map(1,:));
Map = [Map' map.Ranges(1:L)];

n = 1:2:length(Map(:,1));
Map(n,:) = []; 
%% Find the objects vertices through the Map
[Min,Vert,Max]= H.ObjectSearch(Map);

%% Figure
figure(1)
hold on
axis([-3.5,3.5,-3.5,3.5])
h = H.PlotMap(Map,[],[],[]);
% h = H.PlotAnalysis(Map,Min,Vert,Max);

[Xv,Yv] = H.ProjectionFace(Map,Min,Vert,Max);
clear Min Vert Max;
[ObjList] = H.Orthogonal(Map,Xv,Yv,Goal);
clear Xv Yv;

%% Declarando o trace
Dados=[];
Save = [];
flag =0;
%% Main Rotine
while isempty(ObjList)==0
    % Calculate the paths
    [Bplot,PathPoints, Path] =  H.BezierPath(Map,P.pPos.X([1 2]),ObjList,Goal);
    drawnow;

    % Find the Optimal Path
    [OptimalPath, ~] = H.Dijkstra(Path, 1, size(ObjList,1)+2);
    ObjList(OptimalPath(2)-1,:)=[];
    tempo_manouver = tic;
    t = tic;
    for i=1:2
        tmax = cell2mat(PathPoints(OptimalPath(2)-1,2*i-1));
        NavigationPath = cell2mat(PathPoints(OptimalPath(2)-1,2*i));
        
        
        %% CAMINHO
        
        Curve = H.BezierCurve(NavigationPath);
        
        path.n = length(Curve(1,:));
        [path.min,path.k] = min(sqrt((P.pPos.X(1) - Curve(1,:)).^2 + (P.pPos.X(2) - Curve(2,:)).^2));
        
        if path.k == 1
            path.k = path.k + 1;
            path.beta = atan2((Curve(2,path.k) - Curve(2,path.k-1)),(Curve(1,path.k) - Curve(1,path.k-1)));
        end
        
        
        P.pPos.Xd([1 2]) = [Curve(1,path.k); Curve(2,path.k)];
        
        vmax = .32;
        k =1.5;
        %%
        
        
        Track = [];
        t_amostragem = 0.1;
        t_loop = tic;
        
        while path.k < path.n
            if toc(t_loop) > t_amostragem
                t_loop = tic;
                %% Bezier Curve
                
                %% Caminho
                
                [path.min,path.k]= min(sqrt((P.pPos.X(1) - Curve(1,:)).^2 + (P.pPos.X(2) - Curve(2,:)).^2));
                
                
                %%
                if path.min < 0.15 && path.k < path.n
                    path.k = path.k + 1;
                    %disp(path.k)
                    path.betaA = path.beta;
                    path.beta = atan2((Curve(2,path.k) - Curve(2,path.k-1)),(Curve(1,path.k) - Curve(1,path.k-1)));
                    path.dbeta = path.beta - path.betaA;
                    
                    P.pPos.Xd([7 8]) = ...
                        ([vmax/(1 + k*abs(path.dbeta))*cos(path.beta);
                        vmax/(1 + k*abs(path.dbeta))*sin(path.beta)]);
                    
                    
                    
                end
                
                if path.k == path.n
                    P.pPos.Xd([7 8]) = [0;0];
                end
                
                
                
                
                %% Set Desired Position
                P.pPos.Xd([1 2]) = [Curve(1,path.k); Curve(2,path.k)];
                
                % Get Position and Velocity from Optitrack
                rb = OPT.RigidBody;
                P = getOptData(rb,P);
                
                
                % Control System
                K=[cos(P.pPos.X(6)) -0.15*sin(P.pPos.X(6)); sin(P.pPos.X(6)) 0.15*cos(P.pPos.X(6))];
                
                P.pPos.Xtil = P.pPos.Xd([1 2])-P.pPos.X([1 2]);
                
                P.pSC.Ud = K\(P.pPos.Xd([7 8])+0.8*P.pPos.Xtil([1 2]));
                
%                 if i==2
%                     RealTimeControl;
%                    
%                 end
%                 
                % Send Control Signal to Pioneer
                cmd_vel.Linear.X = P.pSC.Ud(1);
                cmd_vel.Angular.Z = P.pSC.Ud(2);
                send(pub,cmd_vel)
                
                % Trace
                
                Dados=[Dados; [P.pPos.Xd([1 2])' P.pPos.X' P.pPos.Xtil' P.pSC.Ud' toc(t)]];
                
                try
                    delete(h1)
                end
                
                h1 = plot(Dados(:,3),Dados(:,4),'.r');
                drawnow
                
            end
            
        end
        
    end
    t_manouver = tic;
    while toc(t_manouver)<2
        Ud = [-0.2; 0];
        cmd_vel.Linear.X = Ud(1);
        cmd_vel.Angular.Z =Ud(2);
        send(pub,cmd_vel)
    end
    flag=0;
    delete(Bplot);
   
end
%% Comando STOP Robots
Ud = [0; 0];
cmd_vel.Linear.X = Ud(1);
cmd_vel.Angular.Z =Ud(2);
send(pub,cmd_vel)