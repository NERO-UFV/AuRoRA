Path = inf(size(ObjList,1)+2);
PathPoints = {};
speed = 0.7; % m/s

% Create Voronoi Matrix
[MatrixVoronoi,vertex] = H.VoronoiGraph(Map,ObjList,Robot,Goal);

i=0;
i=i+1;
Robot = Pos.X;

 Control = [];
        
% Robot position and objet points on Voronoi graph
first=length(MatrixVoronoi(:,1))-2*length(ObjList(:,1))-1;
final=length(MatrixVoronoi(:,1))-2*length(ObjList(:,1))-1+i;

%         plot(vertex(first,1),vertex(first,2),'o', ...
%                     'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r')
%         
%         plot(vertex(final,1),vertex(final,2),'o', ...
%             'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r')
%         
% Find the shorter path on Voronoi Graph

[s, ~] = H.Dijkstra(MatrixVoronoi,first,final);

%       plot(vertex(s,1),vertex(s,2),'k','LineWidth',2)


% Avoid vertex from Voronoi
Avoid= [vertex(s,1)'; vertex(s,2)'];

% Removing extra points on voronoi path
Sequence = [Robot Avoid];
[Pcontrol] = H.RemovalPoint(Sequence);

Control = [Robot Pcontrol ObjList(i,1:2)' ObjList(i,3:4)'];

hold on
plot(Control(1,:),Control(2,:),'o')

t = linspace(0,1,length(Control(1,:))-1);
time = 0:0.001:1;

x = interp1(t,Control(1,2:end),time,'makima');
y = interp1(t,Control(2,2:end),time,'makima');
plot(x,y,'k')




