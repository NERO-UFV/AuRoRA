function [Bplot,PathPoints,Path] =  BezierPath(hpo,Map,Robot,ObjList,Goal)
    %% This function calculate the bezier curve for all objects
    Path = inf(size(ObjList,1)+2);
    PathPoints = {};
    speed = 0.4; % m/s
    
    % Create Voronoi Matrix
    [MatrixVoronoi,vertex] = hpo.VoronoiGraph(Map,ObjList,Robot,Goal);
        
    for i=1:size(ObjList,1)
%         i =1;
%         Robot = Pos.X;
%        %% First part Path
        % Build the Path with Control Points
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
        
        [s, ~] = hpo.Dijkstra(MatrixVoronoi,first,final);
 
%       plot(vertex(s,1),vertex(s,2),'k','LineWidth',2)
        

        % Avoid vertex from Voronoi
        Avoid= [vertex(s,1)'; vertex(s,2)'];
        
        % Removing extra points on voronoi path
        Sequence = [Robot Avoid];
        [Pcontrol] = hpo.RemovalPoint(Sequence);
        
        Control = [Robot Pcontrol ObjList(i,1:2)' ObjList(i,3:4)'];
        
        Curve1 = hpo.BezierCurve(Control);
        b1(i) = plot(Curve1(1,:),Curve1(2,:),'k');
        
        [arclen,~] = hpo.ArcLength(Curve1(1,:),Curve1(2,:));
        Path(1,i+1) = arclen;
        Path(i+1,1) = arclen;
        
        time = arclen/speed;
        PathPoints(i,1) = {time};
        PathPoints(i,2) = {Control};
        %% Second part Path
        % Build the Path
        Control = [];
       
        % Calculate the curve
        
        first=length(MatrixVoronoi(:,1))-length(ObjList(:,1))-1+i;
        final=length(MatrixVoronoi(:,1));
        
%         plot(vertex(first,1),vertex(first,2),'o', ...
%                     'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r')
%         
%         plot(vertex(final,1),vertex(final,2),'o', ...
%             'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r')
%         
        [s, ~] = hpo.Dijkstra(MatrixVoronoi,first,final);
        
        Avoid= [vertex(s,1)'; vertex(s,2)'];
        
        
        Sequence = [Avoid];
        [Pcontrol] = hpo.RemovalPoint(Sequence);
        
        Control = [ObjList(i,3:4)' ObjList(i,5:6)' Pcontrol Goal([1 2])'];
        Curve2 = hpo.BezierCurve(Control);
        b2(i) = plot(Curve2(1,:),Curve2(2,:),'k');
        
        [arclen,~] = hpo.ArcLength(Curve2(1,:),Curve2(2,:));
        Path(i+1,end) = arclen;
        Path(end,i+1) = arclen;
        
        time = arclen/speed;
        PathPoints(i,3) = {time};
        PathPoints(i,4) = {Control};
        
        Bplot(i,:) = [b1(i) b2(i)];
    end

end