classdef HandlePushObj < handle
    
    properties
        ObjectList; % Objects indentified 
    end
    % In a methods block, set the method attributes
    % and add the function signature
    methods
        %% Constructor Class
        function hpo = HandlePushObj
           
        end
      
        %% Object Searching
        [Min,Vert, Max] = ObjectSearch(hpo, Map);
        [tf] = Collinear(hpo,P, tol);
        [Xv,Yv] = ProjectionFace(hpo,Map,Min,Vert,Max);
        
        % Ploting Important Objects
        [h] = PlotMap(hpo,Map,Min,Vert,Max);
        [h] = PlotAnalysis(hpo,Map,Min,Vert,Max);
        
        %% Bezier Path Planning
        [ObjList] = Orthogonal(hpo,Map,Xv,Yv,Goal);
        [Bplot,PathPoints,Path] =  BezierPath(hpo,Map,Robot,ObjList,Goal);
        [Curve] = BezierCurve(hpo,ControlPoints,time)
        [MatrixVoronoi,vertex] = VoronoiGraph(hpo,Map,ObjList,Robot,Goal);
        [arclen,seglen] = ArcLength(hpo,px,py,varargin);
        
        %% Box Control
        [Xcontrol,Ycontrol] = BoxControl(hpo,BoxShape,Robot,Bezier)
        
        %% Dijkstra
        [sp, spcost] = Dijkstra(hpo,matrix_adj, s, d);
        
        %% Priority Queue
        % ==================================================        
       
   
    end
end