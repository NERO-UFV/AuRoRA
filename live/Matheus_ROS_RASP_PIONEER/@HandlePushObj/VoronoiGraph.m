%% VoronoiGraph
function [MatrixVoronoi,vertex] = VoronoiGraph(hpo,Map,ObjList,Robot,Goal)
%% Create Voronoi Diagram

    Xmaximo = max(Map(:,1));
    Xminimo = min(Map(:,1));
    Ymaximo = max(Map(:,2));

    limitx1 = linspace(Xminimo,Xmaximo,50);
    limity1 = -Ymaximo*ones(1,50);

    limitx2 = Xmaximo*ones(1,50);
    limity2 = linspace(-Ymaximo,Ymaximo,50);
    
    xcirc = 0.3*cos(0:pi/6:2*pi)+Goal(1);
    ycirc = 0.3*sin(0:pi/6:2*pi)+Goal(2);

    x = [ObjList(:,7); Goal(1); xcirc'; limitx1'; limitx1'; limitx2'; -limitx2'];
    y = [ObjList(:,8); Goal(2); ycirc'; limity1'; -limity1'; limity2'; limity2'];

    [vertex,c] = voronoin([double(x) double(y)]);

    %% Create Adjacent List
    for i=1:length(vertex(:,1))
        Node = [];
       for j=1:length(c) 
           celula = c{j}; 
           k = find(celula==i);
           if isempty(k)==0
               if celula(k)==celula(1)
                   Node = [Node celula(k+1) celula(end)];
               elseif celula(k)==celula(end)
                   Node = [Node celula(end-1) celula(1)];
               else
                   Node = [Node celula(k-1) celula(k+1)];
               end
           end
       end
           AdjList{i,1} = [Node];
    end

    %% Order the Nodes on Adjacent List
    for i=1:length(AdjList)
        v2 = unique(AdjList{i});
        AdjList{i}=v2;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    L =length(AdjList);
    for i=1:L
        
%         i=1;
%         i=i+1;
        v2 = AdjList{i};
        list = [];
        for j=1:length(v2)
            len = sqrt((vertex(i,1)-vertex(v2(j),1))^2+(vertex(i,2)-vertex(v2(j),2))^2);
            if len > 0.6 && len < 1000
                  list = [list v2(j)];
%            
            end
            
        end
        if isempty(list)==0
            
            for n=1:length(list)
                No1 = i;
                No2 = list(n);
                u(:,1) = linspace(vertex(No1,1),vertex(No2,1),5);
                u(:,2) = linspace(vertex(No1,2),vertex(No2,2),5);

                indice1= length(vertex(:,1));
                vertex = [vertex; u(2:4,1) u(2:4,2)];
                indice2= length(vertex(:,1));
                
                v2 = v2(v2~=list(n));
                v2 = [v2 indice1+1];
                
                v3 = AdjList{list(n)};
                v3 = v3(v3~=i);
                v3 = [v3 indice2];
                AdjList{list(n)} = v3;
                
                for ii=indice1+1:indice2-1
                    AdjList{ii}= [ii+1];
                    if ii+1==indice2
                        AdjList{indice2} = list(n);
                    end
                end
            end
            AdjList{i}=v2;
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%
    Proximity = [Robot ObjList(:,1:2)' ObjList(:,5:6)' Goal(1:2)'];

    for i=1:length(Proximity(1,:))
        zone = sqrt((vertex(:,1)-Proximity(1,i)).^2+(vertex(:,2)-Proximity(2,i)).^2);
        [~,k] = min(zone);
        AdjList{end+1}= [k];
    end

    vertex = [vertex; Proximity'];

    % Plot Voronoi Graph
%     hold on
%     for i=1:length(AdjList)
%             v2 = AdjList{i};
%             for j=1:length(v2)
%                  plot([vertex(i,1) vertex(v2(j),1)],[vertex(i,2) vertex(v2(j),2)],'k');
%             end
%     
%     end
%     
%     plot(vertex(:,1),vertex(:,2),'o', ...
%                     'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'k');
%     %% Create Voronoi Adj Matrix
    MatrixVoronoi = inf(length(vertex(:,1)));

    for i=1:length(AdjList)
        v2 = AdjList{i};
        for j=1:length(v2)

            weigth = sqrt((vertex(i,1)-vertex(v2(j),1))^2+(vertex(i,2)-vertex(v2(j),2))^2);
            MatrixVoronoi(i,v2(j)) = weigth;
            MatrixVoronoi(v2(j),i) = weigth;

        end

    end

    % % First and end point
    % first=210;
    % final=212;
    % 
    % plot(vertex(first,1),vertex(first,2),'o', ...
    %             'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r')
    % 
    % plot(vertex(final,1),vertex(final,2),'o', ...
    %     'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r')
    % 
    % [s, ~] = H.Dijkstra(MatrixVoronoi,first,final);
    % % 
    % plot(vertex(s,1),vertex(s,2),'r','LineWidth',2)
end
