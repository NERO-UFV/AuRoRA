function [h] = PlotMap(hpo,Map,Min,Vert,Max)
    % Plot Map
    h(1)= plot(Map(:,1),Map(:,2),'.b');
    
    % Plot vertices if it is possible
    if isempty(Min)==0
        h(2) = plot(Map(Min,1),Map(Min,2),'o', ...
            'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r');
    end
    if isempty(Max)==0
        h(3) = plot(Map(Max,1),Map(Max,2),'o', ...
            'MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r');

    end

     if isempty(Vert)==0
        h(4) = plot(Map(Vert,1),Map(Vert,2),'o', ...
            'MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r');

    end

end