function [h] = PlotAnalysis(hpo,Map,Min,Vert,Max)

    Dif= zeros(1,length(Map(:,1)));
    for i=2:length(Map(:,1))
        Dif(i) = Map(i,3)-Map(i-1,3);
    end

%% Plot Test
    angle = linspace(-90,90,length(Dif));
    figure(2)
    h(5) = plot(angle,Dif,'LineWidth',1);
    axis([-90,90,min(Dif)-0.3,max(Dif)+0.3])
    grid
    hold on
    if isempty(Min)==0
        h(6) = plot(angle(Min),Dif(Min),'o', ...
            'MarkerSize', 8,'MarkerEdgeColor','b','MarkerFaceColor','b');
    end
    if isempty(Max)==0
        h(7) = plot(angle(Max+1),Dif(Max+1),'o', ...
            'MarkerSize', 8,'MarkerEdgeColor','b','MarkerFaceColor','b');
    end
    hold off
    
    %% Plot Test
    figure(3)
    h(8) = plot(angle,Map(:,3),'LineWidth',1);
    axis([-90,90,min(Map(:,3))-0.3,max(Map(:,3))+0.3])
    grid
    hold on
    if isempty(Min)==0
        h(8) = plot(angle(Min),Map(Min,3),'o', ...
            'MarkerSize', 8,'MarkerEdgeColor','b','MarkerFaceColor','b');
    end
    if isempty(Max)==0
        h(9) = plot(angle(Max),Map(Max,3),'o', ...
            'MarkerSize', 8,'MarkerEdgeColor','b','MarkerFaceColor','b');
    end
    if isempty(Vert)==0
       h(10) = plot(angle(Vert),Map(Vert,3),'o', ...
            'MarkerSize', 8,'MarkerEdgeColor','b','MarkerFaceColor','b');
    end
end