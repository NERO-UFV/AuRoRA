function PlotControlPoints(ControlPoints)
% Ponto de controle:                 (AZUL)
        hold on
        plot3(ControlPoints(:,1),ControlPoints(:,2),ControlPoints(:,3),'b*')
        hold on
        plot3(ControlPoints(:,1),ControlPoints(:,2),ControlPoints(:,3),'bo')
        hold on
        plot3(ControlPoints(:,1),ControlPoints(:,2),ControlPoints(:,3),'b.:')
end

