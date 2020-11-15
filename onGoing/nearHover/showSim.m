function showSim(XX,drone)
%showSIm Summary of this function goes here
%   Detailed explanation goes here


    % =====================================================================
    figure(1)
    axis([-3 3 -3 3 0 3])
    grid on
    hold on
    view(3)
    disp('Start..........')
    % =====================================================================
    ta = 0; 
    p = plot3(XX(1,1),XX(2,1),XX(3,1),'r*','MarkerSize',3,'LineWidth',2);

    for c = 1:length(XX)


        if XX(end,c)-ta > 1/10
            delete(p)
            ta = XX(end,c);
            p = plot3(XX(1,1),XX(2,1),XX(3,1),'r*','MarkerSize',3,'LineWidth',2);
        end

        drone.pPos.X = XX(13:24,c);
        drone.mCADplot;
        drawnow
    end
end

