function [r,s,disc,Map] = laserSweeper(Map) %add f back to return variables
%laserSweeper Formal LiDAR representation as presented in article XX.
%   Receives VREP robot class and exports r,s and f. 
%   [Detailed explanation goes here]

%     V.vHandle('Pioneer_p3dx')
%     % Get Laser Data
%     
%     % If robot is moving, don't stop:
%     mov = V.vGetSensorData(1);
%     if mov(1)==0
%         for i = 1:2
%             Map = V.vGetLaserData(1); 
%             pause(i/10)
%         end
%         disp('parado')
%     else
%         Map = V.vGetLaserData(1);
%     end
    
    if isempty(Map)
        disp('Empty Map! Check if simulation is running.')
    end
    
    r = Map(:,3);
    s = Map(2:end,3)-Map(1:end-1,3);
    
    %$$$$$$$$$$$$$$$Retirar createfigure!
    %% Plot r(theta):
%     rfig = createfigure(1:size(Map,1),Map(1:end,3),...
%                        'ANGLE VS DISTANCE MEASUREMENTS','\theta_n [ °]',...
%                        'D_n [m]','D_n = r(\theta_n)');

    %% Plot s(theta):
%     sfig = createfigure(1:size(Map,1)-1,Map(2:end,3)-Map(1:end-1,3),...
%                        'ANGLE VS DISTANCE DIFFERENCE','\theta_n [ °]',...
%                        'd_n [m]','d_n = s(\theta_n)');
       
    %% Plot f(theta) and find discontinuities:
%     f = figure;
%     axes1 = axes('Parent',f);
%     f(1) = plot(1:2,[Map(1,3) Map(2,3)],'b-');
%     disc = zeros(1,size(Map,1));
    disc = [];
%     grid on
%     hold on
    for i = 2:size(s)-1 
        if s(i) >= max(s)*0.1
%             disc(i) = i;
            disc =[disc,i];
%             f(1) = plot(i-1:i,[Map(i-1,3) Map(i,3)],'b-');
%             f(1) = plot(i,Map(i,3),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
%                             'Marker','o',...
%                             'LineStyle','none',...
%                             'Color',[0 0 1]);
        elseif s(i-1) >= max(s)*.1
%             f(1) = plot(i-1,Map(i,3),'MarkerFaceColor',[1 1 1],...
%                             'Marker','o',...
%                             'LineStyle','none',...
%                             'Color',[0 0 1]);
%             f(1) = plot(i:i+1,[Map(i,3) Map(i+1,3)],'b-');
        elseif s(i-1) <= -max(s)*.1
            disc =[disc,i];
%             disc(i) = i;
%             f(1) = plot(i,Map(i-1,3),'MarkerFaceColor',[1 1 1],...
%                             'Marker','o',...
%                             'LineStyle','none',...
%                             'Color',[0 0 1]);
%             hold on
%             f(1) = plot(i,Map(i,3),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
%                             'Marker','o',...
%                             'LineStyle','none',...
%                             'Color',[0 0 1]);
        else
%             f(1) = plot(i-1:i,[Map(i-1,3) Map(i,3)],'b-');
        end
    end

    % Mark local minima (edges facing robot):
    for i = 0:(size(disc,2)/2)-1
%         hold on
        n = 2*i+1;
        aresta = min(Map(disc(1,n):disc(1,n+1),3));
        m = find(Map(:,3) == aresta);
%         f(1)= plot(m,aresta-0.09,'Marker','^','MarkerFaceColor',[1 0 0],...
%                                  'Color','r','LineWidth',1.3);
    end
%     grid on
%     title('ANGLE VS DISTANCE MEASUREMENTS')
%     ylabel('D [m]');
%     xlabel('\theta [ º]');
%     legend('D = f(\theta)')
%     xlim(axes1,[0 184]);
%     set(axes1,'FontSize',14,'XGrid','on','YGrid','on');
end

