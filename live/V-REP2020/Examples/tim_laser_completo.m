%% IEEE TIM Laser Obj-Recog
% Gráficos e simulação para validação da teoria proposta
% Alexandre (07/08/19)

%% Cleanup, call API and connect to V-REP
clear vars
close all
clc

disp('tim_laser')
V = VREP;
V.vConnect;
Variables;

% Handle Push Obj Class
H = HandlePushObj;
%% Load and stop Pioneer
V.vHandle('Pioneer_p3dx');
Ud = [0; 0];
V.vSendControlSignals(Ud,1);

%% Collecting and plotting measurements
% Get Robot Position
[Pos.Xc,Pos.X, Pos.U] = V.vGetSensorData(1);

% Get Laser Data
for i = 1:2
    Map = V.vGetLaserData(1); 
    pause(i)
end

% Disconnect
V.vDisconnect;


%% Plots
rth = createfigure(1:size(Map,1),Map(1:end,3),'ANGLE VS DISTANCE MEASUREMENTS','\theta_n [ °]','D_n [m]','D_n = r(\theta_n)');
sth = createfigure(1:size(Map,1)-1,Map(2:end,3)-Map(1:end-1,3),'ANGLE VS DISTANCE DIFFERENCE','\theta_n [ °]','d_n [m]','d_n = s(\theta_n)');
s = Map(2:end,3)-Map(1:end-1,3);
% Detectar objetos e marcar gráfico

% [Min,Vert,Max]= H.ObjectSearch(Map);
% figure
% hold on
% axis([-6,6,-6,6])
% h = H.PlotMap(Map,Min,Vert,Max);


f = figure;
axes1 = axes('Parent',f);
f(1) = plot(1:2,[Map(1,3) Map(2,3)],'b-');
disc = [];
grid on
hold on
for i = 2:size(s)-1 
    if s(i) >= max(s)*0.1
        disc =[disc,i];
        f(1) = plot(i-1:i,[Map(i-1,3) Map(i,3)],'b-');
        f(1) = plot(i,Map(i,3),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
                        'Marker','o',...
                        'LineStyle','none',...
                        'Color',[0 0 1]);
    elseif s(i-1) >= max(s)*.1
        f(1) = plot(i-1,Map(i,3),'MarkerFaceColor',[1 1 1],...
                        'Marker','o',...
                        'LineStyle','none',...
                        'Color',[0 0 1]);
        f(1) = plot(i:i+1,[Map(i,3) Map(i+1,3)],'b-');
    elseif s(i-1) <= -max(s)*.1
        disc =[disc,i];
        f(1) = plot(i,Map(i-1,3),'MarkerFaceColor',[1 1 1],...
                        'Marker','o',...
                        'LineStyle','none',...
                        'Color',[0 0 1]);
        hold on
        f(1) = plot(i,Map(i,3),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
                        'Marker','o',...
                        'LineStyle','none',...
                        'Color',[0 0 1]);
    else
        f(1) = plot(i-1:i,[Map(i-1,3) Map(i,3)],'b-');
    end
end

% Mark local minima (edges facing robot):
maxx = max(Map(:,3));
minn = min(Map(:,3));
for i = 0:(size(disc,2)/2)-1
    hold on
    n = 2*i+1;
    aresta = min(Map(disc(n):disc(n+1),3));
    m = find(Map(:,3) == aresta);
    f(1)= plot(m,aresta-0.09,'Marker','^','MarkerFaceColor',[1 0 0],...
                             'Color','r','LineWidth',1.3);
end

grid on
title('ANGLE VS DISTANCE MEASUREMENTS')
ylabel('D [m]');
xlabel('\theta [ º]');
legend('D = f(\theta)')
xlim(axes1,[0 184]);
set(axes1,'FontSize',14,'XGrid','on','YGrid','on');

%% Disconnects MATLAB / V-REP

saveas(rth,'r_clutter.pdf')
saveas(sth,'s_clutter.pdf')
saveas(f,'f_clutter.pdf')
save('Map.mat','Map')

%%
load('clutter.mat')
rth = createfigure(1:size(Map,1),clutter(1:end,3) - Map(1:end,3),'Noise cancelled','\theta_n [ °]','D_n [m]','D_n = r(\theta_n)');

%%
% O=struct('objects',[]);
% for ii = 1:2:size(disc,2)
%     O.objects = [O.objects,disc(ii):disc(ii+1)];
% end

% for ii = 1:size(Map,1)
%     if find(disc==ii)
%         %Do nothing, region of object face or vertex.
%     else
%         wallX = [wallX,Map(ii,1)];
%         wallY = [wallY,Map(ii,1)]; 
%     end
% end
% plot(Map(ii,1),Map(ii,2),'.b')

% Map(:,1) = X coord.
% Map(:,2) = Y coord.
% Map(:,3) = distance measurements
% redux =[]; reduy =[];
% for ii = 1:1:size(Map,1)
%     redux = [redux,Map(ii,1)];
%     reduy = [reduy,Map(ii,2)];
% end
% 

figure
for kk = 3:3:size(Map(:,2),1)
    ang = [kk,kk-1,kk-2];
%     poiX = [redux(kk),redux(kk-1),redux(kk-2)]; % points of interest(3lastX)
%     poiY = [reduy(kk),reduy(kk-1),reduy(kk-2)]; % points of interest(3lastX)
    poi = [Map(kk,3),Map(kk-1,3),Map(kk-2,3)]; % points of interest(3lastX)
%     ang = [Map(kk,2),Map(kk-1,2),Map(kk-2,2)]; % points of interest(3lastX)
    regLine = polyfit(ang,poi,1); % linear regression
%     regLine = polyfit(poi,ang,1); % linear regression
    hold on
%     plot(poi,polyval(regLine,poi),'r')
    plot(ang,polyval(regLine,ang),'r')
end


