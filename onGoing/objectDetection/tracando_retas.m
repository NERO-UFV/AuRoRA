close all
clearvars
clc

V = VREP;
V.vConnect
V.vHandle('Pioneer_p3dx')
for i = 1:2
    Map = V.vGetLaserData(1); 
    pause(i/10)
end
disp('parado')
% [r,s,disc,Map]=laserSweeper(Read); 

%% Cartesian lin-reg:
alphas = []; % angular coeffs
betas = []; % independent coeffs
k = []; % index
% regLine=zeros(size(Map(:,2),1)/3,2);

Points=[];
t = tic;
for kk = 15:15:size(Map(:,2),1)
    X = [Map(kk,1),Map(kk-1,1),Map(kk-2,1),...
         Map(kk-3,1),Map(kk-4,1),Map(kk-5,1),...
         Map(kk-6,1),Map(kk-7,1),Map(kk-8,1)];
    Y = [Map(kk,2),Map(kk-1,2),Map(kk-2,2),...
         Map(kk-3,2),Map(kk-4,2),Map(kk-5,2),...
         Map(kk-6,2),Map(kk-7,2),Map(kk-8,2)];
%     poiX = [Map(kk,1),Map(kk-1,1),Map(kk-2,1)]; % 3 last X
%     poiY = [Map(kk,2),Map(kk-1,2),Map(kk-2,2)]; % 3 last Y
    regLine = polyfit(X,Y,1); % linear regression
%     alphas(kk) = regLine(1);
%     betas(kk) = regLine(2);
%     if abs(alphas(kk))>100
%         alphas(kk)=0;
%         betas(kk)=0;
%     elseif abs(alphas(kk))<10^-3
%         alphas(kk)=0;
%     end
    
%     k = [k,kk];
%         
%     figure(1)
%     hold on
    Ylinha=polyval(regLine,X);
%     p1 = plot(X,Ylinha,'b');
    Points = [X(1),Ylinha(1);Points];
%     pause(0.1)
end


% Proposta de seguimento de reta por regressão (+/-)
plot(Points(:,1),Points(:,2),'Color','black','LineStyle','-.')
hold on
reta = polyfit(Points(:,1),Points(:,2),1);
yr = polyval(reta,Points(:,1));
plot(Points(:,1),yr,'Color','red','LineStyle','--')

% [vx,vy]=voronoi(double(Points(:,1)),double(Points(:,2))); %Voronoi
% regLine = polyfit(vx,vy,1);
% vylinha=polyval(regLine,vx);
% plot(vx,vylinha)

%% Analysis
% figure(1)
% title('Ground truth vs lin-reg')
% hold on
% plot(Map(:,1),Map(:,2),'.black')
% legend('Lin-reg')
% 
% figure(2)
% stem(1:size(alphas,2),alphas)
% title('Alphas')
% 
% figure(3)
% stem(1:size(betas,2),betas)
% title('Betas')
% 
% figure
% t  = linspace(1,100,1000);
% plot(t,alphas(12).*t + betas(12))

%% OLD CODE FOR Vers.Contr.:

% for kk = 3:3:size(Map(:,2),1)
%     poiX = [Map(kk,1),Map(kk-1,1),Map(kk-2,1)]; % 3 last X
%     poiY = [Map(kk,2),Map(kk-1,2),Map(kk-2,2)]; % 3 last Y
%     regLine = polyfit(poiX,poiY,1); % linear regression
%     alphas(kk) = regLine(1);
%     betas(kk) = regLine(2);
%     if abs(alphas(kk))>100
%         alphas(kk)=0;
% %         betas(kk)=0;
%     elseif abs(alphas(kk))<10^-3
%         alphas(kk)=0;
%     end
%     
%     k = [k,kk];
%         
%     figure(1)
%     hold on
%     p1 = plot(poiX,polyval(regLine,poiX),'b'); 
%     pause(0.1)
% end

%% Polar lin-reg:
% figure
% for kk = 3:3:size(Map(:,2),1)
%     ang = [kk,kk-1,kk-2];
% %     poiX = [redux(kk),redux(kk-1),redux(kk-2)]; % points of interest(3lastX)
% %     poiY = [reduy(kk),reduy(kk-1),reduy(kk-2)]; % points of interest(3lastX)
%     poi = [Map(kk,3),Map(kk-1,3),Map(kk-2,3)]; % points of interest(3lastX)
% %     ang = [Map(kk,2),Map(kk-1,2),Map(kk-2,2)]; % points of interest(3lastX)
%     regLine = polyfit(ang,poi,1); % linear regression
% %     regLine = polyfit(poi,ang,1); % linear regression
%     hold on
% %     plot(poi,polyval(regLine,poi),'r')
%     plot(ang,polyval(regLine,ang),'r')
% end

toc(t)
