close all
clearvars
clc

V = VREP;
V.vConnect
V.vHandle('Pioneer_p3dx')
for i = 1:2
    Map = V.vGetRealLaserData(1); 
    pause(i/10)
end
disp('parado')
% [r,s,disc,Map]=laserSweeper(Read); 

%% Cartesian lin-reg:
alphas = []; % angular coeffs
betas = []; % independent coeffs
k = []; % index
LinMap=zeros(12,2); % 
% regLine=zeros(size(Map(:,2),1)/3,2);


t = tic;
for kk = 15:15:size(Map(:,2),1)
    X = [Map(kk,1),Map(kk-1,1),Map(kk-2,1),...
         Map(kk-3,1),Map(kk-4,1),Map(kk-5,1),...
         Map(kk-6,1),Map(kk-7,1),Map(kk-8,1)];
    Y = [Map(kk,2),Map(kk-1,2),Map(kk-2,2),...
         Map(kk-3,2),Map(kk-4,2),Map(kk-5,2),...
         Map(kk-6,2),Map(kk-7,2),Map(kk-8,2)];
     
    b = X\Y;
    Ylinha = X*b;
    
    LinMap(kk/15,1) = X(1);
    LinMap(kk/15,2) = Ylinha(1);    
end

XX = LinMap(:,1);
YY = LinMap(:,2);
b1 = XX\YY;
Yreta = XX*b1;

plot(LinMap(:,1),LinMap(:,2),'r*')
% hold on
% plot(XX,Yreta,'--')

%% Relembrando:
med = mean(LinMap);
hold on
plot(med(1),med(2),'r*')

% Proposta de seguimento de reta por regressão (+/-)
% plot(Points(:,1),Points(:,2),'Color','black','LineStyle','-.')
% hold on
% reta = polyfit(Points(:,1),Points(:,2),1);
% yr = polyval(reta,Points(:,1));
% plot(Points(:,1),yr,'Color','red','LineStyle','--')

% [vx,vy]=voronoi(double(Points(:,1)),double(Points(:,2))); %Voronoi
% regLine = polyfit(vx,vy,1);
% vylinha=polyval(regLine,vx);
% plot(vx,vylinha)

%% Analysis
figure(1)
title('Ground truth vs lin-reg')
hold on
plot(Map(:,1),Map(:,2),'.black')
legend('Lin-reg')
 
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

toc(t)
