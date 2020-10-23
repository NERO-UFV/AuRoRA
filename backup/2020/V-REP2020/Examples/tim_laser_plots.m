%% IEEE TIM Laser Obj-Recog
% Gráficos e simulação para validação da teoria proposta
% Alexandre (07/08)
 
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
%Plots
createfigure(1:size(Map,1),Map(1:end,3),'ANGLE VS DISTANCE MEASUREMENTS','\theta_n [ °]','D_n [m]','D_n = r(\theta_n)')
 
createfigure(1:size(Map,1)-1,Map(2:end,3)-Map(1:end-1,3),'ANGLE VS DISTANCE DIFFERENCE','\theta_n [ °]','d_n [m]','d_n = s(\theta_n)')
 
%% Detectar objetos e marcar gráfico
 
[Min,Vert,Max]= H.ObjectSearch(Map);
figure
hold on
axis([-6,6,-6,6])
h = H.PlotMap(Map,Min,Vert,Max);
 
f = figure;
axes1 = axes('Parent',f);
f(1) = plot(1:2,[Map(1,3) Map(2,3)],'b-');
hold on
for i = 2:size(Map,1)
   switch i
       case Min
           f(1) = plot(i,Map(i,3),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
                        'Marker','o',...
                        'LineStyle','none',...
                        'Color',[0 0 1]);
       case Max
           f(1) = plot(i-1:i,[Map(i-1,3) Map(i,3)],'b-');
           hold on
           f(1) = plot(i,Map(i,3),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
                        'Marker','o',...
                        'LineStyle','none',...
                        'Color',[0 0 1]);
       case Vert
           f(1) = plot(i-1:i,[Map(i-1,3) Map(i,3)],'b-');
           hold on
             plot(i,Map(i,3)-Map(i,3)*.02,'MarkerFaceColor',[1 0 0],'Marker','^','LineStyle','none',...
                'Color',[1 0 0]);
       case Max+1
           f(1) = plot(i,Map(i,3),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
                        'Marker','o',...
                        'LineStyle','none',...
                        'Color',[0 0 1]);
       case Min-1
           f(1) = plot(i-1:i,[Map(i-1,3) Map(i,3)],'b-');
           hold on
           f(1) = plot(i,Map(i,3),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
                        'Marker','o',...
                        'LineStyle','none',...
                        'Color',[0 0 1]);
       otherwise
           f(1) = plot(i-1:i,[Map(i-1,3) Map(i,3)],'b-');
   end
   hold on
end
 
grid on
title('ANGLE VS DISTANCE MEASUREMENTS')
ylabel('D [m]');
xlabel('\theta [ º]');
legend('D = f(\theta)')
xlim(axes1,[0 184]);
set(axes1,'FontSize',14,'XGrid','on','YGrid','on');
 
 
%% Disconnects MATLAB / V-REP
V.vDisconnect;