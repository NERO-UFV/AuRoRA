%% Plotando experimentos (01 e 02/01/20)
% Boas práticas
clearvars
close all
clc
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
cd 'C:\Users\alexa\Documents\MEGA\MEGAsync\NERO_online\Txt_sketches\ICUAS 2020\Backup\data';
% Carregar dados dos bons resultados: 

% dois bamboles uav_up_exp5T174442 / uav_up_exp5T174209 / ??
% size(XX) == 25   909;  uav_up_exp5T174027
% size(XX) == 25   1681;    uav_up_exp5T174209

load 'uav_up_exp5T174209.mat' % DOIS BAMBOLES                           (0) Vmax = 0.4
load 'uav_urban_path_exp7_bebop_simplesT192727.mat' % BAIXO 19:27       (1) Vmax = 0.4
load 'uav_urban_path_exp7_bebop_simplesT193854.mat' % ALTO 19:39        (2) Vmax = 0.2
load 'uav_urban_path_exp7_bebop_simplesT195317.mat' % ALTO ROTACIONADO 19:53	(3) Vmax = 0.15

% Simulation 1 => Vmax = 0.3
% Simulation 2 => Vmax = 0.3

load 'uav_urban_path_exp7_bebop_simplesT192727.mat'
% 'uav_up_exp5T174027.mat'

% txt(end-4) = '2';
%% Posição
% figure(1)
% subplot(131)
% plot(XX(25,:),XX(1,:),'r--','LineWidth',1)
% hold on
% plot(XX(25,:),XX(13,:),'k-','LineWidth',1.3)
% hold on
% title('Position in X','FontSize',15)
% legend('X_d','X','FontSize',15)
% xlabel('Time [s]','FontSize',15)
% ylabel('X [m]','FontSize',15)
% grid on
% 
% subplot(132)
% plot(XX(25,:),XX(2,:),'r--','LineWidth',1)
% hold on
% plot(XX(25,:),XX(14,:),'k-','LineWidth',1.3)
% hold on
% title('Position in Y','FontSize',15)
% legend('Y_d','Y','FontSize',15)
% xlabel('Time [s]','FontSize',15)
% ylabel('Y [m]','FontSize',15)
% grid on
% 
% subplot(133)
% plot(XX(1,:),XX(2,:),'r--','LineWidth',1)
% hold on
% plot(XX(13,:),XX(14,:),'k-','LineWidth',1.3)
% hold on
% title('Position em XY','FontSize',15)
% legend('XY_d','XY','FontSize',15)
% xlabel('X [m]','FontSize',15)
% ylabel('Y [m]','FontSize',15)
% grid on

%% Orientação e erro de caminho  500x700
load 'uav_up_exp5T174209.mat'
figure(1)
% subplot(211)
% plot(XX(25,:),XX(6,:),'r-','LineWidth',2)
% hold on
% plot(XX(25,:),XX(18,:),'k-','LineWidth',2.3)
% title('Orientation','FontSize',15)
% legend('$\psi_d$','$\psi$','FontSize',15,'Location','Southeast');
% xlabel('Time [s]','FontSize',15)
% yticks([-pi,-pi/2,0,pi/2,pi])
% % yticklabels({'A','B','C','D','E'})
% % yticks([0 50 100])
% 
% yticklabels({'$-\pi$';'$-\frac{\pi}{2}$';'$0$';'$\frac{\pi}{2}$';'$\pi$'})
% ylabel('Angle [rad]','FontSize',15)
% 
% grid on
% plot(XX(25,:),(XX(18,:)-XX(6,:))*(180/pi),'k-','LineWidth',1.5)

figure(311)
plot(Curva.rho,'k-','LineWidth',2.3)
hold on
plot(1:size(Curva.rho,2),...
    (Curva.Vmax).*ones(1,size(Curva.rho,2)),'r-','LineWidth',2)

ylim([0,1.3*Curva.Vmax])
xlim([0,size(XX,2)])
% plot(XX(25,:),XX(13,:)-XX(1,:),'r-','LineWidth',1.3)
% hold on
% plot(XX(25,:),XX(14,:)-XX(2,:),'g-','LineWidth',1.3)
% plot(XX(25,:),XX(15,:)-XX(3,:),'b-','LineWidth',1.3)
% ylim([-0.2,0.2])
title('Experiment 1','FontSize',15)
% xlabel('Path iteration','FontSize',15)
ylabel('Error [m]','FontSize',15)
xlabel('Path iteration [n]','FontSize',15)
legend('$\rho$','$\rho_d$','FontSize',15,'Location','Northeast');
% legend('$\rho$','$\rho_d$','FontSize',15,'Location','Northeast');
grid on

load 'uav_urban_path_exp7_bebop_simplesT192727.mat'
figure(312)
plot(Curva.rho,'k-','LineWidth',2.3)
hold on
plot(1:size(Curva.rho,2),...
    (Curva.Vmax).*ones(1,size(Curva.rho,2)),'r-','LineWidth',2)

ylim([0,1.3*Curva.Vmax])
xlim([0,size(XX,2)])
% plot(XX(25,:),XX(13,:)-XX(1,:),'r-','LineWidth',1.3)
% hold on
% plot(XX(25,:),XX(14,:)-XX(2,:),'g-','LineWidth',1.3)
% plot(XX(25,:),XX(15,:)-XX(3,:),'b-','LineWidth',1.3)
% ylim([-0.2,0.2])
title('Experiment 2','FontSize',15)
% xlabel('Path iteration [n]','FontSize',15)
ylabel('Error [m]','FontSize',15)
xlabel('Path iteration [n]','FontSize',15)
legend('$\rho$','$\rho_d$','FontSize',15,'Location','Northeast');
% legend('$\rho$','$\rho_d$','FontSize',15,'Location','Northeast');
grid on

load 'uav_urban_path_exp7_bebop_simplesT195317.mat'
figure(313)
plot(Curva.rho,'k-','LineWidth',2.3)
hold on
plot(1:size(Curva.rho,2),...
    (Curva.Vmax).*ones(1,size(Curva.rho,2)),'r-','LineWidth',2)

ylim([0,1.3*Curva.Vmax])
xlim([0,size(XX,2)])
% plot(XX(25,:),XX(13,:)-XX(1,:),'r-','LineWidth',1.3)
% hold on
% plot(XX(25,:),XX(14,:)-XX(2,:),'g-','LineWidth',1.3)
% plot(XX(25,:),XX(15,:)-XX(3,:),'b-','LineWidth',1.3)
% ylim([-0.2,0.2])
title('Experiment 3','FontSize',15)
xlabel('Path iteration [n]','FontSize',15)
ylabel('Error [m]','FontSize',15)
legend('$\rho$','$\rho_d$','FontSize',15,'Location','Northeast');
grid on


%% Posição nos planos
figure(2)
subplot(321)
plot([XX(13,1),XX(1,2:50:end)],[XX(15,1),XX(3,2:50:end)],'r--','LineWidth',2)
hold on
plot(XX(13,:),XX(15,:),'k-','LineWidth',2.3)
hold on
title('Position in XZ','FontSize',15)
% legend('Desired','Executed','FontSize',15,'Location','Southeast')
xlabel('X [m]','FontSize',15)
ylabel('Z [m]','FontSize',15)
grid on

subplot(322)
plot([XX(14,1),XX(2,2:50:end)],[XX(15,1),XX(3,2:50:end)],'r--','LineWidth',2)
hold on
plot(XX(14,:),XX(15,:),'k-','LineWidth',2.3)
hold on
title('Position in YZ','FontSize',15)
% legend('Desired','Executed','FontSize',15,'Location','Southwest')
xlabel('Y [m]','FontSize',15)
ylabel('Z [m]','FontSize',15)
grid on

subplot(3,2,3:4)
plot(XX(25,1:20:end),XX(3,1:20:end),'r--','LineWidth',2)
hold on
plot(XX(25,:),XX(15,:),'k-','LineWidth',2.3)
hold on
title('Position em Z','FontSize',15)
% legend('$Z_d$','Z','FontSize',15,'Location','Southeast')
% xlabel('Time [s]','FontSize',15)
ylabel('Z [m]','FontSize',15)
grid on

subplot(3,2,5:6)    
plot(XX(25,1:15:end),XX(6,1:15:end),'r--','LineWidth',2)
hold on
plot(XX(25,:),XX(18,:),'k-','LineWidth',2.3)
title('Orientation','FontSize',15)
% legend('$\psi_d$','$\psi$','FontSize',15,'Location','Southeast');
xlabel('Time [s]','FontSize',15)
yticks([-pi,-pi/2,0,pi/2,pi])
% yticklabels({'A','B','C','D','E'})
% yticks([0 50 100])
yticklabels({'$-\pi$';'$-\frac{\pi}{2}$';'$0$';'$\frac{\pi}{2}$';'$\pi$'})
ylabel('Angle [rad]','FontSize',15)
legend('Desired','Executed','FontSize',15,'Location','Southeast')
grid on

%% Plot no espaço
% figure(3) 
% des = plot3(CB(1:30:end,1),CB(1:30:end,2),CB(1:30:end,3),'r*','LineWidth',2);
% hold on
% des = plot3(CB(1:15:end,1),CB(1:15:end,2),CB(1:15:end,3),'r--','LineWidth',2);
% exec = plot3(XX(13,:),XX(14,:),XX(15,:),'k-','LineWidth',2.3);
% grid on
% title('Multi-passage Path','FontSize',15)
% 
% plot3(Bmb(1,1),Bmb(1,2),Bmb(1,3),'g*','Color',[0.4660    0.6740    0.1880])
% plot3(Bmb(1,1),Bmb(1,2),Bmb(1,3),'go','Color',[0.4660    0.6740    0.1880])
% 
% % EXP (3)
% R = [cosd(90),-sind(90),0;...
%      sind(90),cosd(90),0;...
%      0,0,1]; 
% Vb = Vb*R;
% Ub = Ub*R;
% 
% view(-115,35)
% u = 0:0.01:2*pi;
% % EXP (0)
% % Vb = [1,0,0; 0,0,1];
% % Ub = [0,1,0; 0,0,1];
% % 
% % XB = Bmb(2,1) +0.3150.*cos(u).*Vb(1) +0.3150*sin(u).*Ub(1);
% % YB = Bmb(2,2) +0.3150.*cos(u).*Vb(2) +0.3150*sin(u).*Ub(2);
% % ZB = Bmb(2,3) +0.3150.*cos(u).*Vb(3) +0.3150*sin(u).*Ub(3);
% % BMBO = plot3(XB,YB,ZB,'LineWidth',3,'Color',[0.4660    0.6740    0.1880]);
% % plot3(Bmb(2,1),Bmb(2,2),Bmb(2,3),'g*','Color',[0.4660    0.6740    0.1880])
% % plot3(Bmb(2,1),Bmb(2,2),Bmb(2,3),'go','Color',[0.4660    0.6740    0.1880])
% 
% 
% XB = Bmb(1,1) +0.3150.*cos(u).*Vb(1) +0.3150*sin(u).*Ub(1);
% YB = Bmb(1,2) +0.3150.*cos(u).*Vb(2) +0.3150*sin(u).*Ub(2);
% ZB = Bmb(1,3) +0.3150.*cos(u).*Vb(3) +0.3150*sin(u).*Ub(3);
% BMBO = plot3(XB,YB,ZB,'LineWidth',3,'Color',[0.4660    0.6740    0.1880]);
% 
% % legend('Desired Path','Executed Trace','FontSize',16)
% % plot3(Bmb(2,1),Bmb(2,2),Bmb(2,3),'g*')
% % plot3(Bmb(2,1),Bmb(2,2),Bmb(2,3),'go')
% 
% 
% 
% xlabel('X [m]','FontSize',15)
% ylabel('Y [m]','FontSize',15)
% zlabel('Z [m]','FontSize',15)
% axis([-1.5 1 -1.5 2 0 3])
% vert = [-1.5       -1.5     0;
%         1        -1.5     0;
%         1         2     0;
%         -1.5        2     0;
%         1        -1.5     3;
%         1         2     3;
%         -1.5       -1.5     3];
% 
% fac = [1 2 3 4;2 3 6 5;2 1 7 5];
% patch('Vertices',vert,'Faces',fac,...
%     'FaceColor',0.8*[1 1 1],'FaceAlpha',0.3,'LineWidth',1.6)
% 
% legend([des,exec,BMBO],{'Desired','Executed','Restriction'})
%% Velocidades
% figure(5)
% subplot(311)
% plot(XX(25,:),XX(7,:),'r--','LineWidth',1)
% hold on
% plot(XX(25,:),XX(19,:),'k-')
% hold on
% title('Posição em X')
% legend('dXd','dX')
% grid on
% 
% subplot(312)
% plot(XX(25,:),XX(8,:),'r--','LineWidth',1)
% hold on
% plot(XX(25,:),XX(20,:),'k-')
% hold on
% title('Velocidade em Y')
% legend('dYd','dY','FontSize',15)
% grid on
% 
% subplot(313)
% plot(XX(25,:),XX(9,:),'r--','LineWidth',1)
% hold on
% plot(XX(25,:),XX(21,:),'k-')
% hold on
% title('Velocidade em Z','FontSize',15)
% legend('dZd','dZ','FontSize',15)
% grid on

% num = abs(cross(Curva.dX(:,2:end),diff(Curva.dX,1,2)));
% denom = abs(Curva.dX(:,2:end)).^3;
% 
% k = num./denom;
% figure
% plot(num(1,:))
 
% axes('Position',[.7 .7 .2 .2])
% box on

%% Comprimento
L = 0;
for ii =1:size(XX,2)-1
    comp = norm(XX(13:15,ii+1)-XX(13:15,ii));
    L = L + comp;
end
disp(L)


%%
% 
% % 'uav_up_exp5T174209' DOIS BAMBOLES                                    (0)
% % 'uav_urban_path_exp7_bebop_simplesT192727.mat' BAIXO 19:27            (1)
% % 'uav_urban_path_exp7_bebop_simplesT193854.mat' ALTO 19:39             (2)
% % 'uav_urban_path_exp7_bebop_simplesT195317.mat' ALTO ROTACIONADO 19:53	(3)
% figure
% load 'uav_up_exp5T174209.mat'
% plot(Curva.rho/Curva.Vmax,'LineWidth',1.3)
% hold on
% 
% M = max(Curva.rho/Curva.Vmax);
% 
% load 'uav_urban_path_exp7_bebop_simplesT192727.mat'
% plot(Curva.rho/Curva.Vmax,'LineWidth',1.3)
% 
% if max(Curva.rho/Curva.Vmax)>M
%     M = max(Curva.rho/Curva.Vmax);
% end
% 
% load 'uav_urban_path_exp7_bebop_simplesT193854.mat'
% plot(Curva.rho/Curva.Vmax,'LineWidth',1.3)
% 
% if max(Curva.rho/Curva.Vmax)>M
%     M = max(Curva.rho/Curva.Vmax);
% end
% 
% load 'uav_urban_path_exp7_bebop_simplesT195317.mat'
% plot(Curva.rho/Curva.Vmax,'LineWidth',1.3)
% 
% if max(Curva.rho/Curva.Vmax)>M
%     M = max(Curva.rho/Curva.Vmax);
% end
% 
% plot(1:size(Curva.rho,2),ones(1,size(Curva.rho,2)),'r--','LineWidth',2)
% 
% title('Normalized \rho','FontSize',15)
% xlabel('Path iteration','FontSize',15)
% ylabel('Error [m]','FontSize',15)
% xlim([0,2000])
% ylim([0,M])
% grid on

%%
% h = figure(1);
% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% txt1 = ['errosEXP'];
% % ,'simSingle'];
% print(h,txt1,'-dpdf','-r0')

h = figure(313);
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
txt2 = ['ERexp3'];
print(h,txt2,'-dpdf','-r0')


