 clear; close all; clc;
load 'trajetória_2_bebop_com_carga_teste_velocidade.mat'
% Plot results
B1_Xtil = data(:,1:3) - data(:,4:6);
B2_Xtil = data(:,7:9) - data(:,10:12);
Load_Xtil = [data(:,35:37)-data(:,13:15) data(:,38)-data(:,18) data(:,39)-data(:,16)];

% Comprimento dos cabos
barL.pPar.l1 = 1.236;
barL.pPar.l2 = 1.254;

% Comprimento da barL
barL.pPar.L = 1.45;  

barL.pPos.Qd = [0 0 0 deg2rad(0) deg2rad(0) barL.pPar.L]';

% Quadrotor 1
data(:,1) = data(:,1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
data(:,2) = data(:,2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;

data(:,4) = data(:,4) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
data(:,5) = data(:,5) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;

% vel
% vel = data(2:end,1:12) - data(1:end-1,1:12)./(data(2:end,end) - data(1:end-1,end));
% vel = (data(2:end,1:12) - data(1:end-1,1:12))./0.0334;
vel = (data(2:5:end,1:12) - data(1:5:end-1,1:12))./0.0334;
%acc
% acc = (vel(2:end,1:12) - vel(1:end-1,1:12))./0.0334;
acc = (vel(2:5:end,1:12) - vel(1:5:end-1,1:12))./0.0334;

fig = figure;
left_color = [0 0 0];
right_color = [0 0 0];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
subplot(311)
% sgtitle('Q1')
hold on;
yyaxis left
axis([0 80 -1 1])
plot(data(:,end),data(:,4),'g-');
plot(data(:,end),data(:,1),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Time [s]');
ylabel('x [m]');
yyaxis right
plot(data(:,end),B1_Xtil(:,1),'r--');
axis([0 80 -.7 .7])
ylabel('Error x [m]');
legend('UAV position','UAV desired position','UAV Error','interpreter','latex','orientation','horizontal','location','northoutside');
grid on;

subplot(312)
hold on;
yyaxis left
axis([0 80 -1 1])
plot(data(:,end),data(:,5),'g-');
plot(data(:,end),data(:,2),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Time [s]');
ylabel('y [m]');
yyaxis right
plot(data(:,end),B1_Xtil(:,2),'r--');
axis([0 80 -.2 .2])
ylabel('Error y [m]');
grid on;


subplot(313)
hold on;
yyaxis left
axis([0 80 1.25 1.95])
plot(data(:,end),data(:,6),'g-');
plot(data(:,end),data(:,3),'k--');
xlabel('Time [s]');
ylabel('z [m]');
yyaxis right
plot(data(:,end),B1_Xtil(:,3),'r--');
axis([0 80 -.2 .2])
ylabel('Error z [m]');
grid on;

% Quadrotor 2
data(:,7) = data(:,7) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
data(:,8) = data(:,8) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;

data(:,10) = data(:,10) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
data(:,11) = data(:,11) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;


fig = figure;
left_color = [0 0 0];
right_color = [0 0 0];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
subplot(311)
% sgtitle('Q2')
hold on;
yyaxis left
axis([0 80 -1 1])
plot(data(:,end),data(:,10),'g-');
plot(data(:,end),data(:,7),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Time [s]');
ylabel('x [m]');
yyaxis right
plot(data(:,end),B1_Xtil(:,1),'r--');
axis([0 80 -.7 .7])
ylabel('Error x [m]');
legend('UAV position','UAV desired position','UAV Error','interpreter','latex','orientation','horizontal','location','northoutside');
grid on;

subplot(312)
hold on;
yyaxis left
axis([0 80 .45 2.45])
plot(data(:,end),data(:,11),'g-');
plot(data(:,end),data(:,8),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Time [s]');
ylabel('y [m]');
yyaxis right
plot(data(:,end),B2_Xtil(:,2),'r--');
axis([0 80 -.2 .2])
ylabel('Error y [m]');
grid on;


subplot(313)
hold on;
yyaxis left
axis([0 80 1.25 1.95])
plot(data(:,end),data(:,12),'g-');
plot(data(:,end),data(:,9),'k--');
xlabel('Time [s]');
ylabel('z [m]');
yyaxis right
plot(data(:,end),B2_Xtil(:,3),'r--');
axis([0 80 -.2 .2])
ylabel('Error z [m]');
grid on;

%% Load 3D positioning
figure();
hold on;
grid on;
plot3(data(:,13), data(:,14), data(:,15));
plot3(data(:,35)+.05, data(:,36)+.08,data(:,37)+.12);

figure();
hold on;
grid on;
plot3(data(1200:end,4), data(1200:end,5), data(1200:end,6),'r-');
% plot3(data(1200:end,1), data(1200:end,2), data(1200:end,3),'k--');
plot3(data(1200:end,10), data(1200:end,11), data(1200:end,12),'b-');
% plot3(data(1200:end,7), data(1200:end,8), data(1200:end,9),'k--');
plot3(data(1200:1679,13)-.05, data(1200:1679,14)-.08, data(1200:1679,15),'g-','linewidth',1);
plot3(data(1680:end,13)-.05, data(1680:end,14)-.08, data(1680:end,15),'-','color',[0.4940 0.1840 0.5560],'linewidth',1);
plot3(data(1200:end,35), data(1200:end,36),data(1200:end,37)+.12,'k--');
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
legend('$\mathbf{x}_1$ position','$\mathbf{x}_2$ position','2nd cycle payload position','3rd cycle payload position','position references','interpreter','latex','orientation','horizontal','location','northoutside');
%% Plot load position tracking
figure();
% sgtitle('')
subplot(311)
% title('Load position and orientation error')
hold on;
grid on;
plot(data(:,end),data(:,13)-.05,'-','color',[100 150 200]/255,'linewidth',1);
% plot(data(:,end),data(:,14)-.08,'-','color',[170 70 100]/255,'linewidth',1);
% plot(data(:,end),data(:,15)-.12,'-','color',[220 200 120]/255,'linewidth',1);
axis([0 82 -inf inf])


% plot(data(:,end),data(:,13)-.05,'g-','linewidth',1);
plot(data(:,end),data(:,35),'k--');
% legend('$x$','$x_d$','interpreter','latex');
xlabel('Time (s)');
ylabel('X Position (m)','interpreter','latex');
% legend('$x_{bar}$','$y_{bar}$','$z_{bar}$','position reference','interpreter','latex','orientation','horizontal','location','northoutside');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,14)-.08,'-','color',[170 70 100]/255,'linewidth',1);
% plot(data(:,end),data(:,14)-.08,'g-','linewidth',1);
plot(data(:,end),data(:,36),'k--');
% legend('$y$','$y_d$','interpreter','latex');
xlabel('Time (s)');
ylabel('Y Position (m)','interpreter','latex');
axis([0 82 -inf inf])

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,15),'-','color',[220 200 120]/255,'linewidth',1);
% plot(data(:,end),data(:,15)-.12,'g-','linewidth',1);
plot(data(:,end),data(:,37)+.11,'k--');
% legend('$z$','$z_d$','interpreter','latex');
xlabel('Time (s)');
ylabel('Z Position (m)','interpreter','latex');
axis([0 82 -inf inf])

% 
% %% Plot position and orientation error
% fig = figure;
% left_color = [0 0 0];
% right_color = [1 0 0];
% set(fig,'defaultAxesColorOrder',[left_color; right_color]);
% title('Load position and orientation error')
% hold on;
% yyaxis left
% axis([0 120 -1 1])
% plot(data(:,end),Load_Xtil(:,1),'-','color',[0 0.4470 0.7410],'linewidth',1);
% plot(data(:,end),Load_Xtil(:,2),'-','color',[0.8500 0.3250 0.0980],'linewidth',1);
% plot(data(:,end),Load_Xtil(:,3),'-','color',[0.9290 0.6940 0.1250],'linewidth',1);
% % legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
% xlabel('Time [s]');
% ylabel('Error $\mathbf{x} (m)$ ','interpreter','latex');
% yyaxis right
% plot(data(1:10:end,end),rad2deg(Load_Xtil(1:10:end,4))+2.5-90','-','color',[0.4940 0.1840 0.5560],'linewidth',1);
% plot(data(1:10:end,end),rad2deg(Load_Xtil(1:10:end,5))-17,'k-','linewidth',.5);
% axis([0 120 -20 20])
% ylabel('Error $\mathbf{q}_c (^\circ)$ ','interpreter','latex');
% legend('$\tilde{x}$','$\tilde{y}$','$\tilde{z}$','$\tilde{\alpha}_F$','$\tilde{\beta}_F$','interpreter','latex','orientation','horizontal','location','northoutside');
% grid on;

%% Plot position and orientation error 2
figure();
sgtitle('')
subplot(211)
% title('Load position and orientation error')
hold on;
grid on;
% plot(data(:,end),Load_Xtil(:,1),'-','color',[0 0.4470 0.7410],'linewidth',1);
plot(data(:,end),Load_Xtil(:,1),'-','color',[100 150 200]/255,'linewidth',1);
% plot(data(:,end),Load_Xtil(:,2),'-','color',[0.8500 0.3250 0.0980],'linewidth',1);
plot(data(:,end),Load_Xtil(:,2),'-','color',[170 70 100]/255,'linewidth',1);
% plot(data(:,end),Load_Xtil(:,3),'-','color',[0.9290 0.6940 0.1250],'linewidth',1);
plot(data(:,end),Load_Xtil(:,3),'-','color',[220 200 120]/255,'linewidth',1);
axis([0 82 -.4 .4])
% legend('$x$','$x_d$','interpreter','latex');
xlabel('Time (s)');
ylabel('Payload position error (m)','interpreter','latex');
legend('$\tilde{x}_{bar}$','$\tilde{y}_{bar}$','$\tilde{z}_{bar}$','$\tilde{\alpha}_F$','$\tilde{\beta}_F$','interpreter','latex','orientation','horizontal','location','northoutside');

subplot(212)
hold on;
grid on;
plot(data(1:10:end,end),rad2deg(Load_Xtil(1:10:end,4))+2.5-90','-','color',[0.4940 0.1840 0.5560],'linewidth',1);
plot(data(1:10:end,end),rad2deg(Load_Xtil(1:10:end,5))-17,'k-','linewidth',1);
% legend('$y$','$y_d$','interpreter','latex');
axis([0 82 -20 20])
xlabel('Time (s)');
ylabel('Payload orientation error $(^\circ)$ ','interpreter','latex');
legend('$\tilde{\psi}_{bar}$','$\tilde{\theta}_{bar}$','interpreter','latex','orientation','horizontal','location','northoutside');


