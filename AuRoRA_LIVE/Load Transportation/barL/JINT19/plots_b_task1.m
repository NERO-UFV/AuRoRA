clear; close all; clc;
load 'trajetória_2_bebop_com_carga.mat'
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


figure();
subplot(311)
% sgtitle('Q1')
hold on;
axis([0 120 -1 1])
plot(data(:,end),data(:,4),'g-');
plot(data(:,end),data(:,1),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Time [s]');
ylabel('x [m]');
legend('UAV position','UAV desired position','interpreter','latex','orientation','horizontal','location','northoutside');
grid on;

subplot(312)
hold on;
axis([0 120 -1 1])
plot(data(:,end),data(:,5),'g-');
plot(data(:,end),data(:,2),'k--');
xlabel('Time [s]');
ylabel('y [m]');
grid on;


subplot(313)
hold on;
axis([0 120 1.25 1.95])
plot(data(:,end),data(:,6),'g-');
plot(data(:,end),data(:,3),'k--');
xlabel('Time [s]');
ylabel('z [m]');
grid on;

% Quadrotor 2
data(:,7) = data(:,7) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
data(:,8) = data(:,8) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;

data(:,10) = data(:,10) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
data(:,11) = data(:,11) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;


figure();
subplot(311)
% sgtitle('Q2')
hold on;
axis([0 120 -1 1])
plot(data(:,end),data(:,10),'g-');
plot(data(:,end),data(:,7),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Time [s]');
ylabel('x [m]');
legend('UAV position','UAV desired position','interpreter','latex','orientation','horizontal','location','northoutside');
grid on;

subplot(312)
hold on;
axis([0 120 .45 2.45])
plot(data(:,end),data(:,11),'g-');
plot(data(:,end),data(:,8),'k--');
xlabel('Time [s]');
ylabel('y [m]');
grid on;


subplot(313)
hold on;
axis([0 120 1.25 1.95])
plot(data(:,end),data(:,12),'g-');
plot(data(:,end),data(:,9),'k--');
xlabel('Time [s]');
ylabel('z [m]');
grid on;