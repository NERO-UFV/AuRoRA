%% Plot results
Xtil = data(:,1:3) - data(:,7:9);
Xtil2 = data(:,29:31) - data(:,37:39);

figure(2);
hold on;
grid on;
plot(data(:,end),Xtil(:,1));
plot(data(:,end),Xtil(:,2));
plot(data(:,end),Xtil(:,3));
axis([0 60 -0.2 .2])
title('Error');
legend('X','Y','Z');
xlabel('Time[s]');
ylabel(' [m]');

figure(9);
hold on;
grid on;
plot(data(:,end),Xtil2(:,1));
plot(data(:,end),Xtil2(:,2));
plot(data(:,end),Xtil2(:,3));
axis([0 60 -0.2 .2])
title('Error');
legend('X','Y','Z');
xlabel('Time[s]');
ylabel(' [m]');

figure(3);
h1 = plot(data(:,7),data(:,8),'b-');
h2 = plot(data(:,1),data(:,2),'k--');
hold on;
grid on;
h3 = plot3(data(:,7),data(:,8), data(:,9),'g-');
h4 = plot3(data(:,1),data(:,2),data(:,3),'k--');
% title('Seguimento do Líder em 2D e 3D');
legend([h1 h3 h2],'Seguimento 2D','Seguimento 3D','Referência do Líder','interpreter','latex','orientation','horizontal','location','northoutside','fontsize',10);
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
% axis([-2 2 -2 2 0 4])



figure(4);
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,1));
plot(data(:,end),data(:,7));
xlabel('time [s]');
ylabel('x [m]');
legend('$x_{des}$', '$x$','interpreter','latex');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,2));
plot(data(:,end),data(:,8));
xlabel('time [s]');
ylabel('y [m]');
legend('$y_{des}$', '$y$','interpreter','latex');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,3));
plot(data(:,end),data(:,9));
xlabel('time [s]');
ylabel('z [m]');
legend('$z_{des}$', '$z$','interpreter','latex');

figure(5);
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,4));
plot(data(:,end),data(:,10));
xlabel('Tempo[s]');
ylabel(' velocidade [m/s]');
legend('$\dot{x}_{des}$','$\dot{x}$','interpreter','latex');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,5));
plot(data(:,end),data(:,11));
xlabel('Tempo[s]');
ylabel(' velocidade [m/s]');
legend('$\dot{y}_{des}$','$\dot{y}$','interpreter','latex');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,6));
plot(data(:,end),data(:,12));
xlabel('Tempo[s]');
ylabel(' velocidade [m/s]');
legend('$\dot{z}_{des}$','$\dot{z}$','interpreter','latex');

figure(6);
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,29));
plot(data(:,end),data(:,37));
xlabel('Tempo[s]');
ylabel(' posição [m]');
legend('$x_{des}$', '$x$','interpreter','latex');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,30));
plot(data(:,end),data(:,38));
xlabel('Tempo[s]');
ylabel(' posição [m]');
legend('$y_{des}$', '$y$','interpreter','latex');

subplot(313)
hold on;
grid on;
plot(data(:,end) +.3,data(:,31)+.3);
plot(data(:,end)+.3,data(:,39)+.3);
xlabel('Tempo[s]');
ylabel(' posição [m]');
legend('$z_{des}$', '$z$','interpreter','latex');

figure(7);
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,33));
plot(data(:,end),data(:,41));
xlabel('Tempo[s]');
ylabel(' velocidade [m/s]');
legend('$\dot{x}_{des}$','$\dot{x}$','interpreter','latex');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,34));
plot(data(:,end),data(:,42));
xlabel('Tempo[s]');
ylabel(' velocidade [m/s]');
legend('$\dot{y}_{des}$','$\dot{y}$','interpreter','latex');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,35));
plot(data(:,end),data(:,43));
xlabel('Tempo[s]');
ylabel(' velocidade [m/s]');
legend('$\dot{z}_{des}$','$\dot{z}$','interpreter','latex');

%% new figure 4
fig = figure;
left_color = [0 0 0];
right_color = [0 0 0];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
subplot(311)
% sgtitle('Q1')
hold on;
yyaxis left
axis([0 data(end,end) -2 2])
plot(data(:,end),data(:,7),'g-');
plot(data(:,end),data(:,1),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Tempo (s)');
ylabel('x (m)');
yyaxis right
plot(data(:,end),Xtil(:,1),'r--');
axis([0 data(end,end) -.2 .2])
ylabel('Erro x (m)');
legend('Posição VANT','Posição desejada VANT','Erro VANT','interpreter','latex','orientation','horizontal','location','northoutside', 'fontsize',10);
grid on;

subplot(312)
% sgtitle('Q1')
hold on;
yyaxis left
axis([0 data(end,end) -2 2])
plot(data(:,end),data(:,8),'g-');
plot(data(:,end),data(:,2),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Tempo (s)');
ylabel('y (m)');
yyaxis right
plot(data(:,end),Xtil(:,2),'r--');
axis([0 data(end,end) -.2 .2])
ylabel('Erro y (m)');
grid on;


subplot(313)
% sgtitle('Q1')
hold on;
yyaxis left
axis([0 data(end,end) 0 4])
plot(data(:,end),data(:,9),'g-');
plot(data(:,end),data(:,3),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Tempo (s)');
ylabel('z (m)');
yyaxis right
plot(data(:,end),Xtil(:,3),'r--');
axis([0 data(end,end) -2 2])
ylabel('Erro z (m)');
grid on;
%% end

%% new figure 6
fig = figure;
left_color = [0 0 0];
right_color = [0 0 0];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
subplot(311)
% sgtitle('Q1')
hold on;
yyaxis left
axis([0 data(end,end) -2 -1])
plot(data(:,end),data(:,37),'g-');
plot(data(:,end),data(:,29),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Tempo (s)');
ylabel('x (m)');
yyaxis right
plot(data(:,end),Xtil2(:,1),'r--');
axis([0 data(end,end) -.5 .5])
ylabel('Erro x (m)');
legend('Posição VANT','Posição desejada VANT','Erro VANT','interpreter','latex','orientation','horizontal','location','northoutside', 'fontsize',10);
grid on;

subplot(312)
% sgtitle('Q1')
hold on;
yyaxis left
axis([0 data(end,end) -2 2])
plot(data(:,end),data(:,38),'g-');
plot(data(:,end),data(:,30),'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Tempo (s)');
ylabel('y (m)');
yyaxis right
plot(data(:,end),Xtil2(:,2),'r--');
axis([0 data(end,end) -.5 .5])
ylabel('Erro y (m)');
grid on;


subplot(313)
% sgtitle('Q1')
hold on;
yyaxis left
axis([0 data(end,end) -2 2])
plot(data(:,end),data(:,39)+.3,'g-');
plot(data(:,end),data(:,31)+.3,'k--');
% legend('$x$','$x_d$','interpreter','latex','orientation','horizontal','location','northoutside');
xlabel('Tempo (s)');
ylabel('z (m)');
yyaxis right
plot(data(:,end),Xtil2(:,3),'r--');
axis([0 data(end,end) -.5 .5])
ylabel('Erro z (m)');
grid on;
%% end