%% Plot
% Vista 3D
figure()
plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',1.0);
hold on;
plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',1.0);
plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',1.0);
plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',1.0);
plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.5);
plot3(data(2:kk-1,142),data(2:kk-1,143),data(2:kk-1,144),'LineWidth',1.5);

grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$','Load');
% lgX = legend('$X$','$X_d$','Load');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'FontSize',12,'Interpreter','latex');
title('Perspective view (3D)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(-30,30);   

%% Vista Superior (x-y)
figure()
plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
hold on;
plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);
plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Top view (XY-plane)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(0,90); 

% triangle = plot3([B{1}.pPos.X(1) B{2}.pPos.X(1) B{3}.pPos.X(1) B{1}.pPos.X(1)],...
%     [B{1}.pPos.X(2) B{2}.pPos.X(2) B{3}.pPos.X(2) B{1}.pPos.X(2)],...
%     [B{1}.pPos.X(3) B{2}.pPos.X(3) B{3}.pPos.X(3) B{1}.pPos.X(3)], '-b','LineWidth',1.5);

% Vista Lateral (x-z)
figure()
plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
hold on;
plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);
plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Side view (XZ-plane)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(0,0); 

% Vista Lateral (y-z)
figure()
plot3(data(2:kk-1,13),data(2:kk-1,14),data(2:kk-1,15),'r-','LineWidth',0.5);
hold on;
plot3(data(2:kk-1,45),data(2:kk-1,46),data(2:kk-1,47),'g-','LineWidth',0.5);
plot3(data(2:kk-1,77),data(2:kk-1,78),data(2:kk-1,79),'b-','LineWidth',0.5);
plot3(data(2:kk-1,115),data(2:kk-1,116),data(2:kk-1,117),'k-','LineWidth',0.5);
plot3(data(2:kk-1,97),data(2:kk-1,98),data(2:kk-1,99),'m--','LineWidth',1.0);
grid on;
lgX = legend('$R_{1}$','$R_{2}$','$R_{3}$','$X$','$X_d$');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Side view (YZ-plane)','FontSize',14,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',14,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',14,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',14,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(-90,0); 

% x y z
Atil = data(:,142:144) - data(:,97:99);
Atil(:,3) = Atil(:,3) + 1.25;

last = length(data)-45;
figure()
subplot(311)
plot(data(1:last,154),data(1:last,106),'LineWidth',1.5);
hold on;
plot(data(1:last,154),Atil(1:last,1),'LineWidth',1.5);
lgX = legend('$Formation$','$Load$');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');
title('Position','FontSize',15,'interpreter','Latex')
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{x}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,154),data(1:last,107),'LineWidth',1.5);
hold on;
plot(data(1:last,154),Atil(1:last,2),'LineWidth',1.5);
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{y}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,154),data(1:last,108),'LineWidth',1.5);
hold on;
plot(data(1:last,154),Atil(1:last,2),'LineWidth',1.5);
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{z}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')


% p q beta
figure()
subplot(311)
plot(data(1:last,154),data(1:last,109),'LineWidth',1.5);
title('Shape','FontSize',15,'interpreter','Latex')
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{p}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,154),data(1:last,110),'LineWidth',1.5);
axis([0 T -.2 .2])
grid on
ylabel('$\tilde{q}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,154),180/pi*data(1:last,111),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\beta}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')

% phi theta psi
figure()
subplot(311)
plot(data(1:last,154),180/pi*data(1:last,112),'LineWidth',1.5);
title('Orientation','FontSize',15,'interpreter','Latex')
axis([0 T -40 40])
grid on
ylabel('$\tilde{\phi}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,154),180/pi*data(1:last,113),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\theta}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,154),180/pi*data(1:last,114),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\psi}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')

% phi theta psi real
figure()
subplot(311)
plot(data(1:last,154),180/pi*data(1:last,121),'LineWidth',1.5);
title('Orientation','FontSize',15,'interpreter','Latex')
axis([0 T -40 40])
grid on
ylabel('$\tilde{\phi}$','FontSize',15,'interpreter','Latex')

subplot(312)
plot(data(1:last,154),180/pi*data(1:last,122),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\theta}$','FontSize',15,'interpreter','Latex')

subplot(313)
plot(data(1:last,154),180/pi*data(1:last,123),'LineWidth',1.5);
axis([0 T -40 40])
grid on
ylabel('$\tilde{\psi}$','FontSize',15,'interpreter','Latex')
xlabel('Time [s]','FontSize',13,'interpreter','Latex')