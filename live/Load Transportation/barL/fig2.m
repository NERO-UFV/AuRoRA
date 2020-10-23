
t = linspace(0,size(data,2),size(data,1))';
 
subplot(3,1,1)
fig_x = plot(t,data(:,79),'b-',t,data(:,1),'r--',t,data(:,91),'k-',t,data(:,40),'g--');
fig_x(1).LineWidth = 1.5;
fig_x(2).LineWidth = 1;
fig_x(3).LineWidth = 1.5;
fig_x(4).LineWidth = 1;
xlabel('time (s)','FontWeight','bold')
ylabel('Load X-position (m)', 'FontWeight','bold')
% legend('$x_{L_{1}}$','$x_{desL_{1}}$','$x_{L_2}$','$x_{desL_{2}}$','FontSize',12,'Interpreter','LaTeX')
 
subplot(3,1,2)
fig_y = plot(t,data(:,80),'b-',t,data(:,2),'r--',t,data(:,92),'k-',t,data(:,41),'g--');
fig_y(1).LineWidth = 1.5;
fig_y(2).LineWidth = 1;
fig_y(3).LineWidth = 1.5;
fig_y(4).LineWidth = 1;
xlabel('time (s)','FontWeight','bold')
ylabel('Load Y-position (m)', 'FontWeight','bold')
% legend('$y_{L_1}$','$y_{desL_1}$','$y_{L_2}$','$y_{desL_2}$','FontSize',12,'Interpreter','LaTeX')
 
subplot(3,1,3)
fig_z = plot(t,data(:,81) + barL.pPar.l1,'b-',t,data(:,3),'r--',t,data(:,93) + barL.pPar.l2,'k-',t,data(:,42),'g--');
fig_z(1).LineWidth = 1.5;
fig_z(2).LineWidth = 1;
fig_z(3).LineWidth = 1.5;
fig_z(4).LineWidth = 1;
xlabel('time (s)','FontWeight','bold')
ylabel('Load Z-position (m)', 'FontWeight','bold')
% legend('$z_{L_1}$','$z_{desL_1}$','$z_{L_2}$','$z_{desL_2}$','FontSize',12,'Interpreter','LaTeX')
 
figure
t = linspace(0,size(data,2),size(data,1))';
subplot(3,1,1)
fig_e_x = plot(t,data(:,79) - data(:,1),'b-',t,data(:,91) - data(:,40),'k-',t,zeros(size(t)),'r--')
fig_e_x(1).LineWidth = 1.5;
fig_e_x(2).LineWidth = 1.5;
fig_e_x(3).LineWidth = 1.5;
xlabel('time (s)','FontWeight','bold')
ylabel('Load X-error (m)', 'FontWeight','bold')
% legend('$\tilde{x}_{L_1}$','$\tilde{x}_{L_2}$','FontSize',12,'Interpreter','LaTeX')
 
subplot(3,1,2)
fig_e_x = plot(t,data(:,80) - data(:,2),'b-',t,data(:,92) - data(:,41),'k-',t,zeros(size(t)),'r--')
fig_e_x(1).LineWidth = 1.5;
fig_e_x(2).LineWidth = 1.5;
fig_e_x(3).LineWidth = 1.5;
xlabel('time (s)','FontWeight','bold')
ylabel('Load Y-error (m)', 'FontWeight','bold')
% legend('$\tilde{y}_{L_1}$','$\tilde{y}_{L_2}$','FontSize',12,'Interpreter','LaTeX')
 
subplot(3,1,3)
fig_e_x = plot(t,data(:,81) + barL.pPar.l1 +.1 - data(:,3),'b-',t,data(:,93) + barL.pPar.l1 +.1 - data(:,42),'k-',t,zeros(size(t)),'r--')
fig_e_x(1).LineWidth = 1.5;
fig_e_x(2).LineWidth = 1.5;
fig_e_x(3).LineWidth = 1.5;
xlabel('time (s)','FontWeight','bold')
ylabel('Load Z-error (m)', 'FontWeight','bold')
% legend('$\tilde{z}_{L_1}$','$\tilde{z}_{L_2}$','FontSize',12,'Interpreter','LaTeX')
 
 
figure 
plot(t,data(:,106),t,data(:,109))