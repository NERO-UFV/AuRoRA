%%
% close all; clc;

%% PLOTA RESULTADOS

% Posição dos robôs
% Parameters

sl = 1;   %'default';    % largura da linha
st = 12;  % tamanho da fonte
sm = 2;   % tamanho dos símbolos

% Número do experimento escolhido
numFP = 1;
numPP = 1;
numNP = 1;

figure;
subplot(311);
% figure;
fig1 = plot(PX_1.NP{numNP}(1,:),PX_1.NP{numNP}(2,:),'-k','LineWidth',sl);
hold on;
fig2 = plot(PX_2.NP{numNP}(1,:),PX_2.NP{numNP}(2,:),'--r','LineWidth',sl);
fig3 = plot(PX_3.NP{numNP}(1,:),PX_3.NP{numNP}(2,:),'-.b','LineWidth',sl);

% % fig4 = plot(PXd_1.NP{numNP}(1,:),PXd_1.NP{numNP}(2,:),'LineWidth',sl);
% % fig5 = plot(PXd_2.NP{numNP}(1,:),PXd_2.NP{numNP}(2,:),'LineWidth',sl);
% % fig6 = plot(PXd_3.NP{numNP}(1,:),PXd_3.NP{numNP}(2,:),'LineWidth',sl);

axis equal;
axis ([-2 2 -2 2]);
grid on;
title('SP','Interpreter','latex','FontSize',st);
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);
ps1 = plot(PX_1.NP{numNP}(1,1),PX_1.NP{numNP}(2,1),'*k','MarkerSize',10,'LineWidth',1.5);
ps2 = plot(PX_2.NP{numNP}(1,1),PX_2.NP{numNP}(2,1),'*r','MarkerSize',10,'LineWidth',1.5);
ps3 = plot(PX_3.NP{numNP}(1,1),PX_3.NP{numNP}(2,1),'*b','MarkerSize',10,'LineWidth',1.5);
lg1 = legend([fig1 fig2 fig3 ],{'Pioneer 1','Pioneer 2','Pioneer 3'});
lg1.FontSize = 11;
lg1.Location = 'northeast';
set(lg1,'Interpreter','latex');


subplot(312);
% figure;
fig1 = plot(PX_1.FP{numFP}(1,:),PX_1.FP{numFP}(2,:),'-k','LineWidth',sl);
hold on;
fig2 = plot(PX_2.FP{numFP}(1,:),PX_2.FP{numFP}(2,:),'--r','LineWidth',sl);
fig3 = plot(PX_3.FP{numFP}(1,:),PX_3.FP{numFP}(2,:),'-.b','LineWidth',sl);

% % fig4 = plot(PXd_1.FP{numFP}(1,:),PXd_1.FP{numFP}(2,:),'LineWidth',sl);
% % fig5 = plot(PXd_2.FP{numFP}(1,:),PXd_2.FP{numFP}(2,:),'LineWidth',sl);
% % fig6 = plot(PXd_3.FP{numFP}(1,:),PXd_3.FP{numFP}(2,:),'LineWidth',sl);

axis equal;
axis ([-2 2 -2 2]);
grid on;
title('PF','Interpreter','latex','FontSize',st);
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

ps1 = plot(PX_1.FP{numFP}(1,1),PX_1.FP{numFP}(2,1),'*k','MarkerSize',10,'LineWidth',1.5);
ps2 = plot(PX_2.FP{numFP}(1,1),PX_2.FP{numFP}(2,1),'*r','MarkerSize',10,'LineWidth',1.5);
ps3 = plot(PX_3.FP{numFP}(1,1),PX_3.FP{numFP}(2,1),'*b','MarkerSize',10,'LineWidth',1.5);



subplot(313);
% figure;
fig1 = plot(PX_1.PP{numPP}(1,:),PX_1.PP{numPP}(2,:),'-k','LineWidth',sl);
hold on;
fig2 = plot(PX_2.PP{numPP}(1,:),PX_2.PP{numPP}(2,:),'--r','LineWidth',sl);
fig3 = plot(PX_3.PP{numPP}(1,:),PX_3.PP{numPP}(2,:),'-.b','LineWidth',sl);

% % fig4 = plot(PXd_1.PP{numPP}(1,:),PXd_1.PP{numPP}(2,:),'LineWidth',sl);
% % fig5 = plot(PXd_2.PP{numPP}(1,:),PXd_2.PP{numPP}(2,:),'LineWidth',sl);
% % fig6 = plot(PXd_3.PP{numPP}(1,:),PXd_3.PP{numPP}(2,:),'LineWidth',sl);

axis equal;
axis ([-2 2 -2 2]);
grid on;
title('PP','Interpreter','latex','FontSize',st);
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

ps1 = plot(PX_1.PP{numPP}(1,1),PX_1.FP{numPP}(2,1),'*k','MarkerSize',10,'LineWidth',1.5);
ps2 = plot(PX_2.PP{numPP}(1,1),PX_2.FP{numPP}(2,1),'*r','MarkerSize',10,'LineWidth',1.5);
ps3 = plot(PX_3.PP{numPP}(1,1),PX_3.FP{numPP}(2,1),'*b','MarkerSize',10,'LineWidth',1.5);

drawnow;
%%

figure;
axis equal;
axis ([-2 2 -2 2 0 2]);
view(V);
ax = gca;
ax.Projection = 'perspective';
ax.Box = 'on';
hold on;
grid on;
title('FP','Interpreter','latex','FontSize',st);
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);
zlabel('$z$ [m]','interpreter','Latex','FontSize',st);

% plot robots and formation lines
for k = 1:step:length(Time.Forma{numF})
    
    % plot formation line
    x = [PX.Forma{numF}(1,k)         AX.Forma{numF}(1,k)];
    y = [PX.Forma{numF}(2,k)         AX.Forma{numF}(2,k)];
    z = [PX.Forma{numF}(3,k) + h     AX.Forma{numF}(3,k)];
    
    pl = line(x,y,z);
    pl.Color = 'k';
    pl.LineStyle = '-.';
    pl.LineWidth = sl;

end

fig1 = plot3(PX.Forma{numF}(1,:),PX.Forma{numF}(2,:),PX.Forma{numF}(3,:)+h,'-r','LineWidth',sl); hold on;
fig2 = plot3(AX.Forma{numF}(1,1:stepTraj:end),AX.Forma{numF}(2,1:stepTraj:end),AX.Forma{numF}(3,1:stepTraj:end),'-b','LineWidth',sl);

% Initial Posicaos
ps1 = plot3(PX.Forma{numF}(1,1),PX.Forma{numF}(2,1),PX.Forma{numF}(3,1)+h,'*m','MarkerSize',10,'LineWidth',1.5);
ps2 = plot3(AX.Forma{numF}(1,1),AX.Forma{numF}(2,1),AX.Forma{numF}(3,1),'*m','MarkerSize',10,'LineWidth',1.5);

%%

figure;
axis equal;
axis ([-2 2 -2 2 0 2]);
view(V);
ax = gca;
ax.Projection = 'perspective';
ax.Box = 'on';
hold on;
grid on;
title('PP','Interpreter','latex','FontSize',st);
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);
zlabel('$z$ [m]','interpreter','Latex','FontSize',st);

% plot robots and formation lines
for k = 1:step:length(Time.Posicao{numP})
    
    % plot formation line
    x = [PX.Posicao{numP}(1,k)         AX.Posicao{numP}(1,k)];
    y = [PX.Posicao{numP}(2,k)         AX.Posicao{numP}(2,k)];
    z = [PX.Posicao{numP}(3,k) + h     AX.Posicao{numP}(3,k)];
    
    pl = line(x,y,z);
    pl.Color = 'k';
    pl.LineStyle = '-.';
    pl.LineWidth = sl;

end

fig1 = plot3(PX.Posicao{numP}(1,:),PX.Posicao{numP}(2,:),PX.Posicao{numP}(3,:)+h,'-r','LineWidth',sl); hold on;
fig2 = plot3(AX.Posicao{numP}(1,1:stepTraj:end),AX.Posicao{numP}(2,1:stepTraj:end),AX.Posicao{numP}(3,1:stepTraj:end),'-b','LineWidth',sl);

% Initial Posicaos
ps1 = plot3(PX.Posicao{numP}(1,1),PX.Posicao{numP}(2,1),PX.Posicao{numP}(3,1)+h,'*m','MarkerSize',10,'LineWidth',1.5);
ps2 = plot3(AX.Posicao{numP}(1,1),AX.Posicao{numP}(2,1),AX.Posicao{numP}(3,1),'*m','MarkerSize',10,'LineWidth',1.5);


