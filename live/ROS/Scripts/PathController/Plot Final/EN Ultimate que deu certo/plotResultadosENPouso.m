clear; close all; clc;
try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

load('bebop_caminho_Curvatura_EN_SemPrioridade_05_GoPro0100_1m18_20200122T010108.mat');

%% CÁLCULOS PRELIMINARES

sl = 1.0;   %'default';    % largura da linha
s2 = 1.2;
st = 10;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos
ft = 12;

raio_a = 0.8;
raio_b = 1.2;
inc = .001;
term = 200;
s = 0:inc:term;
x = raio_a*sin(2*pi*s/100);
y = raio_b*sin(pi*s/100);
z = 0*ones(1,length(s));

Qtil_hist = Qd_hist - Q_hist;

% Conserta os valores de AlphaTil
for i = 1:size(Qtil_hist,2)
    if abs(Qtil_hist(5,i)) > pi
        if Qtil_hist(5,i) > 0
            Qtil_hist(5,i) = -2*pi + Qtil_hist(5,i);
        else
            Qtil_hist(5,i) =  2*pi + Qtil_hist(5,i);
        end
    end
end

Qtil_hist(5,:) = Qtil_hist(5,:)*180/pi;
Qtil_hist(6,:) = Qtil_hist(6,:).*180/pi;

%Conserta valor de pouso
AlturaP = 0.5;
alturaDiff = 0.05;
kPioneer = 1;
AlturaPReal = kPioneer*0.2654;
AlturaPlataforma = 0.2;

TempoPousando1 = 17;
TempoPouso = 21;
TempoDecola = 39;
TempoPousando2 = 62;
fases = [219 390];
fases_rastro = 10;

% X_P_hist = [X_P_hist X_P_hist(:,end)];
% X_B_hist = [X_B_hist X_B_hist(:,end)];
X_B_hist(3,end) = min(X_B_hist(3,:));
X_B_hist(3,:) = X_B_hist(3,:) - min(X_B_hist(3,:)) + 0.3113;
X_P_hist(3,:) = X_P_hist(3,:) - AlturaPReal - AlturaPlataforma - 0.04626;

%% Desenha Robôs
R{1} = Pioneer3DX;
R{2} = ArDrone;

inicioPlot = 1;
inicioPlot2 = fases(1) - 3;
meioPlot = fases(1);
meioPlot2 = fases(2);
FinalPlot0 = fases(2);
FinalPlot = length(X_P_hist);
PousoPlot = 315;

%% PLOTA RESULTADOS
% figure;
% plot3(X_P_hist(1,:),X_P_hist(2,:),X_P_hist(3,:),'LineWidth',sl)
% hold on
% plot3(X_B_hist(1,:),X_B_hist(2,:),X_B_hist(3,:),'LineWidth',sl)
% plot3(x,y,X_P_hist(3,1)*ones(1,length(x)),'--','LineWidth',sl)
% 
% R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,inicioPlot) X_P_hist(2,inicioPlot) X_P_hist(3,inicioPlot) X_P_hist(6,inicioPlot)];
% R{1}.mCADplot(kPioneer,'r');
% R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,inicioPlot) X_B_hist(2,inicioPlot) X_B_hist(3,inicioPlot) X_B_hist(6,inicioPlot)];
% R{2}.mCADload;
% R{2}.mCADplot;
% R{2}.mCADcolor([1; 1; 1]);
% 
% R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,PousoPlot) X_P_hist(2,PousoPlot) X_P_hist(3,PousoPlot) X_P_hist(6,PousoPlot)];
% R{1}.mCADplot(kPioneer,'r');
% R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,PousoPlot) X_B_hist(2,PousoPlot) X_B_hist(3,PousoPlot) X_B_hist(6,PousoPlot)];
% R{2}.mCADload;
% R{2}.mCADplot;
% R{2}.mCADcolor([1; 1; 1]);
% 
% R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,FinalPlot) X_P_hist(2,FinalPlot) X_P_hist(3,FinalPlot) X_P_hist(6,FinalPlot)];
% R{1}.mCADplot(kPioneer,'r');
% R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,FinalPlot) X_B_hist(2,FinalPlot) X_B_hist(3,FinalPlot) X_B_hist(6,FinalPlot)];
% R{2}.mCADload;
% R{2}.mCADplot;
% R{2}.mCADcolor([1; 1; 1]);
% 
% lgX = legend('$X_p$','$X_b$','$X_d$');
% % xlabel('Time [s]','interpreter','Latex');
% % ylabel('Displacement [m]','interpreter','Latex');
% % xlim([0 t_hist(end)]);
% % ylim(0.25*[-1.0 1.0]);
% zlim(2*[0 1.0]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% zlabel('$z$ [m]','FontSize',10,'Interpreter','latex','FontWeight','bold','FontSize',12);
% ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
% xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
% % title('X_f Error')
% grid on;
% 
% figure;
% ta1 = annotation('textarrow', [0.37 0.43], [0.97 0.94]);
% ta1.String = 'Landing';
% 
% ta2 = annotation('textarrow', [0.81 0.87], [0.97 0.94]);
% ta2.String = 'Landing';
az = 40;
el = 44;

figure;
plot3(X_P_hist(1,1:fases(1)),X_P_hist(2,1:fases(1)),X_P_hist(3,1:fases(1)),'LineWidth',sl)
hold on
plot3(X_B_hist(1,1:fases(1)),X_B_hist(2,1:fases(1)),X_B_hist(3,1:fases(1)),'LineWidth',sl)
plot3(x,y,X_P_hist(3,1)*ones(1,length(x)),'k--','LineWidth',sl)

R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,inicioPlot) X_P_hist(2,inicioPlot) X_P_hist(3,inicioPlot) X_P_hist(6,inicioPlot)];
R{1}.mCADplot(kPioneer,'r');
R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,inicioPlot) X_B_hist(2,inicioPlot) X_B_hist(3,inicioPlot) X_B_hist(6,inicioPlot)];
R{2}.mCADload;
R{2}.mCADplot;
R{2}.mCADcolor([1; 1; 1]);

R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,inicioPlot2) X_P_hist(2,inicioPlot2) X_P_hist(3,inicioPlot2) X_P_hist(6,inicioPlot2)];
R{1}.mCADplot(kPioneer,'r');
R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,inicioPlot2) X_B_hist(2,inicioPlot2) X_B_hist(3,inicioPlot2) X_B_hist(6,inicioPlot2)];
R{2}.mCADload;
R{2}.mCADplot;
R{2}.mCADcolor([1; 1; 1]);

lgX = legend('$\mathbf{x}_1$','$\mathbf{x}_2$','$\mathbf{x}_d$');
zlim(2*[0 1.0]);
xlim([-1.5 1.5])
ylim([-1.5 1.7])
lgX.FontSize = 20;
lgX.Location = 'NorthWest';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',10,'Interpreter','latex','FontWeight','bold','FontSize',12);
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
grid on;
view([az el])

figure;
plot3(X_P_hist(1,(fases(1)):fases(2)),X_P_hist(2,(fases(1)):fases(2)),X_P_hist(3,(fases(1)):fases(2)),'LineWidth',sl)
hold on
plot3(X_B_hist(1,(fases(1)):fases(2)),X_B_hist(2,(fases(1)):fases(2)),X_B_hist(3,(fases(1)):fases(2)),'LineWidth',sl)

plot3(X_P_hist(1,(fases(1)-fases_rastro):fases(1)),X_P_hist(2,(fases(1)-fases_rastro):fases(1)),X_P_hist(3,(fases(1)-fases_rastro):fases(1)),'k','LineWidth',sl)
plot3(X_B_hist(1,(fases(1)-fases_rastro):fases(1)),X_B_hist(2,(fases(1)-fases_rastro):fases(1)),X_B_hist(3,(fases(1)-fases_rastro):fases(1)),'k','LineWidth',sl)
plot3(x,y,X_P_hist(3,1)*ones(1,length(x)),'k--','LineWidth',sl)

R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,meioPlot) X_P_hist(2,meioPlot) X_P_hist(3,meioPlot) X_P_hist(6,meioPlot)];
R{1}.mCADplot(kPioneer,'r');
R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,meioPlot) X_B_hist(2,meioPlot) X_B_hist(3,meioPlot) X_B_hist(6,meioPlot)];
R{2}.mCADload;
R{2}.mCADplot;
R{2}.mCADcolor([1; 1; 1]);

R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,meioPlot2) X_P_hist(2,meioPlot2) X_P_hist(3,meioPlot2) X_P_hist(6,meioPlot2)];
R{1}.mCADplot(kPioneer,'r');
R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,meioPlot2) X_B_hist(2,meioPlot2) X_B_hist(3,meioPlot2) X_B_hist(6,meioPlot2)];
R{2}.mCADload;
R{2}.mCADplot;
R{2}.mCADcolor([1; 1; 1]);

zlim(2*[0 1.0]);
xlim([-1.5 1.5])
ylim([-1.5 1.7])
zlabel('$z$ [m]','FontSize',10,'Interpreter','latex','FontWeight','bold','FontSize',12);
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
% title('X_f Error')
grid on;
view([az el])

figure;
plot3(X_P_hist(1,fases(2):FinalPlot),X_P_hist(2,fases(2):FinalPlot),X_P_hist(3,fases(2):FinalPlot),'LineWidth',sl)
hold on
plot3(X_B_hist(1,fases(2):FinalPlot),X_B_hist(2,fases(2):FinalPlot),X_B_hist(3,fases(2):FinalPlot),'LineWidth',sl)
plot3(X_P_hist(1,(fases(2)-fases_rastro):fases(2)),X_P_hist(2,(fases(2)-fases_rastro):fases(2)),X_P_hist(3,(fases(2)-fases_rastro):fases(2)),'k','LineWidth',sl)
plot3(X_B_hist(1,(fases(2)-fases_rastro):fases(2)),X_B_hist(2,(fases(2)-fases_rastro):fases(2)),X_B_hist(3,(fases(2)-fases_rastro):fases(2)),'k','LineWidth',sl)
plot3(x,y,X_P_hist(3,1)*ones(1,length(x)),'k--','LineWidth',sl)

R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,FinalPlot0) X_P_hist(2,FinalPlot0) X_P_hist(3,FinalPlot0) X_P_hist(6,FinalPlot0)];
R{1}.mCADplot(kPioneer,'r');
R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,FinalPlot0) X_B_hist(2,FinalPlot0) X_B_hist(3,FinalPlot0) X_B_hist(6,FinalPlot0)];
R{2}.mCADload;
R{2}.mCADplot;
R{2}.mCADcolor([1; 1; 1]);

R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,FinalPlot) X_P_hist(2,FinalPlot) X_P_hist(3,FinalPlot) X_P_hist(6,FinalPlot)];
R{1}.mCADplot(kPioneer,'r');
R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,FinalPlot) X_B_hist(2,FinalPlot) X_B_hist(3,FinalPlot) X_B_hist(6,FinalPlot)];
R{2}.mCADload;
R{2}.mCADplot;
R{2}.mCADcolor([1; 1; 1]);

zlim(2*[0 1.0]);
xlim([-1.5 1.5])
ylim([-1.5 1.7])
zlabel('$z$ [m]','FontSize',10,'Interpreter','latex','FontWeight','bold','FontSize',12);
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
% title('X_f Error')
grid on;
view([az el])

%%
figure;
ta1 = annotation('textarrow', [0.17 0.21], [0.97 0.94]);
ta1.String = 'Landing';
ta1.Color = [0 0 0];

ta2 = annotation('textarrow', [0.26 0.24], [0.97 0.94]);
ta2.String = 'Landed';
ta2.Color = [0 0 1];

ta3 = annotation('textarrow', [0.36 0.33], [0.97 0.94]);
ta3.String = 'Take Off';
ta3.Color = [1 0 0];

ta4 = annotation('textarrow', [0.50 0.45], [0.97 0.94]);
ta4.String = 'Landing';
ta4.Color = [0 0 0];

ta5 = annotation('textarrow', [0.60 0.65], [0.97 0.94]);
ta5.String = '';
ta5.Color = [0 0 0];

ta6 = annotation('textarrow', [0.70 0.68], [0.97 0.94]);
ta6.String = 'Landed';
ta6.Color = [0 0 1];

ta7 = annotation('textarrow', [0.80 0.77], [0.97 0.94]);
ta7.String = 'Take Off';
ta7.Color = [1 0 0];

ta8 = annotation('textarrow', [0.91 0.89], [0.97 0.94]);
ta8.String = 'Landing';
ta8.Color = [0 0 0];

annotation(gcf,'rectangle',...
    [0.245 0.112 0.09 0.812],...
    'Color',[0.30 0.75 0.93],...
    'FaceColor',[0 0.45 0.74],...
    'FaceAlpha',0.2);

annotation(gcf,'rectangle',...
    [0.685 0.112 0.09 0.812],...
    'Color',[0.30 0.75 0.93],...
    'FaceColor',[0 0.45 0.74],...
    'FaceAlpha',0.2);

% ta5 = annotation('textarrow', [0.81 0.87], [0.97 0.94]);
% ta5.String = 'Landing';

subplot(321);
plot(t_hist,Qtil_hist(1,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([TempoPousando1 TempoPousando1],[-1000 1000],'k--','LineWidth',s2);
plot([TempoPouso TempoPouso],[-1000 1000],'b--','LineWidth',s2);
plot([TempoDecola TempoDecola],[-1000 1000],'r--','LineWidth',s2);
plot([TempoPousando2 TempoPousando2],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{X}_f$');
% xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{x}$ [m]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(0.25*[-1.0 1.0]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('X_f Error')
grid on;

% figure;
subplot(323);
plot(t_hist,Qtil_hist(2,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([TempoPousando1 TempoPousando1],[-1000 1000],'k--','LineWidth',s2);
plot([TempoPouso TempoPouso],[-1000 1000],'b--','LineWidth',s2);
plot([TempoDecola TempoDecola],[-1000 1000],'r--','LineWidth',s2);
plot([TempoPousando2 TempoPousando2],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{Y}_f$');
% xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{y}$ [m]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(0.25*[-1.0 1.0]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('Y_f Error')
grid on;

% figure;
subplot(325);
plot(t_hist,Qtil_hist(3,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([TempoPousando1 TempoPousando1],[-1000 1000],'k--','LineWidth',s2);
plot([TempoPouso TempoPouso],[-1000 1000],'b--','LineWidth',s2);
plot([TempoDecola TempoDecola],[-1000 1000],'r--','LineWidth',s2);
plot([TempoPousando2 TempoPousando2],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{Z}_f$');
xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{z}$ [m]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(0.25*[-1.0 1.0]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('Z_f Error')
grid on;

% figure;
subplot(322);
plot(t_hist,Qtil_hist(4,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([TempoPousando1 TempoPousando1],[-1000 1000],'k--','LineWidth',s2);
plot([TempoPouso TempoPouso],[-1000 1000],'b--','LineWidth',s2);
plot([TempoDecola TempoDecola],[-1000 1000],'r--','LineWidth',s2);
plot([TempoPousando2 TempoPousando2],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{\rho}_f$');
% xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{\rho}$ [m]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(0.5*[-1.0 1.0]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('\rho_f Error')
grid on;

% figure;
subplot(324);
plot(t_hist,Qtil_hist(5,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([TempoPousando1 TempoPousando1],[-1000 1000],'k--','LineWidth',s2);
plot([TempoPouso TempoPouso],[-1000 1000],'b--','LineWidth',s2);
plot([TempoDecola TempoDecola],[-1000 1000],'r--','LineWidth',s2);
plot([TempoPousando2 TempoPousando2],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{\alpha}_f$');
% xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{\alpha}$ [$^{\circ}$]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(100*[-1.0 1.0]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('\alpha_f Error')
grid on;

% figure;
subplot(326);
plot(t_hist,Qtil_hist(6,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([TempoPousando1 TempoPousando1],[-1000 1000],'k--','LineWidth',s2);
plot([TempoPouso TempoPouso],[-1000 1000],'b--','LineWidth',s2);
plot([TempoDecola TempoDecola],[-1000 1000],'r--','LineWidth',s2);
plot([TempoPousando2 TempoPousando2],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{\beta}_f$');
xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{\beta}$ [$^{\circ}$]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(30*[-1.0 1.0]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('\beta_f Error')
grid on;

%%
velLinearP = sqrt(X_P_hist(7,:).^2 + X_P_hist(8,:).^2);
velLinearP(272) = velLinearP(271);
velLinearP(273) = velLinearP(274);
figure;
subplot(211);
plot(t_hist,velLinearP(1,:),'LineWidth',sl);
hold on;
plot(t_hist,dV_P_hist(1,:),'--','LineWidth',sl);
plot(t_hist,Vd_hist(1,:),'--','LineWidth',sl);
lgX = legend('$V$','$V_{ref}$','$V_d$');
% xlabel('Time [s]','interpreter','Latex');
ylabel('Linear Velocity [m/s]','interpreter','Latex');
xlim([0 t_hist(end)]);
ylim([0.0 0.75]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
% title('Formation Velocity')
grid on;

% figure
X_P_hist(12,272) = X_P_hist(12,271);
X_P_hist(12,273) = X_P_hist(12,274);
subplot(212);
plot(t_hist,X_P_hist(12,:),'LineWidth',sl);
hold on;
plot(t_hist,dV_P_hist(2,:),'--','LineWidth',sl);
lgX = legend('$\omega$','$\omega_{ref}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Angular Velocity [rad/s]','interpreter','Latex');
xlim([0 t_hist(end)]);
ylim([-2 2]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

% velLinearD = sqrt(X_B_hist(7,:).^2 + X_B_hist(8,:).^2);
% figure;
% subplot(211);
% plot(t_hist,velLinearD(1,:),'LineWidth',sl);
% hold on;
% plot(t_hist,dV_B_hist(1,:),'--','LineWidth',sl);
% lgX = legend('$V$','$V_{ref}$');
% xlabel('Time [s]','interpreter','Latex');
% ylabel('Linear Velocity [m/s]','interpreter','Latex');
% xlim([0 t_hist(end)]);
% % ylim([-0.02 0.02]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% grid on;
%
% % figure
% subplot(212);
% plot(t_hist,X_B_hist(12,:),'LineWidth',sl);
% hold on;
% plot(t_hist,dV_B_hist(2,:),'--','LineWidth',sl);
% lgX = legend('$\omega$','$\omega_{ref}$');
% xlabel('Time [s]','interpreter','Latex');
% ylabel('Angular Velocity [rad/s]','interpreter','Latex');
% xlim([0 t_hist(end)]);
% % ylim([-0.02 0.02]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% grid on;

