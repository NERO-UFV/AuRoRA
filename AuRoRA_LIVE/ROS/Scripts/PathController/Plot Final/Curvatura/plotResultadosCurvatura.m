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

load('bebop_caminho_SemCurvatura_045_ResultadoComparacaoLimite_GoPro_2335_43s_20200121T233533.mat');
dV_P_hist(2,236) = dV_P_hist(2,235);

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

% X_P_hist = [X_P_hist X_P_hist(:,end)];
% X_B_hist = [X_B_hist X_B_hist(:,end)];
X_B_hist(3,end) = X_P_hist(3,end) + alturaDiff - AlturaPlataforma;
X_P_hist(3,:) = X_P_hist(3,:) -  AlturaPReal - AlturaPlataforma;

%% Desenha Robôs
R{1} = Pioneer3DX;
R{2} = ArDrone;

inicioPlot = 1;
FinalPlot = length(X_P_hist);

%% PLOTA RESULTADOS
figure;
plot3(X_P_hist(1,:),X_P_hist(2,:),X_P_hist(3,:),'LineWidth',sl)
hold on
plot3(X_B_hist(1,:),X_B_hist(2,:),X_B_hist(3,:),'LineWidth',sl)
plot3(x,y,X_P_hist(3,1)*ones(1,length(x)),'k--','LineWidth',sl)

R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,inicioPlot) X_P_hist(2,inicioPlot) X_P_hist(3,inicioPlot) X_P_hist(6,inicioPlot)];
R{1}.mCADplot(kPioneer,'r');
R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,inicioPlot) X_B_hist(2,inicioPlot) X_B_hist(3,inicioPlot) X_B_hist(6,inicioPlot)];
R{2}.mCADload;
R{2}.mCADplot;
R{2}.mCADcolor([0; 0; 0]);

R{1}.pPos.Xc([1 2 3 6]) = [X_P_hist(1,FinalPlot) X_P_hist(2,FinalPlot) X_P_hist(3,FinalPlot) X_P_hist(6,FinalPlot)];
R{1}.mCADplot(kPioneer,'r');
R{2}.pPos.X([1 2 3 6]) = [X_B_hist(1,FinalPlot) X_B_hist(2,FinalPlot) X_B_hist(3,FinalPlot) X_B_hist(6,FinalPlot)];
R{2}.mCADload;
R{2}.mCADplot;
R{2}.mCADcolor([0; 0; 0]);

% lgX = legend('$X_p$','$X_b$','$X_d$');
% xlabel('Time [s]','interpreter','Latex');
% ylabel('Displacement [m]','interpreter','Latex');
% xlim([0 t_hist(end)]);
% ylim(0.25*[-1.0 1.0]);
zlim(2*[0 1.0]);
% lgX.FontSize = 10;
% lgX.Location = 'NorthEast';
% lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',10,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
% title('X_f Error')
grid on;

figure;
ta1 = annotation('textarrow', [0.35 0.41], [0.97 0.94]);
ta1.String = 'Landing';

ta2 = annotation('textarrow', [0.79 0.85], [0.97 0.94]);
ta2.String = 'Landing';

subplot(321);
plot(t_hist,Qtil_hist(1,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([30 30],[-1000 1000],'k--','LineWidth',s2);
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
plot([30 30],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{Y}_f$');
% xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{y}$ [m]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(0.25*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('Y_f Error')
grid on;

% figure;
subplot(325);
plot(t_hist,Qtil_hist(3,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([30 30],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{Z}_f$');
xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{z}$ [m]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(0.25*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('Z_f Error')
grid on;

% figure;
subplot(322);
plot(t_hist,Qtil_hist(4,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([30 30],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{\rho}_f$');
% xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{\rho}$ [m]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(0.5*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('\rho_f Error')
grid on;

% figure;
subplot(324);
plot(t_hist,Qtil_hist(5,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([30 30],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{\alpha}_f$');
% xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{\alpha}$ [$^{\circ}$]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(100*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('\alpha_f Error')
grid on;

% figure;
subplot(326);
plot(t_hist,Qtil_hist(6,:),'LineWidth',sl);
hold on;
plot(t_hist,zeros(size(t_hist)),'--','LineWidth',sl);
plot([30 30],[-1000 1000],'k--','LineWidth',s2);
% lgX = legend('$\widetilde{\beta}_f$');
xlabel('Time [s]','interpreter','Latex','FontSize',ft);
ylabel('$\tilde{\beta}$ [$^{\circ}$]','interpreter','Latex','FontSize',ft);
xlim([0 t_hist(end)]);
ylim(40*[-1.0 1.0]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
% set(lgX,'Interpreter','latex');
% title('\beta_f Error')
grid on;

velLinearP = sqrt(X_P_hist(7,:).^2 + X_P_hist(8,:).^2);
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
subplot(212);
plot(t_hist,X_P_hist(12,:),'LineWidth',sl);
hold on;
plot(t_hist,dV_P_hist(2,:),'--','LineWidth',sl);
lgX = legend('$\omega$','$\omega_{ref}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Angular Velocity [rad/s]','interpreter','Latex');
xlim([0 t_hist(end)]);
% ylim([-0.02 0.02]);
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

