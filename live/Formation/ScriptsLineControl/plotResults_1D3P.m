% function plotResultsTriangularControl(data)
%% Rotina para plotar gráficos de Formacaolinha3D
% Formato em que os dados foram salvos

% %    1 -- 12         13 -- 24          25 -- 26           27 -- 28 
% %    P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud'         P1.pSC.U'
% %           
% %    29 -- 40        41 -- 52          53 -- 54           55 -- 56    
% %    P2.pPos.Xd'     P2.pPos.X'        P2.pSC.Ud'         P2.pSC.U'
% %
% %    57 -- 68        69 -- 80          81 -- 82           83 -- 84    
% %    P3.pPos.Xd'     P3.pPos.X'        P3.pSC.Ud'         P3.pSC.U'
% %
% %    85 -- 96        97 -- 108         109 -- 112         113 -- 116    
% %    A.pPos.Xd'      A.pPos.X'         A.pSC.Ud'          A.pSC.U'
% %             
% %    113 -- 118      119 -- 124        125 -- 130         131 -- 139          
% %    LF1.pPos.Qd'    LF1.pPos.Q'       LF1.pPos.Qtil'     LF1.pPos.Xd'    
% %
% %    140 -- 145      146 -- 151        152 -- 157         158 -- 166          
% %    LF2.pPos.Qd'    LF2.pPos.Q'       LF2.pPos.Qtil'     LF2.pPos.Xd'    
% %
% %    167
% %    toc(t)  ];

%% Declara os robos e formação
% clear all ; clc;

R{1} = Pioneer3DX;
R{2} = Pioneer3DX;
R{3} = Pioneer3DX;
R{4} = ArDrone;
n = 3;

F{1} = LineFormationControl;
F{2} = LineFormationControl;
nF = 3;

data = data(1:3312,:);

%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end)';           % tempo (s)

% Robot data
P.Xd{1}   = data(:,(1:12))';            % desired pose
P.X{1}    = data(:,12+(1:12))';         % real pose
P.Ud{1}   = data(:,24+(1:2))';          % control signal
P.U{1}    = data(:,26+(1:2))';          % velocidades do robô
P.Xtil{1} = P.Xd{1} - P.X{1};           % erro de postura

P.Xd{2}   = data(:,28+(1:12))';         % desired pose
P.X{2}    = data(:,28+12+(1:12))';      % real pose
P.Ud{2}   = data(:,28+24+(1:2))';       % control signal
P.U{2}    = data(:,28+26+(1:2))';       % velocidades do robô
P.Xtil{2} = P.Xd{2} - P.X{2};           % erro de postura

P.Xd{3}   = data(:,56+(1:12))';         % desired pose
P.X{3}    = data(:,56+12+(1:12))';      % real pose
P.Ud{3}   = data(:,56+24+(1:2))';       % control signal
P.U{3}    = data(:,56+26+(1:2))';       % velocidades do robô
P.Xtil{3} = P.Xd{3} - P.X{3};           % erro de postura

A.Xd   = data(:,84+(1:12))';         % desired pose
A.X    = data(:,84+12+(1:12))';      % real pose
A.Ud   = data(:,84+24+(1:4))';       % control signal
A.U    = data(:,84+28+(1:4))';       % velocidades do robô
A.Xtil = A.Xd - A.X;           % erro de postura

% Formation data
LF.Qd{1}   = data(:,116+(1:6))';   % desired formation
LF.Q{1}    = data(:,116+6+(1:6))';   % formation position
LF.Qtil{1} = data(:,116+12+(1:6))';   % formation error
LF.Xd{1}   = data(:,116+18+(1:6))';   % robots formation desired

LF.Qd{2}   = data(:,140+(1:6))';   % desired formation
LF.Q{2}    = data(:,140+6+(1:6))';   % formation position
LF.Qtil{2} = data(:,140+12+(1:6))';   % formation error
LF.Xd{2}   = data(:,140+18+(1:6))';   % robots formation desired
LF.Qtil{2}(5,:) = LF.Qtil{2}(5,:) - pi/2;
LF.Qtil{2}(6,:) = LF.Qtil{2}(6,:) + pi/2;

LF.Qd{3}   = data(:,164+(1:6))';   % desired formation
LF.Q{3}    = data(:,164+6+(1:6))';   % formation position
LF.Qtil{3} = data(:,164+12+(1:6))';   % formation error
LF.Xd{3}   = data(:,164+18+(1:6))';   % robots formation desired

%% Angle convertion (rad2deg)
for i = 1:n
    P.X{i}(4:6,:) = rad2deg(P.X{i}(4:6,:));
    P.Xd{i}(4:6,:) = rad2deg(P.Xd{i}(4:6,:));
    P.U{i}(2,:) = rad2deg(P.U{i}(2,:));
    P.Ud{i}(2,:) = rad2deg(P.Ud{i}(2,:));
end

for i = 1:nF
    LF.Q{i}([5 6],:) = rad2deg(LF.Q{i}([5 6],:));
    LF.Qd{i}([5 6],:) = rad2deg(LF.Qd{i}([5 6],:));
    LF.Qtil{i}([5 6],:) = rad2deg(LF.Qtil{i}([5 6],:));
end

%% PLOTA RESULTADOS
sl = 1.0;   %'default';    % largura da linha
st = 10;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

%% Imagem inicial

% % % % R{1}.pPos.Xc([1 2 6]) = [0 0 pi/4];
% % % % R{2}.pPos.Xc([1 2 6]) = [1 0 pi/4];
% % % % R{3}.pPos.Xc([1 2 6]) = [-0 1 pi/4];
% % % % R{4}.pPos.X = A.X(:,10);
% % % % R{4}.pPos.X(1:3) = [0 0 1.5];
% % % % 
% % % % figure;
% % % % axis equal
% % % % axis ([-1 2 -1 2 0 2])
% % % % set(gca,'Box','on')
% % % % hold on;
% % % % grid on;
% % % % 
% % % % % Draw robots
% % % % for ii=1:4
% % % %     try
% % % %         R{ii}.mCADdel;
% % % %     catch
% % % %     end
% % % % end
% % % % 
% % % % R{1}.mCADplot(0.75,'r');
% % % % R{2}.mCADplot(0.75,'b');
% % % % R{3}.mCADplot(0.75,'g');
% % % % R{4}.mCADload;
% % % % R{4}.mCADplot;
% % % % R{4}.mCADcolor([0; 0; 0]);
% % % % % view(60,15);
% % % % drawnow;

%% UAV Trajectory
meio = 1650;

figure;
subplot(211);

R{1}.pPos.Xc([1 2 6]) = [P.X{1}(1,meio) P.X{1}(2,meio) deg2rad(P.X{1}(6,meio))];
R{1}.mCADplot(0.75,'r');
hold on;
grid on;

R{4}.pPos.X = A.X(:,meio);
R{4}.mCADload;
R{4}.mCADplot;
R{4}.mCADcolor([0; 0; 0]);

plot3(P.X{1}(1,1:meio),P.X{1}(2,1:meio),P.X{1}(3,1:meio),'-r','LineWidth',sl);
plot3(A.X(1,1:meio),A.X(2,1:meio),A.X(3,1:meio),'-b','LineWidth',sl);
plot3(P.Xd{1}(1,1:meio),P.Xd{1}(2,1:meio),P.Xd{1}(3,1:meio),'--k','LineWidth',sl);
plot3(A.Xd(1,1:meio),A.Xd(2,1:meio),A.Xd(3,1:meio),'--k','LineWidth',sl);

zlabel('$z$ [m]','FontSize',10,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(-40,30);

subplot(212);

R{1}.pPos.Xc([1 2 6]) = [P.X{1}(1,2110) P.X{1}(2,2110) deg2rad(P.X{1}(6,2110))];
R{1}.mCADplot(0.75,'r');
hold on;

R{2}.pPos.Xc([1 2 6]) = [P.X{2}(1,end) P.X{2}(2,end) deg2rad(P.X{2}(6,end))];
R{2}.mCADplot(0.75,'b');

R{4}.pPos.X = A.X(:,end);
R{4}.pPos.X(1:3) = [P.X{2}(1,end) P.X{2}(2,end) 0.3];
R{4}.mCADload;
R{4}.mCADplot;
R{4}.mCADcolor([0; 0; 0]);

T1 = plot3(P.X{1}(1,meio:2110),P.X{1}(2,meio:2110),P.X{1}(3,meio:2110),'-r','LineWidth',sl);
hold on;
grid on;
T3 = plot3(A.X(1,meio:2110),A.X(2,meio:2110),A.X(3,meio:2110),'-g','LineWidth',sl);
plot3(P.Xd{1}(1,meio:2110),P.Xd{1}(2,meio:2110),P.Xd{1}(3,meio:2110),'--k','LineWidth',sl);
plot3(A.Xd(1,meio:2110),A.Xd(2,meio:2110),A.Xd(3,meio:2110),'--k','LineWidth',sl);

zlabel('$z$ [m]','FontSize',10,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
axis equal;
set(gca,'Box','on');
axis([-1.5 1.5 -1.5 1.5 0 2]);
view(-40,30);

T2 = plot3(P.X{2}(1,2110:end),P.X{2}(2,2110:end),P.X{2}(3,2110:end),'-b','LineWidth',sl);
T4 = plot3(A.X(1,2110:end),A.X(2,2110:end),A.X(3,2110:end),'-r','LineWidth',sl);
plot3(P.Xd{2}(1,2110:end),P.Xd{2}(2,2110:end),P.Xd{2}(3,2110:end),'--k','LineWidth',sl);
T5 = plot3(A.Xd(1,2110:end),A.Xd(2,2110:end),A.Xd(3,2110:end),'--k','LineWidth',sl);

lgX = legend([T1 T2 T3 T4 T5],{'$UGV_{1}$','$UGV_{2}$','$UAV$ ( $UGV_{1}$ ON )','$UAV$ ( $UGV_{1}$ OFF )','$X_{d}$'});
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');

drawnow;

%% Erro de Posição das Formações
figure;
subplot(211);
for i = 1:nF
    plot(time,LF.Qtil{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('$F_{1}$','$F_{2}$','$F_{3}$');
ylabel('\textit{\~x}$_{F}$ [m]','interpreter','Latex');
xlim([0 time(end)]);
ylim([-.5 .5]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(212);
for i = 1:nF
    plot(time,LF.Qtil{i}(2,:),'LineWidth',sl);
    hold on;
end
xlabel('Time [s]','interpreter','Latex');
ylabel('\textit{\~y}$_{F}$ [m]','interpreter','Latex');
xlim([0 time(end)]);
ylim([-.5 .5]);
grid on;

%% Erro de Forma das Formações
figure;
subplot(311);
for i = 1:nF
    plot(time,LF.Qtil{i}(4,:),'LineWidth',sl);
    hold on;
end
lgX = legend('$F_{1}$','$F_{2}$','$F_{3}$');
ylabel('$\tilde{\rho}$ [m]','interpreter','Latex');
xlim([0 time(end)]);
ylim([-1 1]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(312);
for i = 1:nF
    plot(time,LF.Qtil{i}(5,:),'LineWidth',sl);
    hold on;
end
ylabel('$\tilde{\alpha}$ [$^{\circ}$]','interpreter','Latex');
xlim([0 time(end)]);
ylim([-40 40]);
grid on;

subplot(313);
for i = 1:nF
    plot(time,LF.Qtil{i}(6,:),'LineWidth',sl);
    hold on;
end
xlabel('Time [s]','interpreter','Latex');
ylabel('$\tilde{\beta}$ [$^{\circ}$]','interpreter','Latex');
xlim([0 time(end)]);
ylim([-40 40]);
grid on;

%% Erro de Posição dos Robôs
figure;
subplot(211);
for i = 1:n
    plot(time,P.Xtil{i}(1,:),'LineWidth',sl);
    hold on;
end
plot(time,A.Xtil(1,:),'LineWidth',sl);
lgX = legend('$UGV_{1}$','$UGV_{2}$','$UGV_{3}$','UAV');
ylabel('$\tilde{x}$ $[m]$','interpreter','Latex');
xlim([0 time(end)]);
ylim([-1 1]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(212);
for i = 1:n
    plot(time,P.Xtil{i}(2,:),'LineWidth',sl);
    hold on;
end
plot(time,A.Xtil(2,:),'LineWidth',sl);
xlabel('Time [s]','interpreter','Latex');
ylabel('$\tilde{y}$ $[m]$','interpreter','Latex');
xlim([0 time(end)]);
ylim([-1 1]);
grid on;

%% Trajetória Robôs
figure;
% subplot(311);
p1 = plot(P.X{1}(1,:),P.X{1}(2,:),'-r','LineWidth',sl);
hold on;
plot(P.Xd{1}(1,:),P.Xd{1}(2,:),'--r','LineWidth',sl);
axis equal;
axis ([-2 2 -2 2]);
grid on;
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

% subplot(312);
p2 = plot(P.X{2}(1,:),P.X{2}(2,:),'-b','LineWidth',sl);
hold on;
plot(P.Xd{2}(1,:),P.Xd{2}(2,:),'--b','LineWidth',sl);
axis equal;
axis ([-2 2 -2 2]);
grid on;
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

% subplot(313);
p3 = plot(P.X{3}(1,:),P.X{3}(2,:),'-g','LineWidth',sl);
hold on;
p4 = plot(P.Xd{3}(1,:),P.Xd{3}(2,:),'--g','LineWidth',sl);
p5 = plot(P.Xd{3}(1,1),P.Xd{3}(2,1),'--k','LineWidth',sl);

axis equal;
axis ([-2 2 -2 2]);
grid on;
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

% subplot(224);
% plot(A.X(1,:),A.X(2,:),'-b','LineWidth',sl);
% hold on;
% plot(A.Xd(1,:),A.Xd(2,:),'--r','LineWidth',sl);
% axis equal;
% axis ([-2 2 -2 2]);
% grid on;
% xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
% ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

lgX = legend([p1 p2 p3 p5],{'$UGV_{1}$','$UGV_{2}$','$UGV_{3}$', '$X_{d}$'});
% lgX = legend([p1 p2 p3],{'$UGV_{1}$','$UGV_{2}$','$UGV_{3}$'});

lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');

%% Velocidade Robôs
figure;
subplot(211);
for i = 1:n
    plot(time,P.U{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('$P_{1}$','$P_{2}$','$P_{3}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Linear Velocity [m/s]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(212);
for i = 1:n
    plot(time,P.U{i}(2,:),'LineWidth',sl);
    hold on;
end
xlabel('Time [s]','interpreter','Latex');
ylabel('Angular Velocity [°/s]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
grid on;

% ArDrone
figure;
subplot(221);
plot(time,A.Ud(1,:),'--','LineWidth',sl);hold on;
plot(time,A.U(1,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\phi_{d}$','$\phi_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\phi$ [degrees]','interpreter','Latex');
title('(a)','Interpreter','latex');

subplot(222);
plot(time,A.Ud(2,:),'--','LineWidth',sl);hold on;
plot(time,A.U(2,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\theta_{d}$','$\theta_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\theta$ [degrees]','interpreter','Latex');
title('(b)','Interpreter','latex');

subplot(223);
plot(time,A.Ud(3,:),'--','LineWidth',sl);hold on;
plot(time,A.U(3,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{z}$ [m/s]','interpreter','Latex');
title('(c)','Interpreter','latex');

subplot(224);
plot(time,A.Ud(4,:),'--','LineWidth',sl);hold on;
plot(time,A.U(4,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{\psi}$ [degrees/s]','interpreter','Latex');
title('(d)','Interpreter','latex');
