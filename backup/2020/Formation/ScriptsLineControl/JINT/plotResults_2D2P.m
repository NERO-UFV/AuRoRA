% function plotResultsTriangularControl(data)
%% Rotina para plotar gráficos de Formacaolinha3D
% Formato em que os dados foram salvos

% %         %   1 -- 12             13 -- 24             25 -- 26              27 -- 28
% %             P{1}.pPos.Xd'       P{1}.pPos.X'         P{1}.pSC.Ud(1:2)'     P{1}.pSC.U(1:2)'
% %         
% %         %   29 -- 40            41 -- 52             53 -- 54              55 -- 56 
% %             P{2}.pPos.Xd'       P{2}.pPos.X'         P{2}.pSC.Ud(1:2)'     P{2}.pSC.U(1:2)'
% %         
% %         %   57 -- 68            69 -- 80             81 -- 84              85 -- 88
% %             A{1}.pPos.Xd'       A{1}.pPos.X'         A{1}.pSC.Ud'          A{1}.pSC.U'
% %
% %         %   89 -- 100           101 -- 112           113 -- 116            117 -- 120
% %             A{2}.pPos.Xd'       A{2}.pPos.X'         A{2}.pSC.Ud'          A{2}.pSC.U'
% %        
% %         %   121 -- 126          127 -- 132           133 -- 138            139 -- 144
% %             LF{1}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
% %         
% %         %   145 -- 150          151 -- 156           157 -- 162            163 -- 168 
% %             LF{2}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
% %        
% %         %   169 -- 174          175 -- 180           181 -- 186            187 -- 192 
% %             LF{3}.pPos.Qd'      LF{1}.pPos.Qtil'     LF{1}.pPos.Xr'        LF{1}.pPos.Xd'
% %
% %         %   193
% %         %   toc(t)  ]

%% Declara os robos e formação
% clear all ; clc;

R{1} = Pioneer3DX;
R{2} = Pioneer3DX;
R{3} = ArDrone;
R{4} = ArDrone;

nP = 2;

F{1} = LineFormationControl;
F{2} = LineFormationControl;

nF = 3;

fim = 570;
data = data(1:fim,:);

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

A.Xd{1}   = data(:,56+(1:12))';         % desired pose
A.X{1}    = data(:,56+12+(1:12))';      % real pose
A.Ud{1}   = data(:,56+24+(1:4))';       % control signal
A.U{1}    = data(:,56+28+(1:4))';       % velocidades do robô
A.Xtil{1} = A.Xd{1} - A.X{1};           % erro de postura

A.Xd{2}   = data(:,88+(1:12))';         % desired pose
A.X{2}    = data(:,88+12+(1:12))';      % real pose
A.Ud{2}   = data(:,88+24+(1:4))';       % control signal
A.U{2}    = data(:,88+28+(1:4))';       % velocidades do robô
A.Xtil{2} = A.Xd{2} - A.X{2};           % erro de postura

% Formation data
LF.Qd{1}   = data(:,120+(1:6))';        % desired formation
LF.Qtil{1} = data(:,120+6+(1:6))';      % formation position
LF.Xr{1}   = data(:,120+12+(1:6))';     % formation error
LF.Xd{1}   = data(:,120+18+(1:6))';     % robots formation desired

LF.Qd{2}   = data(:,144+(1:6))';        % desired formation
LF.Qtil{2} = data(:,144+6+(1:6))';      % formation position
LF.Xr{2}   = data(:,144+12+(1:6))';     % formation error
LF.Xd{2}   = data(:,144+18+(1:6))';     % robots formation desired
% LF.Qtil{2}(5,:) = LF.Qtil{2}(5,:) - pi/2;
% LF.Qtil{2}(6,:) = LF.Qtil{2}(6,:) + pi/2;

LF.Qd{3}   = data(:,168+(1:6))';        % desired formation
LF.Qtil{3} = data(:,168+6+(1:6))';      % formation position
LF.Xr{3}   = data(:,168+12+(1:6))';     % formation error
LF.Xd{3}   = data(:,168+18+(1:6))';     % robots formation desired

%% Angle convertion (rad2deg)
for i = 1:nP
    P.X{i}(4:6,:) = rad2deg(P.X{i}(4:6,:));
    P.Xd{i}(4:6,:) = rad2deg(P.Xd{i}(4:6,:));
    P.U{i}(2,:) = rad2deg(P.U{i}(2,:));
    P.Ud{i}(2,:) = rad2deg(P.Ud{i}(2,:));
end

for i = 1:nF
    LF.Qd{i}([5 6],:) = rad2deg(LF.Qd{i}([5 6],:));
    LF.Qtil{i}([5 6],:) = rad2deg(LF.Qtil{i}([5 6],:));
end

%% PLOTA RESULTADOS
sl = 1.0;   %'default';    % largura da linha
st = 10;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

%% Imagem inicial

R{1}.pPos.Xc([1 2 6]) = [0 0 pi/4];
R{2}.pPos.Xc([1 2 6]) = [1 0 pi/4];
R{3}.pPos.X = A.X{1}(:,10);
R{3}.pPos.X(1:3) = [0 0 1.5];
R{4}.pPos.X = A.X{2}(:,10);
R{4}.pPos.X(1:3) = [1 0 1.5];

figure;
axis equal
axis ([-1 2 -1 1 0 2])
set(gca,'Box','on')
hold on;
grid on;

% Draw robots
for ii=1:4
    try
        R{ii}.mCADdel;
    catch
    end
end

R{1}.mCADplot(0.75,'r');
R{2}.mCADplot(0.75,'m');
R{3}.mCADload;
R{3}.mCADplot;
R{3}.mCADcolor([0; 0; 1]);
R{4}.mCADload;
R{4}.mCADplot;
R{4}.mCADcolor([0; 0; 0]);
% view(60,15);
drawnow;

%% Formation Trajectory
figure;
% subplot(131);
meio = fim/2;
R{1}.pPos.Xc([1 2 6]) = [P.X{1}(1,meio) P.X{1}(2,meio) deg2rad(P.X{1}(6,meio))];
R{1}.mCADplot(0.75,'r');
hold on;
grid on;

R{3}.pPos.X = A.X{1}(:,meio);
R{3}.mCADload;
R{3}.mCADplot;
R{3}.mCADcolor([0; 0; 1]);

T1 = plot3(P.X{1}(1,1:end),P.X{1}(2,1:end),P.X{1}(3,1:end),'-r','LineWidth',sl);
T2 = plot3(A.X{1}(1,1:end),A.X{1}(2,1:end),A.X{1}(3,1:end),'-b','LineWidth',sl);
T3 = plot3(P.Xd{1}(1,1:end),P.Xd{1}(2,1:end),P.Xd{1}(3,1:end),'--k','LineWidth',sl);
plot3(A.Xd{1}(1,1:end),A.Xd{1}(2,1:end),A.Xd{1}(3,1:end),'--k','LineWidth',sl);

zlabel('$z$ [m]','FontSize',10,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-2 2 -2 2 0 2]);
view(-40,30);

lgX = legend([T1 T2 T3],{'$UGV_{1}$','$UAV_{1}$','$X_{d}$'});
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');

drawnow;

% --------------------------------------------------------------------------------
figure()
% subplot(132);
R{1}.pPos.Xc([1 2 6]) = [P.X{1}(1,meio) P.X{1}(2,meio) deg2rad(P.X{1}(6,meio))];
R{1}.mCADplot(0.75,'r');
hold on;
grid on;

R{2}.pPos.Xc([1 2 6]) = [P.X{2}(1,meio) P.X{2}(2,meio) deg2rad(P.X{2}(6,meio))];
R{2}.mCADplot(0.75,'m');

T1 = plot3(P.X{1}(1,1:end),P.X{1}(2,1:end),P.X{1}(3,1:end),'-r','LineWidth',sl);
T2 = plot3(P.X{2}(1,1:end),P.X{2}(2,1:end),P.X{2}(3,1:end),'-m','LineWidth',sl);
T3 = plot3(P.Xd{1}(1,1:end),P.Xd{1}(2,1:end),P.Xd{1}(3,1:end),'--k','LineWidth',sl);
plot3(P.Xd{2}(1,1:end),P.Xd{2}(2,1:end),0*P.Xd{2}(3,1:end),'--k','LineWidth',sl);

zlabel('$z$ [m]','FontSize',10,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1.5 2.5 -2 2 0 2]);
view(-40,30);

lgX = legend([T1 T2 T3],{'$UGV_{1}$','$UGV_{2}$','$X_{d}$'});
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'vertical';
set(lgX,'Interpreter','latex');

drawnow;

% --------------------------------------------------------------------------------
figure()
% subplot(133);
R{1}.pPos.Xc([1 2 6]) = [P.X{2}(1,meio) P.X{2}(2,meio) deg2rad(P.X{2}(6,meio))];
R{1}.mCADplot(0.75,'m');
hold on;
grid on;

R{3}.pPos.X = A.X{2}(:,meio);
R{3}.mCADload;
R{3}.mCADplot;
R{3}.mCADcolor([1; 0.25; 0]);

T1 = plot3(P.X{2}(1,1:end),P.X{2}(2,1:end),P.X{2}(3,1:end),'-m','LineWidth',sl);
T2 = plot3(A.X{2}(1,1:end),A.X{2}(2,1:end),A.X{2}(3,1:end),'-','LineWidth',sl,'Color',[1 0.25 0]);
T3 = plot3(P.Xd{2}(1,1:end),P.Xd{2}(2,1:end),0*P.Xd{2}(3,1:end),'--k','LineWidth',sl);
plot3(A.Xd{2}(1,1:end),A.Xd{2}(2,1:end),A.Xd{2}(3,1:end),'--k','LineWidth',sl);

zlabel('$z$ [m]','FontSize',10,'Interpreter','latex');
ylabel('$y$ [m]','FontSize',10,'Interpreter','latex');
xlabel('$x$ [m]','FontSize',10,'Interpreter','latex');
axis equal;
set(gca,'Box','on')
axis([-1 3 -2 2 0 2]);
view(-40,30);

lgX = legend([T1 T2 T3],{'$UGV_{2}$','$UAV_{2}$','$X_{d}$'});
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
ylim([-1 1]);
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
ylim([-1.5 1.5]);
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
for i = 1:nP
    plot(time,P.Xtil{i}(1,:),'LineWidth',sl);
    hold on;
end
plot(time,A.Xtil{1}(1,:),'LineWidth',sl);
plot(time,A.Xtil{2}(1,:),'LineWidth',sl);

lgX = legend('$UGV_{1}$','$UGV_{2}$','$UAV_{1}$','$UAV_{2}$');
ylabel('$\tilde{x}$ $[m]$','interpreter','Latex');
xlim([0 time(end)]);
ylim([-1 1]);
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(212);
for i = 1:nP
    plot(time,P.Xtil{i}(2,:),'LineWidth',sl);
    hold on;
end
plot(time,A.Xtil{1}(1,:),'LineWidth',sl);
plot(time,A.Xtil{2}(1,:),'LineWidth',sl);xlabel('Time [s]','interpreter','Latex');
ylabel('$\tilde{y}$ $[m]$','interpreter','Latex');
xlim([0 time(end)]);
ylim([-1 1]);
grid on;

%% Velocidade Robôs
figure;
subplot(211);
for i = 1:nP
    plot(time,P.U{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('$UGV_{1}$','$UGV_{2}$');
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
for i = 1:nP
    plot(time,P.U{i}(2,:),'LineWidth',sl);
    hold on;
end
xlabel('Time [s]','interpreter','Latex');
ylabel('Angular Velocity [°/s]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
grid on;

% ArDrone 1
figure;
subplot(221);
plot(time,A.Ud{1}(1,:),'--','LineWidth',sl);hold on;
plot(time,A.U{1}(1,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\phi_{d}$','$\phi_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\phi$ [degrees]','interpreter','Latex');
title('(a)','Interpreter','latex');

subplot(222);
plot(time,A.Ud{1}(2,:),'--','LineWidth',sl);hold on;
plot(time,A.U{1}(2,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\theta_{d}$','$\theta_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\theta$ [degrees]','interpreter','Latex');
title('(b)','Interpreter','latex');

subplot(223);
plot(time,A.Ud{1}(3,:),'--','LineWidth',sl);hold on;
plot(time,A.U{1}(3,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{z}$ [m/s]','interpreter','Latex');
title('(c)','Interpreter','latex');

subplot(224);
plot(time,A.Ud{1}(4,:),'--','LineWidth',sl);hold on;
plot(time,A.U{1}(4,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{\psi}$ [degrees/s]','interpreter','Latex');
title('(d)','Interpreter','latex');

% --------------------------------------------------------------------------------

% ArDrone 2
figure;
subplot(221);
plot(time,A.Ud{2}(1,:),'--','LineWidth',sl);hold on;
plot(time,A.U{2}(1,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\phi_{d}$','$\phi_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\phi$ [degrees]','interpreter','Latex');
title('(a)','Interpreter','latex');

subplot(222);
plot(time,A.Ud{2}(2,:),'--','LineWidth',sl);hold on;
plot(time,A.U{2}(2,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\theta_{d}$','$\theta_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\theta$ [degrees]','interpreter','Latex');
title('(b)','Interpreter','latex');

subplot(223);
plot(time,A.Ud{2}(3,:),'--','LineWidth',sl);hold on;
plot(time,A.U{2}(3,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{z}$ [m/s]','interpreter','Latex');
title('(c)','Interpreter','latex');

subplot(224);
plot(time,A.Ud{2}(4,:),'--','LineWidth',sl);hold on;
plot(time,A.U{2}(4,:),'LineWidth',sl);
axis([0 time(end) -1 1]);
legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{\psi}$ [degrees/s]','interpreter','Latex');
title('(d)','Interpreter','latex');

pause()
%% IAE ---- ITAE --- IASC

%------------------------------    NORMAL   ------------------------------%

NP_IAE_P = zeros(3,1);
NP_IAE_S = zeros(3,1);

NP_IASC_PIONEER1 = zeros(1,1);
NP_IASC_PIONEER2 = zeros(1,1);

NP_IASC_DRONE1 = zeros(1,1);
NP_IASC_DRONE2 = zeros(1,1);


LF.Qtil{1}(5:6,:) = deg2rad(LF.Qtil{1}(5:6,:));
LF.Qtil{2}(5:6,:) = deg2rad(LF.Qtil{2}(5:6,:));
LF.Qtil{3}(5:6,:) = deg2rad(LF.Qtil{3}(5:6,:));

for kk = 1:3
    for i = 1:length(time)-1

        NP_IAE_P(kk) = NP_IAE_P(kk) + norm(LF.Qtil{kk}(1:3,i))*(time(i+1)-time(i));
        NP_IAE_S(kk) = NP_IAE_S(kk) + norm(LF.Qtil{kk}(4:6,i))*(time(i+1)-time(i));

    end
end

NP_IAE_P = sum(NP_IAE_P);
NP_IAE_S = sum(NP_IAE_S);

% % % % P.U{1}(2,:) = deg2rad(P.U{1}(2,:));
% % % % P.U{2}(2,:) = deg2rad(P.U{2}(2,:));


for i = 1:length(time)-1

    
    NP_IASC_PIONEER1 = NP_IASC_PIONEER1 + norm(P.U{1}(:,i))*(time(i+1)-time(i));
    NP_IASC_PIONEER2 = NP_IASC_PIONEER2 + norm(P.U{2}(:,i))*(time(i+1)-time(i));

    NP_IASC_DRONE1 = NP_IASC_DRONE1 + norm(A.U{1}(:,i))*(time(i+1)-time(i));
    NP_IASC_DRONE2 = NP_IASC_DRONE2 + norm(A.U{2}(:,i))*(time(i+1)-time(i));

end



