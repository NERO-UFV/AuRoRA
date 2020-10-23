%% Comandos Iniciais

clear ; close all; clc;

%% Carregando os dados

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
% %    85 -- 90        91 -- 96          97 -- 102          
% %    LF1.pPos.Qd'    LF1.pPos.Qtil'    LF1.pPos.Xd'    
% %
% %    103 -- 108      109 -- 114        115 -- 120          
% %    LF1.pPos.Qd'    LF1.pPos.Qtil'    LF1.pPos.Xd'  
% %
% %    121
% %    toc(t)  ];

% Total de experimentos
numNP = 5;
numFP = 5;
numPP = 5;
numSOLO = 5;

% -----------------------------   NO PRIORITY   ------------------------------- %

for i = 1:numNP  
type = 'NP';
filename = [type , '_' , num2str(i)];
myVars = {'data'};
AllData.NP{i} = load(filename,myVars{:});
AllData.NP{i} = AllData.NP{i}.data;
end

for i = 1:numNP
    % Tempo da simulação
    Time.NP{i}  = AllData.NP{i}(:,end);           % tempo (s)
   
    % Pioneer data
    PXd_1.NP{i}   = AllData.NP{i}(:,(1:12))';        % desired pose
    PX_1.NP{i}    = AllData.NP{i}(:,12+(1:12))';     % real pose
    PUd_1.NP{i}   = AllData.NP{i}(:,24+(1:2))';      % control signal
    PU_1.NP{i}    = AllData.NP{i}(:,26+(1:2))';      % velocidades do robô
    PXtil_1.NP{i} = PXd_1.NP{i} - PX_1.NP{i};        % erro de postura

    PXd_2.NP{i}   = AllData.NP{i}(:,28+(1:12))';       % desired pose
    PX_2.NP{i}    = AllData.NP{i}(:,28+12+(1:12))';    % real pose
    PUd_2.NP{i}   = AllData.NP{i}(:,28+24+(1:2))';     % control signal
    PU_2.NP{i}    = AllData.NP{i}(:,28+26+(1:2))';     % velocidades do robô
    PXtil_2.NP{i} = PXd_2.NP{i} - PX_2.NP{i};          % erro de postura
    
    PXd_3.NP{i}   = AllData.NP{i}(:,28+28+(1:12))';       % desired pose
    PX_3.NP{i}    = AllData.NP{i}(:,28+28+12+(1:12))';    % real pose
    PUd_3.NP{i}   = AllData.NP{i}(:,28+28+24+(1:2))';     % control signal
    PU_3.NP{i}    = AllData.NP{i}(:,28+28+26+(1:2))';     % velocidades do robô
    PXtil_3.NP{i} = PXd_3.NP{i} - PX_3.NP{i};             % erro de postura

    % Formation data
% %     X_A.NP{i}    = [AllData.NP{i}(:,(1:3)) AllData.NP{i}(:,28+(1:3))]';
% %     Xd_A.NP{i}   = AllData.NP{i}(:,96+(1:6))';
% %     Qd_A.NP{i}   = AllData.NP{i}(:,84+(1:6))';      % desired formation
% %     Q_A.NP{i}    = DirTrans_plot(X_A.NP{i});
% %     Qtil_A.NP{i} = AllData.NP{i}(:,90+(1:6))';      % formation error
% %     
% %     X_B.NP{i}    = [AllData.NP{i}(:,(1:3)) AllData.NP{i}(:,28+(1:3))]';
% %     Xd_B.NP{i}   = AllData.NP{i}(:,114+(1:6))';
% %     Qd_B.NP{i}   = AllData.NP{i}(:,102+(1:6))';      % desired formation
% %     Q_B.NP{i}    = DirTrans_plot(X_B.NP{i});
% %     Qtil_B.NP{i} = AllData.NP{i}(:,108+(1:6))';      % formation error
    
    Qd_A.NP{i}   = AllData.NP{i}(:,84+(1:6))';      % desired formation
    Qtil_A.NP{i} = AllData.NP{i}(:,90+(1:6))';      % formation error
    Xd_A.NP{i}   = AllData.NP{i}(:,96+(1:6))';
    
    Qd_B.NP{i}   = AllData.NP{i}(:,102+(1:6))';      % desired formation
    Qtil_B.NP{i} = AllData.NP{i}(:,108+(1:6))';      % formation error
    Qtil_B.NP{i}(5,:) = Qtil_B.NP{i}(5,:) - pi/2;
    Qtil_B.NP{i}(6,:) = Qtil_B.NP{i}(6,:) + pi/2;
    Xd_B.NP{i}   = AllData.NP{i}(:,114+(1:6))';
end

% -----------------------------   FORM PRIORITY   ------------------------------- %
for i = 1:numFP  
type = 'FP';
filename = [type , '_' , num2str(i)];
myVars = {'data'};
AllData.FP{i} = load(filename,myVars{:});
AllData.FP{i} = AllData.FP{i}.data;
end

for i = 1:numFP
    % Tempo da simulação
    Time.FP{i}  = AllData.FP{i}(:,end);           % tempo (s)
   
    % Pioneer data
    PXd_1.FP{i}   = AllData.FP{i}(:,(1:12))';        % desired pose
    PX_1.FP{i}    = AllData.FP{i}(:,12+(1:12))';     % real pose
    PUd_1.FP{i}   = AllData.FP{i}(:,24+(1:2))';      % control signal
    PU_1.FP{i}    = AllData.FP{i}(:,26+(1:2))';      % velocidades do robô
    PXtil_1.FP{i} = PXd_1.FP{i} - PX_1.FP{i};        % erro de postura

    PXd_2.FP{i}   = AllData.FP{i}(:,28+(1:12))';       % desired pose
    PX_2.FP{i}    = AllData.FP{i}(:,28+12+(1:12))';    % real pose
    PUd_2.FP{i}   = AllData.FP{i}(:,28+24+(1:2))';     % control signal
    PU_2.FP{i}    = AllData.FP{i}(:,28+26+(1:2))';     % velocidades do robô
    PXtil_2.FP{i} = PXd_2.FP{i} - PX_2.FP{i};          % erro de postura
    
    PXd_3.FP{i}   = AllData.FP{i}(:,28+28+(1:12))';       % desired pose
    PX_3.FP{i}    = AllData.FP{i}(:,28+28+12+(1:12))';    % real pose
    PUd_3.FP{i}   = AllData.FP{i}(:,28+28+24+(1:2))';     % control signal
    PU_3.FP{i}    = AllData.FP{i}(:,28+28+26+(1:2))';     % velocidades do robô
    PXtil_3.FP{i} = PXd_3.FP{i} - PX_3.FP{i};             % erro de postura

    % Formation data
    Qd_A.FP{i}   = AllData.FP{i}(:,84+(1:6))';      % desired formation
    Qtil_A.FP{i} = AllData.FP{i}(:,90+(1:6))';      % formation error
    Xd_A.FP{i}   = AllData.FP{i}(:,96+(1:6))';
    
    Qd_B.FP{i}   = AllData.FP{i}(:,102+(1:6))';      % desired formation
    Qtil_B.FP{i} = AllData.FP{i}(:,108+(1:6))';      % formation error
    Qtil_B.FP{i}(5,:) = Qtil_B.FP{i}(5,:) - pi/2;
    Qtil_B.FP{i}(6,:) = Qtil_B.FP{i}(6,:) + pi/2;
    Xd_B.FP{i}   = AllData.FP{i}(:,114+(1:6))';
end

% -----------------------------  POSITION PRIORITY ------------------------------- %

for i = 1:numPP  
type = 'PP';
filename = [type , '_' , num2str(i)];
myVars = {'data'};
AllData.PP{i} = load(filename,myVars{:});
AllData.PP{i} = AllData.PP{i}.data;
end

for i = 1:numPP
    % Tempo da simulação
    Time.PP{i}  = AllData.PP{i}(:,end);           % tempo (s)
   
    % Pioneer data
    PXd_1.PP{i}   = AllData.PP{i}(:,(1:12))';        % desired pose
    PX_1.PP{i}    = AllData.PP{i}(:,12+(1:12))';     % real pose
    PUd_1.PP{i}   = AllData.PP{i}(:,24+(1:2))';      % control signal
    PU_1.PP{i}    = AllData.PP{i}(:,26+(1:2))';      % velocidades do robô
    PXtil_1.PP{i} = PXd_1.PP{i} - PX_1.PP{i};        % erro de postura

    PXd_2.PP{i}   = AllData.PP{i}(:,28+(1:12))';       % desired pose
    PX_2.PP{i}    = AllData.PP{i}(:,28+12+(1:12))';    % real pose
    PUd_2.PP{i}   = AllData.PP{i}(:,28+24+(1:2))';     % control signal
    PU_2.PP{i}    = AllData.PP{i}(:,28+26+(1:2))';     % velocidades do robô
    PXtil_2.PP{i} = PXd_2.PP{i} - PX_2.PP{i};          % erro de postura
    
    PXd_3.PP{i}   = AllData.PP{i}(:,28+28+(1:12))';       % desired pose
    PX_3.PP{i}    = AllData.PP{i}(:,28+28+12+(1:12))';    % real pose
    PUd_3.PP{i}   = AllData.PP{i}(:,28+28+24+(1:2))';     % control signal
    PU_3.PP{i}    = AllData.PP{i}(:,28+28+26+(1:2))';     % velocidades do robô
    PXtil_3.PP{i} = PXd_3.PP{i} - PX_3.PP{i};             % erro de postura

    % Formation data
    Qd_A.PP{i}   = AllData.PP{i}(:,84+(1:6))';      % desired formation
    Qtil_A.PP{i} = AllData.PP{i}(:,90+(1:6))';      % formation error
    Xd_A.PP{i}   = AllData.PP{i}(:,96+(1:6))';
    
    Qd_B.PP{i}   = AllData.PP{i}(:,102+(1:6))';      % desired formation
    Qtil_B.PP{i} = AllData.PP{i}(:,108+(1:6))';      % formation error
    Qtil_B.PP{i}(5,:) = Qtil_B.PP{i}(5,:) - pi/2;
    Qtil_B.PP{i}(6,:) = Qtil_B.PP{i}(6,:) + pi/2;
    Xd_B.PP{i}   = AllData.PP{i}(:,114+(1:6))';
end

% -----------------------------  POSIÇAO  ------------------------------- %

for i = 1:numSOLO  
type = 'SOLO';
filename = [type , '_' , num2str(i)];
myVars = {'data'};
AllData.SOLO{i} = load(filename,myVars{:});
AllData.SOLO{i} = AllData.SOLO{i}.data;
end

for i = 1:numPP
    % Tempo da simulação
    Time.SOLO{i}  = AllData.SOLO{i}(:,end);           % tempo (s)
   
    % Pioneer data
    PXd_1.SOLO{i}   = AllData.SOLO{i}(:,(1:12))';        % desired pose
    PX_1.SOLO{i}    = AllData.SOLO{i}(:,12+(1:12))';     % real pose
    PUd_1.SOLO{i}   = AllData.SOLO{i}(:,24+(1:2))';      % control signal
    PU_1.SOLO{i}    = AllData.SOLO{i}(:,26+(1:2))';      % velocidades do robô
    PXtil_1.SOLO{i} = PXd_1.SOLO{i} - PX_1.SOLO{i};        % erro de postura

end

sl = 1.0;   %'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos



%% Plotar dados da mesma estratégia juntos (Forma, Posição ou Normal)
% A partir da comparação entre os resultados de cada tipo, escolher o
% melhor para em seguida comparar cada uma das estratégias de controle

% NO PRIORITY  (NP)

% Erro de Posição
figure;
subplot(221);
for i = 1:numNP
    plot(Time.NP{i} ,Qtil_A.NP{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~x}$_1$','\textit{\~x}$_2$','\textit{\~x}$_3$','\textit{\~x}$_4$','\textit{\~x}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
ylim([-0.02 0.02]);
title('(a)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(222);
for i = 1:numNP
    plot(Time.NP{i} ,Qtil_A.NP{i}(2,:),'LineWidth',sl);
    hold on;
end
lgY = legend('\textit{\~y}$_1$','\textit{\~y}$_2$','\textit{\~y}$_3$','\textit{\~y}$_4$','\textit{\~y}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
ylim([-0.02 0.02]);
title('(b)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

subplot(223);
for i = 1:numNP
    plot(Time.NP{i} ,Qtil_B.NP{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~x}$_1$','\textit{\~x}$_2$','\textit{\~x}$_3$','\textit{\~x}$_4$','\textit{\~x}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
ylim([-0.02 0.02]);
title('(c)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(224);
for i = 1:numNP
    plot(Time.NP{i} ,Qtil_B.NP{i}(2,:),'LineWidth',sl);
    hold on;
end
lgY = legend('\textit{\~y}$_1$','\textit{\~y}$_2$','\textit{\~y}$_3$','\textit{\~y}$_4$','\textit{\~y}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
ylim([-0.02 0.02]);
title('(d)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

% Erro de Forma
figure;
subplot(321);
for i = 1:numNP
    plot(Time.NP{i} ,Qtil_A.NP{i}(4,:),'LineWidth',sl);
    hold on;
end
lgRho = legend('$$\tilde{\rho_1}$$','$$\tilde{\rho_2}$$','$$\tilde{\rho_3}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
title('(a)','Interpreter','latex');
lgRho.FontSize = 10;
lgRho.Location = 'NorthEast';
lgRho.Orientation = 'horizontal';
set(lgRho,'Interpreter','latex');
grid on;

subplot(323);
for i = 1:numNP
    plot(Time.NP{i} ,180/pi*Qtil_A.NP{i}(5,:),'LineWidth',sl);
    hold on;
end
lgAlpha = legend('$$\tilde{\alpha_1}$$','$$\tilde{\alpha_2}$$','$$\tilde{\alpha_3}$$','$$\tilde{\alpha_4}$$','$$\tilde{\alpha_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
title('(b)','Interpreter','latex');
lgAlpha.FontSize = 10;
lgAlpha.Location = 'NorthEast';
lgAlpha.Orientation = 'horizontal';
set(lgAlpha,'Interpreter','latex');
grid on;

subplot(325);
for i = 1:numNP
    plot(Time.NP{i} ,180/pi*Qtil_A.NP{i}(6,:),'LineWidth',sl);
    hold on;
end
lgBeta = legend('$$\tilde{\beta_1}$$','$$\tilde{\beta_2}$$','$$\tilde{\beta_3}$$','$$\tilde{\beta_4}$$','$$\tilde{\beta_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
title('(c)','Interpreter','latex');
lgBeta.FontSize = 10;
lgBeta.Location = 'NorthEast';
lgBeta.Orientation = 'horizontal';
set(lgBeta,'Interpreter','latex');
grid on;

subplot(322);
for i = 1:numNP
    plot(Time.NP{i} ,Qtil_B.NP{i}(4,:),'LineWidth',sl);
    hold on;
end
lgRho = legend('$$\tilde{\rho_1}$$','$$\tilde{\rho_2}$$','$$\tilde{\rho_3}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
title('(d)','Interpreter','latex');
lgRho.FontSize = 10;
lgRho.Location = 'NorthEast';
lgRho.Orientation = 'horizontal';
set(lgRho,'Interpreter','latex');
grid on;

subplot(324);
for i = 1:numNP
    plot(Time.NP{i} ,180/pi*Qtil_B.NP{i}(5,:),'LineWidth',sl);
    hold on;
end
lgAlpha = legend('$$\tilde{\alpha_1}$$','$$\tilde{\alpha_2}$$','$$\tilde{\alpha_3}$$','$$\tilde{\alpha_4}$$','$$\tilde{\alpha_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
title('(e)','Interpreter','latex');
lgAlpha.FontSize = 10;
lgAlpha.Location = 'NorthEast';
lgAlpha.Orientation = 'horizontal';
set(lgAlpha,'Interpreter','latex');
grid on;

subplot(326);
for i = 1:numNP
    plot(Time.NP{i} ,180/pi*Qtil_B.NP{i}(6,:),'LineWidth',sl);
    hold on;
end
lgBeta = legend('$$\tilde{\beta_1}$$','$$\tilde{\beta_2}$$','$$\tilde{\beta_3}$$','$$\tilde{\beta_4}$$','$$\tilde{\beta_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.NP{i}(end)]);
title('(f)','Interpreter','latex');
lgBeta.FontSize = 10;
lgBeta.Location = 'NorthEast';
lgBeta.Orientation = 'horizontal';
set(lgBeta,'Interpreter','latex');
grid on;

%% POSITION PRIORITY (PP)

% Erro de Posição
figure;
subplot(221);
for i = 1:numPP
    plot(Time.PP{i} ,Qtil_A.PP{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~x}$_1$','\textit{\~x}$_2$','\textit{\~x}$_3$','\textit{\~x}$_4$','\textit{\~x}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
ylim([-0.02 0.02]);
title('(a)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(222);
for i = 1:numPP
    plot(Time.PP{i} ,Qtil_A.PP{i}(2,:),'LineWidth',sl);
    hold on;
end
lgY = legend('\textit{\~y}$_1$','\textit{\~y}$_2$','\textit{\~y}$_3$','\textit{\~y}$_4$','\textit{\~y}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
ylim([-0.02 0.02]);
title('(b)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

subplot(223);
for i = 1:numNP
    plot(Time.PP{i} ,Qtil_B.PP{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~x}$_1$','\textit{\~x}$_2$','\textit{\~x}$_3$','\textit{\~x}$_4$','\textit{\~x}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
ylim([-0.02 0.02]);
title('(c)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(224);
for i = 1:numPP
    plot(Time.PP{i} ,Qtil_B.PP{i}(2,:),'LineWidth',sl);
    hold on;
end
lgY = legend('\textit{\~y}$_1$','\textit{\~y}$_2$','\textit{\~y}$_3$','\textit{\~y}$_4$','\textit{\~y}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
ylim([-0.02 0.02]);
title('(d)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

% Erro de Forma
figure;
subplot(321);
for i = 1:numPP
    plot(Time.PP{i} ,Qtil_A.PP{i}(4,:),'LineWidth',sl);
    hold on;
end
lgRho = legend('$$\tilde{\rho_1}$$','$$\tilde{\rho_2}$$','$$\tilde{\rho_3}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
title('(a)','Interpreter','latex');
lgRho.FontSize = 10;
lgRho.Location = 'NorthEast';
lgRho.Orientation = 'horizontal';
set(lgRho,'Interpreter','latex');
grid on;

subplot(323);
for i = 1:numPP
    plot(Time.PP{i} ,180/pi*Qtil_A.PP{i}(5,:),'LineWidth',sl);
    hold on;
end
lgAlpha = legend('$$\tilde{\alpha_1}$$','$$\tilde{\alpha_2}$$','$$\tilde{\alpha_3}$$','$$\tilde{\alpha_4}$$','$$\tilde{\alpha_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
title('(b)','Interpreter','latex');
lgAlpha.FontSize = 10;
lgAlpha.Location = 'NorthEast';
lgAlpha.Orientation = 'horizontal';
set(lgAlpha,'Interpreter','latex');
grid on;

subplot(325);
for i = 1:numPP
    plot(Time.PP{i} ,180/pi*Qtil_A.PP{i}(6,:),'LineWidth',sl);
    hold on;
end
lgBeta = legend('$$\tilde{\beta_1}$$','$$\tilde{\beta_2}$$','$$\tilde{\beta_3}$$','$$\tilde{\beta_4}$$','$$\tilde{\beta_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
title('(c)','Interpreter','latex');
lgBeta.FontSize = 10;
lgBeta.Location = 'NorthEast';
lgBeta.Orientation = 'horizontal';
set(lgBeta,'Interpreter','latex');
grid on;

subplot(322);
for i = 1:numPP
    plot(Time.PP{i} ,Qtil_B.PP{i}(4,:),'LineWidth',sl);
    hold on;
end
lgRho = legend('$$\tilde{\rho_1}$$','$$\tilde{\rho_2}$$','$$\tilde{\rho_3}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
title('(d)','Interpreter','latex');
lgRho.FontSize = 10;
lgRho.Location = 'NorthEast';
lgRho.Orientation = 'horizontal';
set(lgRho,'Interpreter','latex');
grid on;

subplot(324);
for i = 1:numPP
    plot(Time.PP{i} ,180/pi*Qtil_B.PP{i}(5,:),'LineWidth',sl);
    hold on;
end
lgAlpha = legend('$$\tilde{\alpha_1}$$','$$\tilde{\alpha_2}$$','$$\tilde{\alpha_3}$$','$$\tilde{\alpha_4}$$','$$\tilde{\alpha_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
title('(e)','Interpreter','latex');
lgAlpha.FontSize = 10;
lgAlpha.Location = 'NorthEast';
lgAlpha.Orientation = 'horizontal';
set(lgAlpha,'Interpreter','latex');
grid on;

subplot(326);
for i = 1:numPP
    plot(Time.PP{i} ,180/pi*Qtil_B.PP{i}(6,:),'LineWidth',sl);
    hold on;
end
lgBeta = legend('$$\tilde{\beta_1}$$','$$\tilde{\beta_2}$$','$$\tilde{\beta_3}$$','$$\tilde{\beta_4}$$','$$\tilde{\beta_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.PP{i}(end)]);
title('(f)','Interpreter','latex');
lgBeta.FontSize = 10;
lgBeta.Location = 'NorthEast';
lgBeta.Orientation = 'horizontal';
set(lgBeta,'Interpreter','latex');
grid on;

%% FORMATION PRIORITY (FP)

% Erro de Posição
figure;
subplot(221);
for i = 1:numFP
    plot(Time.FP{i} ,Qtil_A.FP{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~x}$_1$','\textit{\~x}$_2$','\textit{\~x}$_3$','\textit{\~x}$_4$','\textit{\~x}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
ylim([-0.1 0.1]);
title('(a)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(222);
for i = 1:numFP
    plot(Time.FP{i} ,Qtil_A.FP{i}(2,:),'LineWidth',sl);
    hold on;
end
lgY = legend('\textit{\~y}$_1$','\textit{\~y}$_2$','\textit{\~y}$_3$','\textit{\~y}$_4$','\textit{\~y}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
ylim([-0.1 0.1]);
title('(b)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

subplot(223);
for i = 1:numFP
    plot(Time.FP{i} ,Qtil_B.FP{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~x}$_1$','\textit{\~x}$_2$','\textit{\~x}$_3$','\textit{\~x}$_4$','\textit{\~x}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
ylim([-0.1 0.1]);
title('(c)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(224);
for i = 1:numFP
    plot(Time.FP{i} ,Qtil_B.FP{i}(2,:),'LineWidth',sl);
    hold on;
end
lgY = legend('\textit{\~y}$_1$','\textit{\~y}$_2$','\textit{\~y}$_3$','\textit{\~y}$_4$','\textit{\~y}$_5$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
ylim([-0.1 0.1]);
title('(d)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

% Erro de Forma
figure;
subplot(321);
for i = 1:numFP
    plot(Time.FP{i} ,Qtil_A.FP{i}(4,:),'LineWidth',sl);
    hold on;
end
lgRho = legend('$$\tilde{\rho_1}$$','$$\tilde{\rho_2}$$','$$\tilde{\rho_3}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
title('(a)','Interpreter','latex');
lgRho.FontSize = 10;
lgRho.Location = 'NorthEast';
lgRho.Orientation = 'horizontal';
set(lgRho,'Interpreter','latex');
grid on;

subplot(323);
for i = 1:numFP
    plot(Time.FP{i} ,180/pi*Qtil_A.FP{i}(5,:),'LineWidth',sl);
    hold on;
end
lgAlpha = legend('$$\tilde{\alpha_1}$$','$$\tilde{\alpha_2}$$','$$\tilde{\alpha_3}$$','$$\tilde{\alpha_4}$$','$$\tilde{\alpha_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
title('(b)','Interpreter','latex');
lgAlpha.FontSize = 10;
lgAlpha.Location = 'NorthEast';
lgAlpha.Orientation = 'horizontal';
set(lgAlpha,'Interpreter','latex');
grid on;

subplot(325);
for i = 1:numFP
    plot(Time.FP{i} ,180/pi*Qtil_A.FP{i}(6,:),'LineWidth',sl);
    hold on;
end
lgBeta = legend('$$\tilde{\beta_1}$$','$$\tilde{\beta_2}$$','$$\tilde{\beta_3}$$','$$\tilde{\beta_4}$$','$$\tilde{\beta_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
title('(c)','Interpreter','latex');
lgBeta.FontSize = 10;
lgBeta.Location = 'NorthEast';
lgBeta.Orientation = 'horizontal';
set(lgBeta,'Interpreter','latex');
grid on;

subplot(322);
for i = 1:numFP
    plot(Time.FP{i} ,Qtil_B.FP{i}(4,:),'LineWidth',sl);
    hold on;
end
lgRho = legend('$$\tilde{\rho_1}$$','$$\tilde{\rho_2}$$','$$\tilde{\rho_3}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_4}$$','$$\tilde{\rho_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
title('(d)','Interpreter','latex');
lgRho.FontSize = 10;
lgRho.Location = 'NorthEast';
lgRho.Orientation = 'horizontal';
set(lgRho,'Interpreter','latex');
grid on;

subplot(324);
for i = 1:numFP
    plot(Time.FP{i} ,180/pi*Qtil_B.FP{i}(5,:),'LineWidth',sl);
    hold on;
end
lgAlpha = legend('$$\tilde{\alpha_1}$$','$$\tilde{\alpha_2}$$','$$\tilde{\alpha_3}$$','$$\tilde{\alpha_4}$$','$$\tilde{\alpha_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
title('(e)','Interpreter','latex');
lgAlpha.FontSize = 10;
lgAlpha.Location = 'NorthEast';
lgAlpha.Orientation = 'horizontal';
set(lgAlpha,'Interpreter','latex');
grid on;

subplot(326);
for i = 1:numFP
    plot(Time.FP{i} ,180/pi*Qtil_B.FP{i}(6,:),'LineWidth',sl);
    hold on;
end
lgBeta = legend('$$\tilde{\beta_1}$$','$$\tilde{\beta_2}$$','$$\tilde{\beta_3}$$','$$\tilde{\beta_4}$$','$$\tilde{\beta_5}$$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [degrees]','interpreter','Latex');
xlim([0 Time.FP{i}(end)]);
title('(f)','Interpreter','latex');
lgBeta.FontSize = 10;
lgBeta.Location = 'NorthEast';
lgBeta.Orientation = 'horizontal';
set(lgBeta,'Interpreter','latex');
grid on;

%% Plotar os melhores dados de cada estratégia para compará-las
% Plotagem final para comparação das estratégias. Plotar cada variável em
% uma janela separadada, contendo os valores dos três casos

sl = 1;   %'default';    % largura da linha
st = 12;  % tamanho da fonte
sm = 2;   % tamanho dos símbolos

% Número do experimento escolhido
numFP = 1;
numPP = 1;
numNP = 1;

% Erro de Posição
figure;
subplot(221);
plot(Time.NP{numNP} ,Qtil_A.NP{numNP}(1,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,Qtil_A.FP{numFP}(1,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl)
plot(Time.PP{numPP} ,Qtil_A.PP{numPP}(1,:),'-.b','LineWidth',sl);
ylabel('\textit{\~x}$_{F}$ [m]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-.15 .15]);
yticks([ -0.1  0  0.1 ]);
lgX = legend('$SP$','$PF$','$PP$');
lgX.FontSize = 10;
lgX.Location = 'northoutside';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(223);
plot(Time.NP{numNP} ,Qtil_A.NP{numNP}(2,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,Qtil_A.FP{numFP}(2,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl)
plot(Time.PP{numPP} ,Qtil_A.PP{numPP}(2,:),'-.b','LineWidth',sl);
xlabel('Tempo [s]','interpreter','Latex','FontSize',st);
ylabel('\textit{\~y}$_{F}$ [m]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-.15 .15]);
yticks([ -0.1  0  0.1 ]);
grid on;

subplot(222);
plot(Time.NP{numNP} ,Qtil_B.NP{numNP}(1,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,Qtil_B.FP{numFP}(1,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl)
plot(Time.PP{numPP} ,Qtil_B.PP{numPP}(1,:),'-.b','LineWidth',sl);
ylabel('\textit{\~x}$_{F}$ [m]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-.15 .15]);
yticks([ -0.1  0  0.1 ]);
lgX = legend('$SP$','$PF$','$PP$');
lgX.FontSize = 10;
lgX.Location = 'northoutside';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(224);
plot(Time.NP{numNP} ,Qtil_B.NP{numNP}(2,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,Qtil_B.FP{numFP}(2,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl)
plot(Time.PP{numPP} ,Qtil_B.PP{numPP}(2,:),'-.b','LineWidth',sl);
xlabel('Tempo [s]','interpreter','Latex','FontSize',st);
ylabel('\textit{\~y}$_{F}$ [m]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-.15 .15]);
yticks([ -0.1  0  0.1 ]);
grid on;

%% Erro de Forma

figure;
subplot(321);
plot(Time.NP{numNP} ,Qtil_A.NP{numNP}(4,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,Qtil_A.FP{numFP}(4,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl)
plot(Time.PP{numPP} ,Qtil_A.PP{numPP}(4,:),'-.b','LineWidth',sl)

ylabel('$$\tilde{\rho}$$ [m]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-.25 .25]);
yticks([ -0.2  0  0.2 ]);
lgX = legend('$SP$','$PF$','$PP$');
lgX.FontSize = 10;
lgX.Location = 'northoutside';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(323);
plot(Time.NP{numNP} ,180/pi*Qtil_A.NP{numNP}(5,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,180/pi*Qtil_A.FP{numFP}(5,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl);
plot(Time.PP{numPP} ,180/pi*Qtil_A.PP{numPP}(5,:),'-.b','LineWidth',sl);
ylabel('$$\tilde{\alpha}$$ [graus]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-20 20]);
yticks([ -15  0 15 ]);
grid on;

subplot(325);
plot(Time.NP{numNP} ,180/pi*Qtil_A.NP{numNP}(6,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,180/pi*Qtil_A.FP{numFP}(6,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl);
plot(Time.PP{numPP} ,180/pi*Qtil_A.PP{numPP}(6,:),'-.b','LineWidth',sl);
xlabel('Tempo [s]','interpreter','Latex','FontSize',st);
ylabel('$$\tilde{\beta}$$ [graus]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-20 20]);
yticks([ -15 0  15 ]);
grid on;

subplot(322);
plot(Time.NP{numNP} ,Qtil_B.NP{numNP}(4,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,Qtil_B.FP{numFP}(4,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl)
plot(Time.PP{numPP} ,Qtil_B.PP{numPP}(4,:),'-.b','LineWidth',sl)

ylabel('$$\tilde{\rho}$$ [m]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-.25 .25]);
yticks([ -0.2  0  0.2 ]);
lgX = legend('$SP$','$PF$','$PP$');
lgX.FontSize = 10;
lgX.Location = 'northoutside';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(324);
plot(Time.NP{numNP} ,180/pi*Qtil_B.NP{numNP}(5,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,180/pi*Qtil_B.FP{numFP}(5,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl);
plot(Time.PP{numPP} ,180/pi*Qtil_B.PP{numPP}(5,:),'-.b','LineWidth',sl);
ylabel('$$\tilde{\alpha}$$ [graus]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-20 20]);
yticks([ -15 0  15 ]);
grid on;

subplot(326);
plot(Time.NP{numNP} ,180/pi*Qtil_B.NP{numNP}(6,:),'-k','LineWidth',sl);
hold on;
plot(Time.FP{numFP} ,180/pi*Qtil_B.FP{numFP}(6,:),'-or','MarkerIndices',...
    1:40:length(Time.FP{numFP}),'MarkerSize',sm,'LineWidth',sl);
plot(Time.PP{numPP} ,180/pi*Qtil_B.PP{numPP}(6,:),'-.b','LineWidth',sl);
xlabel('Tempo [s]','interpreter','Latex','FontSize',st);
ylabel('$$\tilde{\beta}$$ [graus]','interpreter','Latex','FontSize',st);
xlim([0 Time.NP{numNP}(end)]);
ylim([-20 20]);
yticks([ -15 0  15 ]);
grid on;


%% IAE ---- ITAE --- IASC

numFP = 1;
numPP = 1;
numNP = 1;

%------------------------------    NP   ------------------------------%

NP_IAE_P_A = {0;0;0;0;0};
NP_IAE_F_A = {0;0;0;0;0};

NP_IAE_P_B = {0;0;0;0;0};
NP_IAE_F_B = {0;0;0;0;0};

NP_ITAE_P_A = {0;0;0;0;0};
NP_ITAE_F_A = {0;0;0;0;0};

NP_ITAE_P_B = {0;0;0;0;0};
NP_ITAE_F_B = {0;0;0;0;0};

NP_IASC_PIONEER_1 = {0;0;0;0;0};
NP_IASC_PIONEER_2 = {0;0;0;0;0};
NP_IASC_PIONEER_3 = {0;0;0;0;0};

for ii = 1:5
    for i = 1:length(Time.NP{ii})-1

        NP_IAE_P_A{ii} = NP_IAE_P_A{ii} + norm(Qtil_A.NP{ii}(1:3,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i));
        NP_IAE_F_A{ii} = NP_IAE_F_A{ii} + norm(Qtil_A.NP{ii}(4:6,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i));

        NP_IAE_P_B{ii} = NP_IAE_P_B{ii} + norm(Qtil_B.NP{ii}(1:3,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i));
        NP_IAE_F_B{ii} = NP_IAE_F_B{ii} + norm(Qtil_B.NP{ii}(4:6,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i));

        NP_ITAE_P_A{ii} = NP_ITAE_P_A{ii} + norm(Qtil_A.NP{ii}(1:3,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i))*Time.NP{ii}(i);
        NP_ITAE_F_A{ii} = NP_ITAE_F_A{ii} + norm(Qtil_A.NP{ii}(4:6,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i))*Time.NP{ii}(i);

        NP_ITAE_P_B{ii} = NP_ITAE_P_B{ii} + norm(Qtil_B.NP{ii}(1:3,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i))*Time.NP{ii}(i);
        NP_ITAE_F_B{ii} = NP_ITAE_F_B{ii} + norm(Qtil_B.NP{ii}(4:6,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i))*Time.NP{ii}(i);

        NP_IASC_PIONEER_1{ii} = NP_IASC_PIONEER_1{ii} + norm(PU_1.NP{ii}(:,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i));
        NP_IASC_PIONEER_2{ii} = NP_IASC_PIONEER_2{ii} + norm(PU_2.NP{ii}(:,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i));
        NP_IASC_PIONEER_3{ii} = NP_IASC_PIONEER_3{ii} + norm(PU_3.NP{ii}(:,i))*(Time.NP{ii}(i+1)-Time.NP{ii}(i));

    end
end
%------------------------------    FORMA   -------------------------------%

FP_IAE_P_A = {0;0;0;0;0};
FP_IAE_F_A = {0;0;0;0;0};

FP_IAE_P_B = {0;0;0;0;0};
FP_IAE_F_B = {0;0;0;0;0};

FP_ITAE_P_A = {0;0;0;0;0};
FP_ITAE_F_A = {0;0;0;0;0};

FP_ITAE_P_B = {0;0;0;0;0};
FP_ITAE_F_B = {0;0;0;0;0};

FP_IASC_PIONEER_1 = {0;0;0;0;0};
FP_IASC_PIONEER_2 = {0;0;0;0;0};
FP_IASC_PIONEER_3 = {0;0;0;0;0};

for ii = 1:5
    for i = 1:length(Time.FP{ii})-1

        FP_IAE_P_A{ii} = FP_IAE_P_A{ii} + norm(Qtil_A.FP{ii}(1:3,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i));
        FP_IAE_F_A{ii} = FP_IAE_F_A{ii} + norm(Qtil_A.FP{ii}(4:6,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i));

        FP_IAE_P_B{ii} = FP_IAE_P_B{ii} + norm(Qtil_B.FP{ii}(1:3,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i));
        FP_IAE_F_B{ii} = FP_IAE_F_B{ii} + norm(Qtil_B.FP{ii}(4:6,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i));

        FP_ITAE_P_A{ii} = FP_ITAE_P_A{ii} + norm(Qtil_A.FP{ii}(1:3,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i))*Time.FP{ii}(i);
        FP_ITAE_F_A{ii} = FP_ITAE_F_A{ii} + norm(Qtil_A.FP{ii}(4:6,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i))*Time.FP{ii}(i);

        FP_ITAE_P_B{ii} = FP_ITAE_P_B{ii} + norm(Qtil_B.FP{ii}(1:3,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i))*Time.FP{ii}(i);
        FP_ITAE_F_B{ii} = FP_ITAE_F_B{ii} + norm(Qtil_B.FP{ii}(4:6,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i))*Time.FP{ii}(i);

        FP_IASC_PIONEER_1{ii} = FP_IASC_PIONEER_1{ii} + norm(PU_1.FP{ii}(:,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i));
        FP_IASC_PIONEER_2{ii} = FP_IASC_PIONEER_2{ii} + norm(PU_2.FP{ii}(:,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i));
        FP_IASC_PIONEER_3{ii} = FP_IASC_PIONEER_3{ii} + norm(PU_3.FP{ii}(:,i))*(Time.FP{ii}(i+1)-Time.FP{ii}(i));

    end
end

%------------------------------    POSICAO   -------------------------------%

PP_IAE_P_A = {0;0;0;0;0};
PP_IAE_F_A = {0;0;0;0;0};

PP_IAE_P_B = {0;0;0;0;0};
PP_IAE_F_B = {0;0;0;0;0};

PP_ITAE_P_A = {0;0;0;0;0};
PP_ITAE_F_A = {0;0;0;0;0};

PP_ITAE_P_B = {0;0;0;0;0};
PP_ITAE_F_B = {0;0;0;0;0};

PP_IASC_PIONEER_1 = {0;0;0;0;0};
PP_IASC_PIONEER_2 = {0;0;0;0;0};
PP_IASC_PIONEER_3 = {0;0;0;0;0};

for ii = 1:5
    for i = 1:length(Time.PP{ii})-1

        PP_IAE_P_A{ii} = PP_IAE_P_A{ii} + norm(Qtil_A.PP{ii}(1:3,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i));
        PP_IAE_F_A{ii} = PP_IAE_F_A{ii} + norm(Qtil_A.PP{ii}(4:6,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i));

        PP_IAE_P_B{ii} = PP_IAE_P_B{ii} + norm(Qtil_B.PP{ii}(1:3,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i));
        PP_IAE_F_B{ii} = PP_IAE_F_B{ii} + norm(Qtil_B.PP{ii}(4:6,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i));

        PP_ITAE_P_A{ii} = PP_ITAE_P_A{ii} + norm(Qtil_A.PP{ii}(1:3,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i))*Time.PP{ii}(i);
        PP_ITAE_F_A{ii} = PP_ITAE_F_A{ii} + norm(Qtil_A.PP{ii}(4:6,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i))*Time.PP{ii}(i);

        PP_ITAE_P_B{ii} = PP_ITAE_P_B{ii} + norm(Qtil_B.PP{ii}(1:3,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i))*Time.PP{ii}(i);
        PP_ITAE_F_B{ii} = PP_ITAE_F_B{ii} + norm(Qtil_B.PP{ii}(4:6,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i))*Time.PP{ii}(i);

        PP_IASC_PIONEER_1{ii} = PP_IASC_PIONEER_1{ii} + norm(PU_1.PP{ii}(:,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i));
        PP_IASC_PIONEER_2{ii} = PP_IASC_PIONEER_2{ii} + norm(PU_2.PP{ii}(:,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i));
        PP_IASC_PIONEER_3{ii} = PP_IASC_PIONEER_3{ii} + norm(PU_3.PP{ii}(:,i))*(Time.PP{ii}(i+1)-Time.PP{ii}(i));

    end
end
