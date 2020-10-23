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
% %    85 -- 96        97 -- 108         109 -- 110         111 -- 112    
% %    P4.pPos.Xd'     P4.pPos.X'        P4.pSC.Ud'         P4.pSC.U'
% %             
% %    113 -- 118      119 -- 124        125 -- 130         131 -- 139          
% %    TF1.pPos.Qd'    TF1.pPos.Q'       TF1.pPos.Qtil'     TF1.pPos.Xd'    
% %
% %    140 -- 145      146 -- 151        152 -- 157         158 -- 166          
% %    TF2.pPos.Qd'    TF2.pPos.Q'       TF2.pPos.Qtil'     TF2.pPos.Xd'    
% %
% %    167
% %    toc(t)  ];

%% Declara os robos e formação
R{1} = Pioneer3DX;
R{2} = Pioneer3DX;
R{3} = Pioneer3DX;
R{4} = Pioneer3DX;

n = 3;

F{1} = TriangularFormationControl;
F{2} = TriangularFormationControl;

nF = 1;

%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end);           % tempo (s)

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

% % P.Xd{4}   = data(:,84+(1:12))';         % desired pose
% % P.X{4}    = data(:,84+12+(1:12))';      % real pose
% % P.Ud{4}   = data(:,84+24+(1:2))';       % control signal
% % P.U{4}    = data(:,84+26+(1:2))';       % velocidades do robô
% % P.Xtil{4} = P.Xd{4} - P.X{4};           % erro de postura

% Formation data
TF.Qd{1}   = data(:,84+(1:6))';   % desired formation
TF.Q{1}    = data(:,90+(1:6))';   % formation position
TF.Qtil{1} = data(:,96+(1:6))';   % formation error
TF.Xd{1}   = data(:,102+(1:9))';   % robots formation desired

% % TF.Qd{1}   = data(:,112+(1:6))';   % desired formation
% % TF.Q{1}    = data(:,118+(1:6))';   % formation position
% % TF.Qtil{1} = data(:,124+(1:6))';   % formation error
% % TF.Xd{1}   = data(:,130+(1:9))';   % robots formation desired
% % 
% % TF.Qd{2}   = data(:,139+(1:6))';   % desired formation
% % TF.Q{2}    = data(:,145+(1:6))';   % formation position
% % TF.Qtil{2} = data(:,151+(1:6))';   % formation error
% % TF.Xd{2}   = data(:,157+(1:9))';   % robots formation desired

%% Angle convertion (rad2deg)
for i = 1:n
    P.X{i}(4:6,:) = rad2deg(P.X{i}(4:6,:));
    P.Xd{i}(4:6,:) = rad2deg(P.Xd{i}(4:6,:));
    P.U{i}(2,:) = rad2deg(P.U{i}(2,:));
    P.Ud{i}(2,:) = rad2deg(P.Ud{i}(2,:));
end

for i = 1:nF
    TF.Q{i}([3 6],:) = rad2deg(TF.Q{i}([3 6],:));
    TF.Qd{i}([3 6],:) = rad2deg(TF.Qd{i}([3 6],:));
    TF.Qtil{i}([3 6],:) = rad2deg(TF.Qtil{i}([3 6],:));
end

%% PLOTA RESULTADOS
sl = 1.0;   %'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

% Erro de Posição dos Robôs
figure;
subplot(211);
for i = 1:n
    plot(time,P.Xtil{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~x}$_1$','\textit{\~x}$_2$','\textit{\~x}$_3$','\textit{\~x}$_4$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(a)','Interpreter','latex');
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
lgY = legend('\textit{\~y}$_1$','\textit{\~y}$_2$','\textit{\~y}$_3$','\textit{\~y}$_4$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(b)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

%% Erro de Posição das Formações
figure;
subplot(211);
for i = 1:nF
    plot(time,TF.Qtil{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~x}$_{F1}$','\textit{\~x}$_{F2}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(a)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(212);
for i = 1:nF
    plot(time,TF.Qtil{i}(2,:),'LineWidth',sl);
    hold on;
end
lgY = legend('\textit{\~y}$_{F1}$','\textit{\~y}$_{F2}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Error [m]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(b)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

%% Erro de Forma das Formações
figure;
subplot(221);
for i = 1:nF
    plot(time,TF.Qtil{i}(4,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~p}$_{F1}$','\textit{\~p}$_{F2}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('p Error [m]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(a)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(222);
for i = 1:nF
    plot(time,TF.Qtil{i}(5,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{\~q}$_{F1}$','\textit{\~q}$_{F2}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('q Error [m]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(b)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(223);
for i = 1:nF
    plot(time,TF.Qtil{i}(3,:),'LineWidth',sl);
    hold on;
end
lgX = legend('$\tilde{\psi}_{F1}$','$\tilde{\psi}_{F2}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('\psi Error [°]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(c)','Interpreter','latex');
lgX.FontSize = 10;
lgX.Location = 'NorthEast';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(224);
for i = 1:nF
    plot(time,TF.Qtil{i}(6,:),'LineWidth',sl);
    hold on;
end
lgY = legend('$\tilde{\beta}_{F1}$','$\tilde{\beta}_{F2}$');
xlabel('Time [s]','interpreter','Latex');
ylabel('\beta Error [°]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(d)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

%% Trajetória Robôs
figure;
subplot(221);
plot(P.X{1}(1,:),P.X{1}(2,:),'-b','LineWidth',sl);
hold on;
plot(P.Xd{1}(1,:),P.Xd{1}(2,:),'--r','LineWidth',sl);
axis equal;
axis ([-3 3 -3 3]);
grid on;
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

subplot(222);
plot(P.X{2}(1,:),P.X{2}(2,:),'-b','LineWidth',sl);
hold on;
plot(P.Xd{2}(1,:),P.Xd{2}(2,:),'--r','LineWidth',sl);
axis equal;
axis ([-3 3 -3 3]);
grid on;
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

subplot(223);
plot(P.X{3}(1,:),P.X{3}(2,:),'-b','LineWidth',sl);
hold on;
plot(P.Xd{3}(1,:),P.Xd{3}(2,:),'--r','LineWidth',sl);
axis equal;
axis ([-3 3 -3 3]);
grid on;
xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

% % subplot(224);
% % plot(P.X{4}(1,:),P.X{4}(2,:),'-b','LineWidth',sl);
% % hold on;
% % plot(P.Xd{4}(1,:),P.Xd{4}(2,:),'--r','LineWidth',sl);
% % axis equal;
% % axis ([-3 3 -3 3]);
% % grid on;
% % xlabel('$x$ [m]','interpreter','Latex','FontSize',st);
% % ylabel('$y$ [m]','Interpreter','latex','FontSize',st);

%% Velocidade Robôs
figure;
subplot(211);
for i = 1:n
    plot(time,P.U{i}(1,:),'LineWidth',sl);
    hold on;
end
lgX = legend('\textit{v}$_1$','\textit{v}$_2$','\textit{v}$_3$','\textit{v}$_4$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Linear Velocity [m/s]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(a)','Interpreter','latex');
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
lgY = legend('\textit{w}$_1$','\textit{w}$_2$','\textit{w}$_3$','\textit{w}$_4$');
xlabel('Time [s]','interpreter','Latex');
ylabel('Angular Velocity [°/s]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([-0.02 0.02]);
title('(b)','Interpreter','latex');
lgY.FontSize = 10;
lgY.Location = 'NorthEast';
lgY.Orientation = 'horizontal';
set(lgY,'Interpreter','latex');
grid on;

%%
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % %%
% % % % % % sl= 1; %'default';    % largura da linha
% % % % % % st = 14;  % tamanho da fonte
% % % % % % ss = 2;   % tamanho dos símbolos
% % % % % % 
% % % % % % % Posição dos robôs
% % % % % % % Parameters
% % % % % % scale  = 1;     % robot model plot scale
% % % % % % Pcolor = 'k';   % robot A color
% % % % % % step   = 200;   % model plot step
% % % % % % h      = 0.1;   % line height
% % % % % % 
% % % % % % figure;
% % % % % % axis equal
% % % % % % axis ([-2 2 -2 2 0 3])
% % % % % % set(gca,'Box','on')
% % % % % % 
% % % % % % % ax = axis;
% % % % % % % set(gca,'xticklabel',[0 1])
% % % % % % % set(gca,'yticklabel',[])
% % % % % % % set(gca,'zticklabel',[])
% % % % % % % set(gca,'xticklabel',[])
% % % % % % 
% % % % % % hold on, grid on;
% % % % % % 
% % % % % % % Initial positions
% % % % % % ps1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'c^','MarkerSize',5,'LineWidth',3);
% % % % % % ps2 = plot3(AX(1,1),AX(2,1),AX(3,1),'c^','MarkerSize',5,'LineWidth',3);
% % % % % % 
% % % % % % % % Final positions
% % % % % % % ps1 = plot3(PX(1,end),PX(2,end),PX(3,end)+h,'ksq','MarkerSize',10,'LineWidth',3);
% % % % % % % ps2 = plot3(AX(1,end),AX(2,end),AX(3,end),'ksq','MarkerSize',10,'LineWidth',3);
% % % % % % 
% % % % % % 
% % % % % % % % Percourse made
% % % % % % % p1 = plot3(PX(1,:),PX(2,:),PX(3,:)+h,'r-','LineWidth',0.5);
% % % % % % % p2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',0.5);
% % % % % % % title('Posição dos Robôs','fontSize',lt);
% % % % % % xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
% % % % % % zlabel('$z$ [m]','interpreter','Latex');
% % % % % % 
% % % % % % % % Desired positions
% % % % % % % % pd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'k.','MarkerSize',20,'LineWidth',2);hold on % pioneer
% % % % % % % % pd2 = plot3(x2,y2,z2,'k.','MarkerSize',20,'LineWidth',2);     % drone
% % % % % % 
% % % % % % % % %Formation line
% % % % % % % % vec = [1 length(PX)];
% % % % % % % %   for k = 1:length(vec)
% % % % % % % %     x = [PX(1,vec(k))    AX(1,vec(k))];
% % % % % % % %     y = [PX(2,vec(k))    AX(2,vec(k))];
% % % % % % % %     z = [PX(3,vec(k))+h   AX(3,vec(k))];
% % % % % % % % 
% % % % % % % %     pl = line(x,y,z);
% % % % % % % %     pl.Color = 'g';
% % % % % % % %     pl.LineStyle = '-';
% % % % % % % %     pl.LineWidth = 1;
% % % % % % % %   end
% % % % % % 
% % % % % % 
% % % % % % % plot robots and formation lines
% % % % % % for k = 1:step:length(time)
% % % % % %     % Pioneer center position
% % % % % %     P.pPos.Xc([1 2 6]) = PX([1 2 6],k) - ...
% % % % % %         [P.pPar.a*cos(PX(6,k)); P.pPar.a*sin(PX(6,k)); 0];
% % % % % %     % ArDrone position
% % % % % %     A.pPos.X = AX(:,k);
% % % % % %     
% % % % % %     %   % Plota pioneer3dx bonitão
% % % % % %     try
% % % % % %     delete(fig1)
% % % % % %     delete(fig2)
% % % % % %     end
% % % % % %     P.mCADdel;
% % % % % %     P.mCADplot(scale,Pcolor);
% % % % % %     A.mCADplot;
% % % % % %     
% % % % % %     drawnow
% % % % % %     view(60,15)
% % % % % %     
% % % % % %     % plot formation line
% % % % % %     x = [PX(1,k)    AX(1,k)];
% % % % % %     y = [PX(2,k)    AX(2,k)];
% % % % % %     z = [PX(3,k)+h   AX(3,k)];
% % % % % %     
% % % % % %     pl = line(x,y,z);
% % % % % %     pl.Color = 'g';
% % % % % %     pl.LineStyle = '-';
% % % % % %     pl.LineWidth = 0.5;
% % % % % %     
% % % % % %     
% % % % % % %     hold on
% % % % % % %     % plot desired formation line
% % % % % % %     xd = [Xd(1,k)    AXd(1,k)];
% % % % % % %     yd = [Xd(2,k)    AXd(2,k)];
% % % % % % %     zd = [Xd(3,k)+h  AXd(3,k)];
% % % % % % %     
% % % % % % %     pld = line(xd,yd,zd);
% % % % % % %     pld.Color = 'm';
% % % % % % %     pld.LineStyle = '--';
% % % % % % 
% % % % % % % Percourse made
% % % % % % fig1 = plot3(PX(1,1:k),PX(2,1:k),PX(3,1:k)+h,'r-','LineWidth',0.8); hold on;
% % % % % % fig2 = plot3(AX(1,1:k),AX(2,1:k),AX(3,1:k),'b-','LineWidth',0.8);
% % % % % % 
% % % % % % 
% % % % % %     pause(.1)
% % % % % % end
% % % % % % %
% % % % % % lg1 = legend([pl ps1],{'Formation Line','Start position'});
% % % % % % % lg1 = legend([ps1 pd1],{'Start position','Desired Positions'});
% % % % % % % lg1 = legend(pl,{'Formation line'});
% % % % % % %
% % % % % % lg1.FontSize = 11;
% % % % % % lg1.Location = 'SouthEast';
% % % % % % set(lg1,'Interpreter','latex');
% % % % % % % legend('boxoff')
% % % % % % %
% % % % % % % % Salva a imagem
% % % % % % % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\posicao.png');
% % % % % % 
% % % % % % 
% % % % % % % Velocidades ...............................................................
% % % % % % % Pioneer
% % % % % % figure;
% % % % % % subplot(121), plot(time,PUd(1,:),'--','LineWidth',sl), hold on;
% % % % % % plot(time,PU(1,:),'LineWidth',sl);
% % % % % % legend({'$u_{d}$','$u_{r}$'},'Interpreter','latex');
% % % % % % grid on,xlim([0 time(end)]);
% % % % % % xlabel('Time [s]','interpreter','Latex'),ylabel('Velocity [m/s]','interpreter','Latex');
% % % % % % title('(a)','Interpreter','latex');
% % % % % % subplot(122), plot(time,180/pi*PUd(2,:),'--','LineWidth',sl), hold on;
% % % % % % plot(time,180/pi*PU(2,:),'LineWidth',sl)
% % % % % % legend({'$\omega_{d}$','$\omega_{r}$'},'Interpreter','latex');
% % % % % % grid on,xlim([0 time(end)]);
% % % % % % xlabel('Time [s]','interpreter','Latex'),ylabel('Velocity [degrees/s]','interpreter','Latex');
% % % % % % title('(b)','Interpreter','latex');
% % % % % % 
% % % % % % % ArDrone
% % % % % % figure;
% % % % % % 
% % % % % % subplot(221);
% % % % % % plot(time,AUd(1,:),'--','LineWidth',sl);hold on;
% % % % % % plot(time,AU(1,:),'LineWidth',sl);
% % % % % % axis([0 time(end) -1 1]);
% % % % % % legend({'$\phi_{d}$','$\phi_{r}$'},'Interpreter','latex');
% % % % % % grid on,xlim([0 time(end)]);
% % % % % % xlabel('Time [s]','interpreter','Latex'),ylabel('$\phi$ [degrees]','interpreter','Latex');
% % % % % % title('(a)','Interpreter','latex');
% % % % % % 
% % % % % % subplot(222);
% % % % % % plot(time,AUd(2,:),'--','LineWidth',sl);hold on;
% % % % % % plot(time,AU(2,:),'LineWidth',sl);
% % % % % % axis([0 time(end) -1 1]);
% % % % % % legend({'$\theta_{d}$','$\theta_{r}$'},'Interpreter','latex');
% % % % % % grid on,xlim([0 time(end)]);
% % % % % % xlabel('Time [s]','interpreter','Latex'),ylabel('$\theta$ [degrees]','interpreter','Latex');
% % % % % % title('(b)','Interpreter','latex');
% % % % % % 
% % % % % % subplot(223);
% % % % % % plot(time,AUd(3,:),'--','LineWidth',sl);hold on;
% % % % % % plot(time,AU(3,:),'LineWidth',sl);
% % % % % % axis([0 time(end) -1 1]);
% % % % % % legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'Interpreter','latex');
% % % % % % grid on,xlim([0 time(end)]);
% % % % % % xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{z}$ [m/s]','interpreter','Latex');
% % % % % % title('(c)','Interpreter','latex');
% % % % % % 
% % % % % % subplot(224);
% % % % % % plot(time,AUd(4,:),'--','LineWidth',sl);hold on;
% % % % % % plot(time,AU(4,:),'LineWidth',sl);
% % % % % % axis([0 time(end) -1 1]);
% % % % % % legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'Interpreter','latex');
% % % % % % grid on,xlim([0 time(end)]);
% % % % % % xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{\psi}$ [degrees/s]','interpreter','Latex');
% % % % % % title('(d)','Interpreter','latex');
% % % % % % 
% % % % % % 
% % % % % % % % % Salva a imagem
% % % % % % % % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velLinear.png');
% % % % % % %
% % % % % % %
% % % % % % % Erro da formação ....................................................................
% % % % % % figure;
% % % % % % % Position
% % % % % % subplot(211),plot(time,Qtil(1,:),'b--','LineWidth',sl),hold on;
% % % % % % plot(time,Qtil(2,:),'r-','LineWidth',sl);
% % % % % % plot(time,Qtil(4,:),'g-.','LineWidth',.5);
% % % % % % xlabel('Time [s]','interpreter','Latex'),ylabel('Error [m]','interpreter','Latex');
% % % % % % xlim([0 time(end)]);
% % % % % % title('(a)','Interpreter','latex');
% % % % % % 
% % % % % % lg3 = legend('$x_f$','$y_f$','$\rho_f$');
% % % % % % lg3.FontSize = 10;
% % % % % % lg3.Location = 'NorthEast';
% % % % % % set(lg3,'Interpreter','latex');
% % % % % % grid on;
% % % % % % 
% % % % % % 
% % % % % % % Angles
% % % % % % subplot(212),plot(time,180/pi*Qtil(5,:),'b--','LineWidth',sl),hold on;
% % % % % % plot(time,180/pi*Qtil(6,:),'r-','LineWidth',sl);
% % % % % % xlabel('Time [s]','interpreter','Latex'),ylabel('Error [degrees]','interpreter','Latex');
% % % % % % xlim([0 time(end)]);
% % % % % % title('(b)','Interpreter','latex');
% % % % % % 
% % % % % % % lg3 = legend('$x_f [m]$','$y_f [m]$','$z_f [m]$','$\rho_f [m]$','$\alpha_f [rad]$','$\beta_f [rad]$');
% % % % % % lg4 = legend('$\alpha_f$','$\beta_f$');
% % % % % % lg4.FontSize = 10;
% % % % % % lg4.Location = 'NorthEast';
% % % % % % set(lg4,'Interpreter','latex');
% % % % % % grid on
% % % % % % 
% % % % % % % Pioneer
% % % % % % figure;
% % % % % % plot(PX(1,:),PX(2,:),'LineWidth',sl), hold on;
% % % % % % plot(PXd(1,:),PXd(2,:),'--','LineWidth',sl);
% % % % % % legend({'$PX$','$PX_{d}$'},'Interpreter','latex');
% % % % % % grid on,title('(a)','Interpreter','latex');
% % % % % % 
% % % % % % figure;
% % % % % % plot3(AX(1,:),AX(2,:),AX(3,:),'LineWidth',sl), hold on;
% % % % % % plot3(AXd(1,:),AXd(2,:),AXd(3,:),'--','LineWidth',sl)
% % % % % % legend({'$AX$','$AX_{d}$'},'Interpreter','latex');
% % % % % % grid on, title('(b)','Interpreter','latex');
% % % % % % axis equal
% % % % % % axis ([-2 2 -2 2 0 3])
% % % % % % set(gca,'Box','on')
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % %
% % % % % % % % % Salva a imagem
% % % % % % % % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\Erroformacao.png');
% % % % % % %
% % % % % % %%
% % % % % % % % % Sinal de controle - velocidade linear
% % % % % % % % figure;
% % % % % % % % hold on, grid on;
% % % % % % % % plot(time,Ud(:,1),'LineWidth',sl);
% % % % % % % % plot(time,AUd(:,1),'LineWidth',sl);
% % % % % % % % title('Sinal de controle (Velocidade Linear)','fontSize',st);
% % % % % % % % xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [m/s]','interpreter','Latex');
% % % % % % % % legend('Robô 1', 'Robô 2');
% % % % % % %
% % % % % % %
% % % % % % % % Velocidades Angulares
% % % % % % % figure;
% % % % % % % hold on, grid on;
% % % % % % % plot(time,U(:,2),'r','LineWidth',sl);
% % % % % % % plot(time,AU(:,2),'b','LineWidth',sl);
% % % % % % % % title('Velocidade Angular','fontSize',st);
% % % % % % % xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Speed$ [rad/s]','interpreter','Latex');
% % % % % % % legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
% % % % % % % xlim([0 time(end)]);
% % % % % % %
% % % % % % % % % Salva a imagem
% % % % % % % % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velAngular.png');
% % % % % % %
% % % % % % %
% % % % % % % % % Sinal de controle - velocidade angular
% % % % % % % % figure;
% % % % % % % % hold on, grid on;
% % % % % % % % plot(time,Ud(:,2),'LineWidth',sl);
% % % % % % % % plot(time,AUd(:,2),'LineWidth',sl);
% % % % % % % % title('Sinal de controle (Velocidade Angular)','fontSize',st);
% % % % % % % % xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [rad/s]','interpreter','Latex');
% % % % % % % % legend('Robô 1', 'Robô 2');
% % % % % % %
% % % % % % % %% Erros
% % % % % % % % % Erro de posição
% % % % % % % % figure;
% % % % % % % % hold on, grid on;
% % % % % % % % plot(time,Xtil(:,[1 2]),'LineWidth',sl);
% % % % % % % % plot(time,AXtil(:,[1 2]),'LineWidth',sl);
% % % % % % % % % title('Erro de posição','fontSize',st);
% % % % % % % % xlabel('$Time$ [s]','interpreter','Latex'),ylabel('Error [m]','interpreter','Latex');
% % % % % % % % xlim([0 time(end)]);
% % % % % % % %
% % % % % % % % lg2 = legend('$x_{robot 1}$','$y_{robot 1}$', '$x_{robot 2}$','$y_{robot 2}$');
% % % % % % % % lg2.FontSize = 12;
% % % % % % % % lg2.Location = 'SouthEast';
% % % % % % % % set(lg2,'Interpreter','latex');
% % % % % % % %
% % % % % % % % % % Salva a imagem
% % % % % % % % % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\ErroPosicao.png');
% % % % % % %
