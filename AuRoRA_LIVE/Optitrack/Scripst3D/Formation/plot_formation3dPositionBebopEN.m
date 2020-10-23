%% Rotina para plotar gráficos de Formacaolinha3D
clear
close all
clc

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Declara os robos
P  = Pioneer3DX;
A  = ArDrone;
LF = LineFormation3D;

%% Carrega dados
% data = load('FL3dSIMU_20180925T223717.txt');
data = load('FL3d_PositionUtimateBebop20200113T193945.txt');

%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Pioneer data
PXd   = data(:,(1:12))';       % desired pose
PX    = data(:,12+(1:12))';    % real pose
PUd   = data(:,24+(1:2))';     % control signal
% PU    = data(:,26+(1:2))';     % velocidades do robô
PXtil = PXd - PX; % erro de postura

% Drone data
AXd   = data(:,28+(1:12))';
AX    = data(:,40+(1:12))';
AUd   = data(:,52+(1:6))';
% AU    = data(:,58+(1:4))';  


% Formation data
Qd   = data(:,62+(1:6))';   % desired formation
Q    = data(:,68+(1:6))';   % real formation

% Q(4,:) = Q(4,:) ;%+ 0.3;      % add pioneer height at rho distance

% Alpha quadrant adjust
% a = 1;
% for ii = 1:length(Q)-1
%     angle = Q(5,ii+1) - Q(5,ii);
%     if abs(angle)<6.5  && abs(angle) > 6  % detect breaks
%         ind(a) = ii;   % save break vector indice
%         a = a+1;
%     end
% end      % detect breaks
% 
% for f = 1:2:length(ind)-1 
%     for jj = ind(f):ind(f+1)-1
%         if Q(5,jj+1)>0      % -pi --> +pi
%             Q(5,jj+1) = Q(5,jj+1) - 2*pi;
%         elseif Q(5,jj+1)<0  % +pi --> -pi
%             Q(5,jj+1) = Q(5,jj+1) + 2*pi;
%         end
%     end
% end  % do the thing

Qtil = Qd - Q;              % formation error

% Robots desired positions
for kk = 1:length(Qd)
    xf   = Qd(1,kk);  yf    = Qd(2,kk);   zf = Qd(3,kk);
    rhof = Qd(4,kk);  alfaf = Qd(5,kk);   betaf = Qd(6,kk);
    
    x2(kk) = xf + rhof*cos(alfaf)*cos(betaf);
    y2(kk) = yf + rhof*sin(alfaf)*cos(betaf);
    z2(kk) = zf + rhof*sin(betaf);
end

AXd(1:3,:) = [x2;y2;z2];
% Position Error
AXtil = AXd - AX;
% 
% % Quadrant correction 
% for k=1:length(Qd)
%     if abs(Qd(5,k)) > pi
%         if Qd(5,k) > 0
%             Qd(5,k) = Qd(5,k) - 2*pi;
%         else
%             Qd(5,k) = Qd(5,k) + 2*pi;
%         end
%     end
%     if abs(Q(5,k)) > pi
%         if Q(5,k) > 0
%             Q(5,k) = Q(5,k) - 2*pi;
%         else
%             Q(5,k) = Q(5,k) + 2*pi;
%         end
%     end
%      if abs(Qtil(5,k)) > pi
%         if Qtil(5,k) > 0
%             Qtil(5,k) = Qtil(5,k) - 2*pi;
%         else
%             Qtil(5,k) = Qtil(5,k) + 2*pi;
%         end
%     end
% end
% 



%% PLOTA RESULTADOS
% Tamanho das letras, gráficos e símbolos
sizeDesired = 1.2;      % largura das linhas de valores desejados
sizeReal    = 1;      % largura da linha de valor real
sizeLabel   = 16;     % tamanho da fonte dos labels
sizeSymbol  = [10 2];  % [tamanho dos símbolos   espessura da linha]
sizeLegenda = 15;     % tamanho das legendas
location    = 'SouthEast'; %'NorthEast';
% Posição dos robôs
% Parameters
scale  = 1;     % pioneer model plot scale
Pcolor = 'k';   % pioneer
step   = 50;   % model plot step
h      = 0.1;   % line height (to plot above the ground)

figure;
axis equal
axis ([-2 2 -2 2 0 2])
set(gca,'Box','on')
hold on, grid on;

% Initial positions
ps1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'c^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
ps2 = plot3(AX(1,1),AX(2,1),AX(3,1),'c^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% % Final positions
% ps1 = plot3(PX(1,end),PX(2,end),PX(3,end)+h,'ksq','MarkerSize',10,'LineWidth',3);
% ps2 = plot3(AX(1,end),AX(2,end),AX(3,end),'ksq','MarkerSize',10,'LineWidth',3);


% % Percourse made
% p1 = plot3(PX(1,:),PX(2,:),PX(3,:)+h,'r-','LineWidth',0.5);
% p2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',0.5);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex', 'FontSize',sizeLabel),ylabel('$y$ [m]','Interpreter','latex', 'FontSize',sizeLabel);
zlabel('$z$ [m]','interpreter','Latex', 'FontSize',sizeLabel);

% % Desired positions
pd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'ro','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));hold on % pioneer
pd2 = plot3(x2,y2,z2,'bx','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));     % drone


% plot robots and formation lines
for k = 1:step:length(time)
    % Pioneer center position
%     P.pPos.Xc([1 2 6]) = PX([1 2 6],k) - ...
%         [P.pPar.a*cos(PX(6,k)); P.pPar.a*sin(PX(6,k)); 0];

    P.pPos.Xc([1 2 6]) = PX([1 2 6],k); 
    % Bebopposition
    A.pPos.X = AX(:,k);
    
    %   % Plota pioneer3dx bonitão
    try
        delete(fig1)
        delete(fig2)
    end
    P.mCADdel;
    P.mCADplot(scale,Pcolor);
    A.mCADplot;
    
    drawnow
    view(60,15)
    
%     % plot formation line
%     x = [PX(1,k)    AX(1,k)];
%     y = [PX(2,k)    AX(2,k)];
%     z = [PX(3,k)+h   AX(3,k)];
%     
%     pl = line(x,y,z);
%     pl.Color = 'g';
%     pl.LineStyle = '-';
%     pl.LineWidth = 0.5;
%     
    
    %     hold on
    %     % plot desired formation line
    %     xd = [Xd(1,k)    AXd(1,k)];
    %     yd = [Xd(2,k)    AXd(2,k)];
    %     zd = [Xd(3,k)+h  AXd(3,k)];
    %
    %     pld = line(xd,yd,zd);
    %     pld.Color = 'm';
    %     pld.LineStyle = '--';
    
    % Percourse made
    plot3(PX(1,1:k),PX(2,1:k),PX(3,1:k)+h,'r-','LineWidth',0.8); hold on;
    plot3(AX(1,1:k),AX(2,1:k),AX(3,1:k),'b-','LineWidth',0.8);
    
    
    pause(1/30)
end
%
% lg1 = legend([pl ps1 pd1 pd2],{'Linha da forma\c{c}\~{a}o','Posi\c{c}\~{a}o inicial','Trajet\''{o}ria desejada Pioneer 3-DX','Trajet\''{o}ria desejada AR.Drone2.0'});
lg1 = legend([ps1 pd1 pd2],{'Initial position','Pioneer 3-DX desired position','Bebop desired position'});
% lg1 = legend([ps1 pd1],{'Start position','Desired Positions'});
% lg1 = legend(pl,{'Formation line'});
%
lg1.FontSize = sizeLegenda;
lg1.Location = 'SouthEast';
set(lg1,'Interpreter','latex');

% ##########################################################################
%% Separated windows
% ##########################################################################

figure;
axis equal
axis ([-2 2 -2 2 0 2])
set(gca,'Box','on')

hold on, grid on;

% Initial positions
pstart1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'c^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
pstart2 = plot3(AX(1,1),AX(2,1),AX(3,1),'c^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% % Final positions
pend1 = plot3(PX(1,end),PX(2,end),PX(3,end)+h,'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
pend2 = plot3(AX(1,end),AX(2,end),AX(3,end),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));


% % Percourse made
pp1 = plot3(PX(1,:),PX(2,:),PX(3,:)+h,'r-','LineWidth',sizeReal);
pp2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',sizeReal);
xlabel('$x$ [m]','interpreter','Latex', 'FontSize',sizeLabel),ylabel('$y$ [m]','Interpreter','latex', 'FontSize',sizeLabel);
zlabel('$z$ [m]','interpreter','Latex', 'FontSize',sizeLabel);

% % Desired positions
pdd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'ro','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));hold on % pioneer
pdd2 = plot3(x2,y2,z2,'bx','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));     % drone


%Formation line
vec = [1 length(PX)];
for k = 1:length(vec)
    x = [PX(1,vec(k))    AX(1,vec(k))];
    y = [PX(2,vec(k))    AX(2,vec(k))];
    z = [PX(3,vec(k))+h   AX(3,vec(k))];
    
    ppl = line(x,y,z);
    ppl.Color = 'g';
    ppl.LineStyle = '-';
    ppl.LineWidth = 1;
end


leg1 = legend([ppl pstart1 pend1 pdd1 pdd2],{'Formation line',...
 'Initial position','Final position', ...
 'Pioneer 3-DX desired position','Bebop desired position'});
leg1.FontSize = sizeLegenda;
leg1.Location = location;
set(leg1,'Interpreter','latex');
% Superior View *********************************************************
figure;
axis equal
axis ([-1.5 1.5 -1.5 1.5])
set(gca,'Box','on')

hold on, grid on;

% Initial positions
pstart1 = plot(PX(1,1),PX(2,1),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
pstart2 = plot(AX(1,1),AX(2,1),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% % Final positions
pend1 = plot(PX(1,end),PX(2,end),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
pend2 = plot(AX(1,end),AX(2,end),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));


% Percourse made
preal1 = plot(PX(1,:),PX(2,:),'r-','LineWidth',sizeReal);
preal2 = plot(AX(1,:),AX(2,:),'b-','LineWidth',sizeReal);
xlabel('$x$ [m]','interpreter','Latex', 'FontSize',sizeLabel),ylabel('$y$ [m]','Interpreter','latex', 'FontSize',sizeLabel);

% Desired positions
pdes1 = plot(Qd(1,:),Qd(2,:),'ro','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));hold on % pioneer
pdes2 = plot(x2,y2,'bx','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));     % drone


leg1 = legend([pstart1 pend1 pdes1 pdes2 preal1 preal2],{'Initial position', ...
    'Final position','Pioneer 3-DX desired position', ...
    'Bebop desired position', ...
    'Pioneer 3-DX real position','Bebop real position'});

leg1.FontSize = sizeLegenda;
leg1.Location = location;
set(leg1,'Interpreter','latex');
% 
% % ######################################################################
% %% Velocidades 
% % Pioneer ****************************************************************
% figure;
% plot(time,PUd(1,:),'b--','LineWidth',sizeDesired), hold on;
% plot(time,PU(1,:),'r','LineWidth',sizeReal);
% legend({'$u_{d}$','$u_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Velocidade [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% 
% figure;
% plot(time,PUd(2,:),'b--','LineWidth',sizeDesired), hold on;
% plot(time,PU(2,:),'r','LineWidth',sizeReal)
% legend({'$\omega_{d}$','$\omega_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Velocidade [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% 
% % Bebop****************************************************************
% % phi
% figure;
% plot(time,AUd(1,:),'b--','LineWidth',sizeDesired);hold on;
% plot(time,AU(1,:),'r','LineWidth',sizeReal);
% legend({'$\phi_{d}$','$\phi_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\phi$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% 
% % theta
% figure;
% plot(time,AUd(2,:),'b--','LineWidth',sizeDesired);hold on;
% plot(time,AU(2,:),'r','LineWidth',sizeReal);
% legend({'$\theta_{d}$','$\theta_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\theta$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% 
% % dZ
% figure;
% plot(time,AUd(3,:),'b--','LineWidth',sizeDesired);hold on;
% plot(time,AU(3,:),'r','LineWidth',sizeReal);
% legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% grid on,xlim([0 time(end)]),ylim([-1 1]);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\dot{z}$ [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% 
% % dPsi
% figure;
% plot(time,AUd(4,:),'b--','LineWidth',sizeDesired);hold on;
% plot(time,-AU(4,:),'r','LineWidth',sizeReal);
% legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\dot{\psi}$ [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% 

% Erro da formação ****************************************************
% Position
figure;
plot(time,Qtil(1,:),'b--','LineWidth',sizeReal),hold on;
plot(time,Qtil(2,:),'r-','LineWidth',sizeReal);
plot(time,Qtil(4,:),'g-.','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Error [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgg3 = legend('$x_f$','$y_f$','$\rho_f$');
lgg3.FontSize = 14;
lgg3.Location = location;
set(lgg3,'Interpreter','latex');
grid on;


% Angles
figure;
plot(time,Qtil(5,:),'b--','LineWidth',sizeReal),hold on;
plot(time,Qtil(6,:),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Error [rad]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgg4 = legend('$\alpha_f$','$\beta_f$');
lgg4.FontSize = 14;
lgg4.Location = location;
set(lgg4,'Interpreter','latex');
grid on

% #########################################################################
%% Plota variáveis da formação
% xf
figure;
plot(time,(Qd(1,:)),'b--','LineWidth',sizeReal),hold on;
plot(time,(Q(1,:)),'r-','LineWidth',sizeReal);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Dist\^ancia [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgg1 = legend('$x_{fd}$','$x_{fr}$');
lgg1.FontSize = 14;
lgg1.Location = location;
set(lgg1,'Interpreter','latex');
grid on

% yf
figure;
plot(time,(Qd(2,:)),'b--','LineWidth',sizeReal),hold on;
plot(time,(Q(2,:)),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Distance [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgg2 = legend('$y_{fd}$','$y_{fr}$');
lgg2.FontSize = 14;
lgg2.Location = location;
set(lgg2,'Interpreter','latex');
grid on

% rhof
figure;
plot(time,(Qd(4,:)),'b--','LineWidth',sizeReal),hold on;
plot(time,(Q(4,:)),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Distance [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgg4 = legend('$\rho_{fd}$','$\rho_{fr}$');
lgg4.FontSize = 14;
lgg4.Location = location;
set(lgg4,'Interpreter','latex');
grid on

% alphaf
figure;
plot(time,rad2deg(Qd(5,:)),'b--','LineWidth',sizeReal),hold on;
plot(time,rad2deg(Q(5,:)),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Angle [degree]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgg5 = legend('$\alpha_{fd}$','$\alpha_{fr}$');
lgg5.FontSize = 14;
lgg5.Location = location;
set(lgg5,'Interpreter','latex');
grid on

% betaf
figure;
plot(time,rad2deg(Qd(6,:)),'b--','LineWidth',sizeReal),hold on;
plot(time,rad2deg(Q(6,:)),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Angle [degree]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgg6 = legend('$\beta_{fd}$','$\beta_{fr}$');
lgg6.FontSize = 14;
lgg6.Location = location;
set(lgg6,'Interpreter','latex');
grid on

% #########################################################################
%% Variáveis dos robôs
% Pioneer ***********************************************************
%x
figure;
plot(time,PXd(1,:),'b--','LineWidth',sizeReal),hold on;
plot(time,PX(1,:),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Distance [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgp1 = legend('$x_{1d}$','$x_{1r}$');
lgp1.FontSize = 14;
lgp1.Location = location;
set(lgp1,'Interpreter','latex');
grid on

%y
figure;
plot(time,PXd(2,:),'b--','LineWidth',sizeReal),hold on;
plot(time,PX(2,:),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Distance [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgp2 = legend('$y_{1d}$','$y_{1r}$');
lgp2.FontSize = 14;
lgp2.Location = location;
set(lgp2,'Interpreter','latex');
grid on

% %z   % se for pioneer será sempre 0
% figure;
% plot(time,PXd(3,:),'b--','LineWidth',sizeReal),hold on;
% plot(time,PX(3,:),'r-','LineWidth',sizeReal);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Dist\^ancia [m]','FontSize',sizeLabel,'interpreter','Latex');
% xlim([0 time(end)]);
% 
% lgp3 = legend('$z_{1d}$','$z_{1r}$');
% lgp3.FontSize = 14;
% lgp3.Location = location;
% set(lgp3,'Interpreter','latex');
% grid on


% Drone *****************************************************************
%x
figure;
plot(time,AXd(1,:),'b--','LineWidth',sizeReal),hold on;
plot(time,AX(1,:),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Distance [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgd1 = legend('$x_{2d}$','$x_{2r}$');
lgd1.FontSize = 14;
lgd1.Location = location;
set(lgd1,'Interpreter','latex');
grid on

%y
figure;
plot(time,AXd(2,:),'b--','LineWidth',sizeReal),hold on;
plot(time,AX(2,:),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Distance [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgd2 = legend('$y_{2d}$','$y_{2r}$');
lgd2.FontSize = 14;
lgd2.Location = location;
set(lgd2,'Interpreter','latex');
grid on

%z
figure;
plot(time,AXd(3,:),'b--','LineWidth',sizeReal),hold on;
plot(time,AX(3,:),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Altitude [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lgd3 = legend('$z_{2d}$','$z_{2r}$');
lgd3.FontSize = 14;
lgd3.Location = location;
set(lgd3,'Interpreter','latex');
grid on

% #########################################################################
% % %% Com Subplots
% % % Velocidades ...............................................................
% % % Pioneer
% % figure;
% % subplot(321)
% % plot(time,PUd(1,:),'b--','LineWidth',sizeDesired), hold on;
% % plot(time,PU(1,:),'r','LineWidth',sizeReal);
% % legend({'$u_{d}$','$u_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('Velocidade [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% % % title('(a)','Interpreter','latex');
% % 
% % subplot(322),
% % plot(time,PUd(2,:),'b--','LineWidth',sizeDesired), hold on;
% % plot(time,PU(2,:),'r','LineWidth',sizeReal)
% % legend({'$\omega_{d}$','$\omega_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('Velocidade [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% % % title('(b)','Interpreter','latex');
% % 
% % % ArDrone
% % % figure;
% % subplot(323)
% % plot(time,AUd(1,:),'b--','LineWidth',sizeDesired);hold on;
% % plot(time,AU(1,:),'r','LineWidth',sizeReal);
% % legend({'$\phi_{d}$','$\phi_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('$\phi$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% % % title('(a)','Interpreter','latex');
% % 
% % subplot(324)
% % plot(time,AUd(2,:),'b--','LineWidth',sizeDesired);hold on;
% % plot(time,AU(2,:),'r','LineWidth',sizeReal);
% % legend({'$\theta_{d}$','$\theta_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('$\theta$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% % % title('(b)','Interpreter','latex');
% % 
% % subplot(325)
% % plot(time,AUd(3,:),'b--','LineWidth',sizeDesired);hold on;
% % plot(time,AU(3,:),'r','LineWidth',sizeReal);
% % legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('$\dot{z}$ [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% % % title('(c)','Interpreter','latex');
% % 
% % subplot(326)
% % plot(time,AUd(4,:),'b--','LineWidth',sizeDesired);hold on;
% % plot(time,-AU(4,:),'r','LineWidth',sizeReal);
% % legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('$\dot{\psi}$ [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% % % title('(d)','Interpreter','latex');
% % 
% % 
% % % Erro da formação ....................................................................
% % 
% % % Position
% % figure;
% % subplot(211)
% % plot(time,Qtil(1,:),'b--','LineWidth',sizeReal),hold on;
% % plot(time,Qtil(2,:),'r-','LineWidth',sizeReal);
% % plot(time,Qtil(4,:),'g-.','LineWidth',sizeReal);
% % xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('Erro [m]','FontSize',sizeLabel,'interpreter','Latex');
% % xlim([0 time(end)]);
% % % title('(a)','Interpreter','latex');
% % 
% % lg3 = legend('$x_f$','$y_f$','$\rho_f$');
% % lg3.FontSize = 14;
% % lg3.Location = 'SouthEast';
% % set(lg3,'Interpreter','latex');
% % grid on;
% % 
% % 
% % % Angles
% % % figure;
% % subplot(212)
% % plot(time,Qtil(5,:),'b--','LineWidth',sizeReal),hold on;
% % plot(time,Qtil(6,:),'r-','LineWidth',sizeReal);
% % xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('Erro [rad]','FontSize',sizeLabel,'interpreter','Latex');
% % xlim([0 time(end)]);
% % 
% % lg4 = legend('$\alpha_f$','$\beta_f$');
% % lg4.FontSize = 14;
% % lg4.Location = 'SouthEast';
% % set(lg4,'Interpreter','latex');
% % grid on

% 
% %% Com Subplots
% % Velocidades ...............................................................
% % Pioneer
% figure;
% subplot(321)
% plot(time,PUd(1,:),'b--','LineWidth',sizeDesired), hold on;
% plot(time,PU(1,:),'r','LineWidth',sizeReal);
% % legend({'$u_{d}$','$u_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Linear Velocity [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(a)','Interpreter','latex');
% 
% subplot(322),
% plot(time,PUd(2,:),'b--','LineWidth',sizeDesired), hold on;
% plot(time,PU(2,:),'r','LineWidth',sizeReal)
% % legend({'$\omega_{d}$','$\omega_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Angular Velocity [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(b)','Interpreter','latex');
% 
% % ArDrone
% % figure;
% subplot(323)
% plot(time,AUd(1,:),'b--','LineWidth',sizeDesired);hold on;
% plot(time,AU(1,:),'r','LineWidth',sizeReal);
% % legend({'$\phi_{d}$','$\phi_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Roll [rad]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(a)','Interpreter','latex');
% 
% subplot(324)
% plot(time,AUd(2,:),'b--','LineWidth',sizeDesired);hold on;
% plot(time,AU(2,:),'r','LineWidth',sizeReal);
% % legend({'$\theta_{d}$','$\theta_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Pitch [rad]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(b)','Interpreter','latex');
% 
% subplot(325)
% plot(time,AUd(3,:),'b--','LineWidth',sizeDesired);hold on;
% plot(time,AU(3,:),'r','LineWidth',sizeReal);
% % legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Altitude rate [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(c)','Interpreter','latex');
% 
% subplot(326)
% plot(time,AUd(4,:),'b--','LineWidth',sizeDesired);hold on;
% plot(time,-AU(4,:),'r','LineWidth',sizeReal);
% % legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Heading rate [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(d)','Interpreter','latex');
% legend({'Desired','$Real$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% 
% 
% % Erro da formação ....................................................................
% 
% % Position
% figure;
% subplot(211)
% plot(time,Qtil(1,:),'b--','LineWidth',sizeReal),hold on;
% plot(time,Qtil(2,:),'r-','LineWidth',sizeReal);
% plot(time,Qtil(4,:),'g-.','LineWidth',sizeReal);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Erro [m]','FontSize',sizeLabel,'interpreter','Latex');
% xlim([0 time(end)]);
% % title('(a)','Interpreter','latex');
% 
% lg3 = legend('$x_f$','$y_f$','$\rho_f$');
% lg3.FontSize = 14;
% lg3.Location = 'SouthEast';
% set(lg3,'Interpreter','latex');
% grid on;
% 
% 
% % Angles
% % figure;
% subplot(212)
% plot(time,Qtil(5,:),'b--','LineWidth',sizeReal),hold on;
% plot(time,Qtil(6,:),'r-','LineWidth',sizeReal);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Erro [rad]','FontSize',sizeLabel,'interpreter','Latex');
% xlim([0 time(end)]);
% 
% lg4 = legend('$\alpha_f$','$\beta_f$');
% lg4.FontSize = 14;
% lg4.Location = 'SouthEast';
% set(lg4,'Interpreter','latex');
% grid on
% 
