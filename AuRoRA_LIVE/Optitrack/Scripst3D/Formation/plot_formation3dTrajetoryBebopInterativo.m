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
% data = load('FL3d_TrajectoryUtimateBebop20200114T012033.txt');
data = load('FL3d_TrajectoryUtimateBebop20200113T232638.txt');
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

% % Alpha quadrant adjust
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
%
Qtil = Qd - Q;              % formation error


% Second robot desired positions
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

%% Angle convertion (rad2deg)
% X(4:6,:) = rad2deg(X(4:6,:));
% U(2,:) = rad2deg(U(2,:));
% Xd(4:6,:) = rad2deg(Xd(4:6,:));
% Ud(2,:) = rad2deg(Ud(2,:));

% AX(4:6,:) = rad2deg(AX(4:6,:));
% AU([1 2 4],:) = rad2deg(AU([1 2 4],:));
% AXd(4:6,:) = rad2deg(AXd(4:6,:));
% AUd([1 2 4],:) = rad2deg(AUd([1 2 4],:));

% Qd([5 6],:) = rad2deg(Qd([5 6],:));
% Qtil([5 6],:) = rad2deg(Qtil([5 6],:));

%% PLOTA RESULTADOS
% Tamanho das letras, gráficos e símbolos
sizeDesired = 2;      % largura das linhas de valores desejados
sizeReal    = 1;      % largura da linha de valor real
sizeLabel   = 14;     % tamanho da fonte dos labels
sizeSymbol  = [20 2];  % [tamanho dos símbolos   espessura da linha]
sizeLegenda = 15;     % tamanho das legendas
location    = 'SouthEast'; %'NorthEast';
% Posição dos robôs
% Parameters
scale  = 1;     % pioneer model plot scale
Pcolor = 'k';   % pioneer
step   = 10;   % model plot step
h      = 0.1;   % line height (to plot above the ground)

figure(1);  
% view(0,30);
view(-140,20);
% view(-100,9);
% script for maximize plot window ---
% pause(0.00001);
% frame_h = get(handle(gcf),'JavaFrame');
% set(frame_h,'Maximized',1);
% --------------------------------
axis equal
axis ([-2 2 -2 2 0 2])
set(gca,'Box','on')

pause(3);

% ax = axis;
% set(gca,'xticklabel',[0 1])
% set(gca,'yticklabel',[])
% set(gca,'zticklabel',[])
% set(gca,'xticklabel',[])
% pause    % para dar tempo de iniciar captura de tela
hold on, grid on;

% % Initial positions
% ps1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'c^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% ps2 = plot3(AX(1,1),AX(2,1),AX(3,1),'c^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% % Final positions
% ps1 = plot3(PX(1,end),PX(2,end),PX(3,end)+h,'ksq','MarkerSize',10,'LineWidth',3);
% ps2 = plot3(AX(1,end),AX(2,end),AX(3,end),'ksq','MarkerSize',10,'LineWidth',3);


% % Percourse made
% p1 = plot3(PX(1,:),PX(2,:),PX(3,:)+h,'r-','LineWidth',0.5);
% p2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',0.5);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','interpreter','Latex');

% % % Desired positions
% pd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'k--','LineWidth',2);hold on % pioneer
% pd2 = plot3(x2,y2,z2,'k--','LineWidth',2);     % drone

% pioneer
ppd1 = plot3(PXd(1,1),PXd(2,1),PXd(3,1)+h,'k*','LineWidth',sizeSymbol(2),'MarkerSize',sizeSymbol(1));  % desired
pp1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'ro','LineWidth',sizeSymbol(2),'MarkerSize',sizeSymbol(1));      % real

% drone
ppd2 = plot3(AXd(1,1),AXd(2,1),AXd(3,1),'k*','LineWidth',sizeSymbol(2),'MarkerSize',sizeSymbol(1)); % desired
pp2 = plot3(AX(1,1),AX(2,1),AX(3,1),'bo','LineWidth',sizeSymbol(2),'MarkerSize',sizeSymbol(1));     % real

% plot formation lines
    % plot formation line
    x = [PX(1,1)    AX(1,1)];
    y = [PX(2,1)    AX(2,1)];
    z = [PX(3,1)+h   AX(3,1)];
    
    pl = line(x,y,z);
    pl.Color = 'g';
    pl.LineStyle = '--';
    pl.LineWidth = 0.8;
    
    
    % plot desired formation line
    xd = [PXd(1,1)    AXd(1,1)];
    yd = [PXd(2,1)    AXd(2,1)];
    zd = [PXd(3,1)+h  AXd(3,1)];
    
    pld = line(xd,yd,zd);
    pld.Color = 'k';
    pld.LineStyle = '-';
    pld.LineWidth = 0.8;
    
    
lg1 = legend([ppd1 pp1],{'Desired','Real'});
% lg1 = legend([ps1 pd1],{'Start position','Desired Positions'});
% lg1 = legend(pl,{'Formation line'});
%
lg1.FontSize = sizeLegenda;
lg1.Location = 'NorthWest';
 set(lg1,'Interpreter','latex');
% legend('boxoff')


pause(5);
% delete(lg1);
legend off

for k = 1:step:length(time)
    
    
    
    % Pioneer center position
    P.pPos.Xc([1 2 6]) = PX([1 2 6],k) - ...
        [P.pPar.a*cos(PX(6,k)); P.pPar.a*sin(PX(6,k)); 0];
    % Bebop position
    A.pPos.X = AX(:,k);
    
    %     %   % Plota pioneer3dx bonitão
    %     try
    %         delete(fig1)
    %         delete(fig2)
    %     end
    %     P.mCADdel;
    %     P.mCADplot(scale,Pcolor);
    %     A.mCADplot;
    
    drawnow
    %     view(60,15)
%     view(-100,9)
%     view(0,30);  % vista da gopro
    view(-140,20);  % vista da porta do lab
    
    % delete previous points
    delete(pl), delete(pld);
    % plot formation line
    x = [PX(1,k)    AX(1,k)];
    y = [PX(2,k)    AX(2,k)];
    z = [PX(3,k)+h   AX(3,k)];
    
    pl = line(x,y,z);
    pl.Color = 'g';
    pl.LineStyle = '--';
    pl.LineWidth = 0.8;
    
    % plot desired formation line
    xd = [PXd(1,k)    AXd(1,k)];
    yd = [PXd(2,k)    AXd(2,k)];
    zd = [PXd(3,k)+h  AXd(3,k)];
    
    pld = line(xd,yd,zd);
    pld.Color = 'k';
    pld.LineStyle = '-';
    pld.LineWidth = 0.8;
    
    delete(ppd1), delete(pp1), delete(ppd2), delete(pp2);
    
    % Desired positions
    ppd1 = plot3(PXd(1,k),PXd(2,k),PXd(3,k)+h,'k*','LineWidth',sizeSymbol(2),'MarkerSize',sizeSymbol(1));  % pioneer
    pp1 = plot3(PX(1,k),PX(2,k),PX(3,k)+h,'ro','LineWidth',sizeSymbol(2),'MarkerSize',sizeSymbol(1));
    
    ppd2 = plot3(AXd(1,k),AXd(2,k),AXd(3,k),'k*','LineWidth',sizeSymbol(2),'MarkerSize',sizeSymbol(1)); % drone
    pp2 = plot3(AX(1,k),AX(2,k),AX(3,k),'bo','LineWidth',sizeSymbol(2),'MarkerSize',sizeSymbol(1));
    
        
    %     % Percourse made
    fig1 = plot3(PX(1,1:k),PX(2,1:k),PX(3,1:k)+h,'r-','LineWidth',0.8); hold on;
    fig2 = plot3(AX(1,1:k),AX(2,1:k),AX(3,1:k),'b-','LineWidth',0.8);
    
    
    pause(.2)
    
end

% % % Desired positions
% pd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'k--','LineWidth',2);hold on % pioneer
% pd2 = plot3(x2,y2,z2,'k--','LineWidth',2);     % drone

% 
% lg1 = legend([pl ps1 pd1],{'Formation Line','Start position','Desired Trajectory'});
% % lg1 = legend([ps1 pd1],{'Start position','Desired Positions'});
% % lg1 = legend(pl,{'Formation line'});
% %
% lg1.FontSize = sizeLegenda;
% lg1.Location = 'SouthEast';
% set(lg1,'Interpreter','latex');
% legend('boxoff')
%
% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\posicao.png');

%
% % Velocidades ...............................................................
% % Pioneer
% figure;
% subplot(321)
% plot(time,PUd(1,:),'b--','LineWidth',sizeDesired)
% legend({'$u_{d}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Velocity [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(a)','Interpreter','latex');
%
% subplot(322),
% plot(time,PUd(2,:),'b--','LineWidth',sizeDesired)
% legend({'$\omega_{d}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Velocity [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(b)','Interpreter','latex');
%
% % Bebop
% % figure;
% subplot(323)
% plot(time,AUd(1,:),'b--','LineWidth',sizeDesired);hold on;
% % plot(time,AU(1,:),'r','LineWidth',sizeReal);
% % legend({'$\phi_{d}$','$\phi_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% legend({'$\phi_{d}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\phi$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(a)','Interpreter','latex');
%
% subplot(324)
% plot(time,AUd(2,:),'b--','LineWidth',sizeDesired);hold on;
% % plot(time,AU(2,:),'r','LineWidth',sizeReal);
% % legend({'$\theta_{d}$','$\theta_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% legend({'$\theta_{d}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
%
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\theta$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(b)','Interpreter','latex');
%
% subplot(325)
% plot(time,AUd(3,:),'b--','LineWidth',sizeDesired);hold on;
% % plot(time,AU(3,:),'r','LineWidth',sizeReal);
% % legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% legend({'$\dot{z}_{d}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\dot{z}$ [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(c)','Interpreter','latex');
%
% subplot(326)
% plot(time,AUd(4,:),'b--','LineWidth',sizeDesired);hold on;
% % plot(time,-AU(4,:),'r','LineWidth',sizeReal);
% % legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% legend({'$\dot{\psi}_{d}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\dot{\psi}$ [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% % title('(d)','Interpreter','latex');

%
% % Erro da formação ....................................................................
%
% % Position
% figure;
% subplot(211)
% plot(time,Qtil(1,:),'b--','LineWidth',sizeReal),hold on;
% plot(time,Qtil(2,:),'r-','LineWidth',sizeReal);
% plot(time,Qtil(4,:),'g-.','LineWidth',sizeReal);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Error [m]','FontSize',sizeLabel,'interpreter','Latex');
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
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Error [rad]','FontSize',sizeLabel,'interpreter','Latex');
% xlim([0 time(end)]);
% % title('(b)','Interpreter','latex');
%
% % lg3 = legend('$x_f [m]$','$y_f [m]$','$z_f [m]$','$\rho_f [m]$','$\alpha_f [rad]$','$\beta_f [rad]$');
% lg4 = legend('$\alpha_f$','$\beta_f$');
% lg4.FontSize = 14;
% lg4.Location = 'SouthEast';
% set(lg4,'Interpreter','latex');
% grid on
%
%
% % ##########################################################################
% %% Separated windows
% % ##########################################################################
%
% figure;
% axis equal
% axis ([-1.5 1.5 -2 2 0 2])
% set(gca,'Box','on')
%
% hold on, grid on;
%
% % Initial positions
% pstart1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'c^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% pstart2 = plot3(AX(1,1),AX(2,1),AX(3,1),'c^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
%
% % % Final positions
% pend1 = plot3(PX(1,end),PX(2,end),PX(3,end)+h,'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% pend2 = plot3(AX(1,end),AX(2,end),AX(3,end),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
%
%
% % % Percourse made
% pp1 = plot3(PX(1,:),PX(2,:),PX(3,:)+h,'r-','LineWidth',sizeReal);
% pp2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',sizeReal);
% % title('Posição dos Robôs','fontSize',lt);
% xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
% zlabel('$z$ [m]','interpreter','Latex');
%
% % % Desired positions
% pdd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'k--','LineWidth',sizeDesired); % pioneer
% pdd2 = plot3(x2,y2,z2,'k--','LineWidth',sizeDesired);     % drone
%
% %Formation line
% vec = [1 length(PX)];
% for k = 1:length(vec)
%     x = [PX(1,vec(k))    AX(1,vec(k))];
%     y = [PX(2,vec(k))    AX(2,vec(k))];
%     z = [PX(3,vec(k))+h   AX(3,vec(k))];
%
%     ppl = line(x,y,z);
%     ppl.Color = 'g';
%     ppl.LineStyle = '-';
%     ppl.LineWidth = 1;
% end
%
%
% leg1 = legend([ppl pstart1 pend1 pdd1],{'Formation Line','Start position','Final Position','Desired Trajectory'});
% % lg1 = legend([ps1 pd1],{'Start position','Desired Positions'});
% % lg1 = legend(pl,{'Formation line'});
% %
% leg1.FontSize = sizeLegenda;
% leg1.Location = location;
% set(leg1,'Interpreter','latex');
% % legend('boxoff')
% %
% % Superior View ###################################################################
% figure;
% axis equal
% axis ([-1.5 1.5 -2 2])
% set(gca,'Box','on')
%
% hold on, grid on;
%
% % Initial positions
% pstart1 = plot(PX(1,1),PX(2,1),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% pstart2 = plot(AX(1,1),AX(2,1),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
%
% % % Final positions
% pend1 = plot(PX(1,end),PX(2,end),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% pend2 = plot(AX(1,end),AX(2,end),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
%
%
% % % Percourse made
% preal1 = plot(PX(1,:),PX(2,:),'r-','LineWidth',sizeReal);
% preal2 = plot(AX(1,:),AX(2,:),'b-','LineWidth',sizeReal);
% % title('Posição dos Robôs','fontSize',lt);
% xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
%
% % % Desired positions
% pdd1 = plot(Qd(1,:),Qd(2,:),'k--','LineWidth',sizeDesired); % pioneer
% pdd2 = plot(x2,y2,'k--','LineWidth',sizeDesired);     % drone
%
%
% leg1 = legend([pstart1 pend1 pdd1 preal1 preal2],{'Start position','Final Position','Desired Trajectory','Pioneer3-DX','Bebop'});
% % lg1 = legend([ps1 pd1],{'Start position','Desired Positions'});
% % lg1 = legend(pl,{'Formation line'});
% %
% leg1.FontSize = sizeLegenda;
% leg1.Location = location;
% set(leg1,'Interpreter','latex');
% % legend('boxoff')
% %
%
%
% %
% % % Velocidades ######################################################################
% % % Pioneer
% % figure;
% % plot(time,PUd(1,:),'b--','LineWidth',sizeDesired), hold on;
% % % plot(time,PU(1,:),'r','LineWidth',sizeReal);
% % % legend({'$u_{d}$','$u_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % legend({'$u_{d}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('Velocity [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% %
% % figure;
% % plot(time,PUd(2,:),'b--','LineWidth',sizeDesired), hold on;
% % % legend({'$\omega_{d}$','$\omega_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % legend({'$\omega_{d}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('Velocity [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% %
% % % Bebop######################################################################
% % % phi
% % figure;
% % plot(time,AUd(1,:),'b--','LineWidth',sizeDesired);hold on;
% % % plot(time,AU(1,:),'r','LineWidth',sizeReal);
% % % legend({'$\phi_{d}$','$\phi_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % legend({'$\phi_{d}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('$\phi$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% %
% % % theta
% % figure;
% % plot(time,AUd(2,:),'b--','LineWidth',sizeDesired);hold on;
% % % plot(time,AU(2,:),'r','LineWidth',sizeReal);
% % % legend({'$\theta_{d}$','$\theta_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % legend({'$\theta_{d}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('$\theta$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% %
% % % dZ
% % figure;
% % plot(time,AUd(3,:),'b--','LineWidth',sizeDesired);hold on;
% % % plot(time,AU(3,:),'r','LineWidth',sizeReal);
% % % legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % legend({'$\dot{z}_{d}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('$\dot{z}$ [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% %
% % % dPsi
% % figure;
% % plot(time,AUd(4,:),'b--','LineWidth',sizeDesired);hold on;
% % % plot(time,-AU(4,:),'r','LineWidth',sizeReal);
% % % legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % legend({'$\dot{\psi}_{d}$'},'FontSize',sizeLegenda,'Location',location,'Interpreter','latex');
% % grid on,xlim([0 time(end)]);
% % xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% % ylabel('$\dot{\psi}$ [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% %
%
% % Erro da formação ....................................................................
%
% % Position
% figure;
% plot(time,Qtil(1,:),'b--','LineWidth',sizeReal),hold on;
% plot(time,Qtil(2,:),'r-','LineWidth',sizeReal);
% plot(time,Qtil(4,:),'g-.','LineWidth',sizeReal);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Error [m]','FontSize',sizeLabel,'interpreter','Latex');
% xlim([0 time(end)]);
% % title('(a)','Interpreter','latex');
%
% lgg3 = legend('$x_f$','$y_f$','$\rho_f$');
% lgg3.FontSize = 14;
% lgg3.Location = location;
% set(lgg3,'Interpreter','latex');
% grid on;
%
%
% % Angles
% figure;
% plot(time,Qtil(5,:),'b--','LineWidth',sizeReal),hold on;
% plot(time,Qtil(6,:),'r-','LineWidth',sizeReal);
% xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('Error [rad]','FontSize',sizeLabel,'interpreter','Latex');
% xlim([0 time(end)]);
% % title('(b)','Interpreter','latex');
%
% % lg3 = legend('$x_f [m]$','$y_f [m]$','$z_f [m]$','$\rho_f [m]$','$\alpha_f [rad]$','$\beta_f [rad]$');
% lgg4 = legend('$\alpha_f$','$\beta_f$');
% lgg4.FontSize = 14;
% lgg4.Location = location;
% set(lgg4,'Interpreter','latex');
% grid on