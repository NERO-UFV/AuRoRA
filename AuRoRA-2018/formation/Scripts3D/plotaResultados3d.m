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
P = Pioneer3DX;
A = ArDrone;
LF = LineFormation3D;

%% Carrega dados
data = load('FL3dSIMU_20180925T223717.txt');
% data = load('FL3dSIMU_20180925T213855.txt');
% data = load('FL3d_TrajExp20181002T185652.txt');
% data = load('FL3dSIMU_20180926T143244.txt');

%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Pioneer data
PXd   = data(:,(1:12))';       % desired pose
PX    = data(:,12+(1:12))';    % real pose
PUd   = data(:,24+(1:2))';     % control signal
PU    = data(:,26+(1:2))';     % velocidades do robô
PXtil = PXd - PX; % erro de postura

% Drone data
AXd   = data(:,28+(1:12))';
AX    = data(:,40+(1:12))';
AUd   = data(:,52+(1:4))';
AU    = data(:,56+(1:4))';
AXtil = AXd - AX;

% Formation data
Qd   = data(:,60+(1:6))';   % desired formation
Qtil = data(:,66+(1:6))';   % formation error

% % Alpha quadrant adjust
% a = 1;
% for ii = 1:length(Qtil)-1
%     angle = Qtil(5,ii+1) - Qtil(5,ii);
%     if abs(angle)<6.5  && abs(angle) > 6  % detect breaks
%         ind(a) = ii;   % save break vector indice
%         a = a+1;
%     end
% end      % detect breaks
% 
% for f = 1:2:length(ind)-1 
%     for jj = ind(f):ind(f+1)-1
%         if Qtil(5,jj+1)>0      % -pi --> +pi
%             Qtil(5,jj+1) = Qtil(5,jj+1) - 2*pi;
%         elseif Qtil(5,jj+1)<0  % +pi --> -pi
%             Qtil(5,jj+1) = Qtil(5,jj+1) + 2*pi;
%         end
%     end
% end  % do the thing

% Robots desired positions
for kk = 1:length(Qd)
xf   = Qd(1,kk);  yf    = Qd(2,kk);   zf = Qd(3,kk);
rhof = Qd(4,kk);  alfaf = Qd(5,kk);   betaf = Qd(6,kk);

x2(kk) = xf + rhof*cos(alfaf)*cos(betaf);
y2(kk) = yf + rhof*sin(alfaf)*cos(betaf);
z2(kk) = zf + rhof*sin(betaf);
end

for k=1:length(Qd)
    if abs(Qd(5,k)) > pi
        if Qd(5,k) > 0
            Qd(5,k) = Qd(5,k) - 2*pi;
        else
            Qd(5,k) = Qd(5,k) + 2*pi;
        end
    end
   
     if abs(Qtil(5,k)) > pi
        if Qtil(5,k) > 0
            Qtil(5,k) = Qtil(5,k) - 2*pi;
        else
            Qtil(5,k) = Qtil(5,k) + 2*pi;
        end
    end
end


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
sizeLabel   = 18;     % tamanho da fonte dos labels
sizeSymbol  = [10 2];  % [tamanho dos símbolos   espessura da linha]
sizeLegenda = 18;     % tamanho das legendas
location    = 'SouthEast'; %'NorthEast';
% Posição dos robôs

% Posição dos robôs
% Parameters
scale  = 1;     % robot model plot scale
Pcolor = 'k';   % robot A color
step   = 200;   % model plot step
h      = 0.1;   % line height

figure(1);
axis equal
axis ([-3 3 -3 3 0 3])
set(gca,'Box','on')

% ax = axis;
% set(gca,'xticklabel',[0 1])
% set(gca,'yticklabel',[])
% set(gca,'zticklabel',[])
% set(gca,'xticklabel',[])
% pause(3)
hold on, grid on;

% Initial positions
ps1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'c^','MarkerSize',3,'LineWidth',2.5);
ps2 = plot3(AX(1,1),AX(2,1),AX(3,1),'c^','MarkerSize',3,'LineWidth',2.5);

% % Final positions
% ps1 = plot3(PX(1,end),PX(2,end),PX(3,end)+h,'ksq','MarkerSize',10,'LineWidth',3);
% ps2 = plot3(AX(1,end),AX(2,end),AX(3,end),'ksq','MarkerSize',10,'LineWidth',3);


% % Percourse made
% p1 = plot3(PX(1,:),PX(2,:),PX(3,:)+h,'r-','LineWidth',0.5);
% p2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',0.5);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','interpreter','Latex');

% % Desired positions
pd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'k.','MarkerSize',15,'LineWidth',2);hold on % pioneer
pd2 = plot3(x2,y2,z2,'k.','MarkerSize',15,'LineWidth',2);     % drone

% % %Formation line
% % vec = [1 length(PX)];
% %   for k = 1:length(vec)
% %     x = [PX(1,vec(k))    AX(1,vec(k))];
% %     y = [PX(2,vec(k))    AX(2,vec(k))];
% %     z = [PX(3,vec(k))+h   AX(3,vec(k))];
% % 
% %     pl = line(x,y,z);
% %     pl.Color = 'g';
% %     pl.LineStyle = '-';
% %     pl.LineWidth = 1;
% %   end


% plot robots and formation lines
for k = 1:step:length(time)
    % Pioneer center position
    P.pPos.Xc([1 2 6]) = PX([1 2 6],k) - ...
        [P.pPar.a*cos(PX(6,k)); P.pPar.a*sin(PX(6,k)); 0];
    % ArDrone position
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
    
    % plot formation line
    x = [PX(1,k)    AX(1,k)];
    y = [PX(2,k)    AX(2,k)];
    z = [PX(3,k)+h   AX(3,k)];
    
    pl = line(x,y,z);
    pl.Color = 'g';
    pl.LineStyle = '-';
    pl.LineWidth = 0.5;
    
    
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
fig1 = plot3(PX(1,1:k),PX(2,1:k),PX(3,1:k)+h,'r-','LineWidth',0.8); hold on;
fig2 = plot3(AX(1,1:k),AX(2,1:k),AX(3,1:k),'b-','LineWidth',0.8);


    pause(.1)
end
%
lg1 = legend([pl ps1 pd1],{'Formation Line','Start position','Desired Positions'});
% lg1 = legend([ps1 pd1],{'Start position','Desired Positions'});
% lg1 = legend(pl,{'Formation line'});
%
lg1.FontSize = 11;
lg1.Location = 'SouthEast';
set(lg1,'Interpreter','latex');
% legend('boxoff')
%
% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\posicao.png');


% Velocidades ...............................................................
% Pioneer
figure;
 subplot(321)
plot(time,PUd(1,:),'b--','LineWidth',sizeDesired), hold on;
plot(time,PU(1,:),'r','LineWidth',sizeReal);
% legend({'$u_{d}$','$u_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Linear Velocity [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% title('(a)','Interpreter','latex');

 subplot(322),
plot(time,PUd(2,:),'b--','LineWidth',sizeDesired), hold on;
plot(time,PU(2,:),'r','LineWidth',sizeReal)
% legend({'$\omega_{d}$','$\omega_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Angular Velocity [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% title('(b)','Interpreter','latex');

% ArDrone
% figure;
 subplot(323)
plot(time,AUd(1,:),'b--','LineWidth',sizeDesired);hold on;
plot(time,AU(1,:),'r','LineWidth',sizeReal);
% legend({'$\phi_{d}$','$\phi_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Roll [rad]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\phi$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% title('(a)','Interpreter','latex');

 subplot(324)
plot(time,AUd(2,:),'b--','LineWidth',sizeDesired);hold on;
plot(time,AU(2,:),'r','LineWidth',sizeReal);
% legend({'$\theta_{d}$','$\theta_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Pitch [rad]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\theta$ [rad]','FontSize',sizeLabel,'interpreter','Latex');
% title('(b)','Interpreter','latex');

subplot(325)
plot(time,AUd(3,:),'b--','LineWidth',sizeDesired);hold on;
plot(time,AU(3,:),'r','LineWidth',sizeReal);
% legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Altitude rate [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\dot{z}$ [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% title('(c)','Interpreter','latex');

subplot(326)
plot(time,AUd(4,:),'b--','LineWidth',sizeDesired);hold on;
plot(time,AU(4,:),'r','LineWidth',sizeReal);
% legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Heading rate [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$\dot{\psi}$ [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% title('(d)','Interpreter','latex');
legend({'Desired','Real'},'FontSize',sizeLegenda,'Location','Southeast','Interpreter','latex');


% Erro da formação ....................................................................

% Position
figure;
subplot(211)
plot(time,Qtil(1,:),'b--','LineWidth',sizeReal),hold on;
plot(time,Qtil(2,:),'r-','LineWidth',sizeReal);
plot(time,Qtil(4,:),'g-.','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Error [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
% title('(a)','Interpreter','latex');

lg3 = legend('$x_f$','$y_f$','$\rho_f$');
lg3.FontSize = 14;
lg3.Location = 'SouthEast';
set(lg3,'Interpreter','latex');
grid on;


% Angles
% ##################################################
% %% ajuste de shattering na marra para arquivo FL3dSIMU_20180925T223717.txt
% for jj = 2165:2278
%     if Qtil(5,jj+1)>0      % -pi --> +pi
%         Qtil(5,jj+1) = Qtil(5,jj+1) - 2*pi;
%     elseif Qtil(5,jj+1)<0  % +pi --> -pi
%         Qtil(5,jj+1) = Qtil(5,jj+1) + 2*pi;
%     end
% end
% 
% for jj = 3579:length(Qtil)-1
%     if Qtil(5,jj+1)>0      % -pi --> +pi
%         Qtil(5,jj+1) = Qtil(5,jj+1) - 2*pi;
%     elseif Qtil(5,jj+1)<0  % +pi --> -pi
%         Qtil(5,jj+1) = Qtil(5,jj+1) + 2*pi;
%     end
% end
% 
% for jj = 4690:length(Qtil)-1
%     Qtil(5,jj+1) = Qtil(5,jj+1) + 2*pi;
%     
% end
% #################################################
% figure;
subplot(212)
plot(time,Qtil(5,:),'b--','LineWidth',sizeReal),hold on;
plot(time,Qtil(6,:),'r-','LineWidth',sizeReal);
xlabel('Time [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Error [rad]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
% title('(b)','Interpreter','latex');

% lg3 = legend('$x_f [m]$','$y_f [m]$','$z_f [m]$','$\rho_f [m]$','$\alpha_f [rad]$','$\beta_f [rad]$');
lg4 = legend('$\alpha_f$','$\beta_f$');
lg4.FontSize = 14;
lg4.Location = 'SouthEast';
set(lg4,'Interpreter','latex');
grid on