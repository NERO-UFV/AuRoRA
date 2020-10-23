function plotResults(data)
%% Rotina para plotar gráficos de Formacaolinha3D
close all
%% Declara os robos
P = Pioneer3DX;
A = ArDrone;
% LF = LineFormation3D;

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

% Robots desired positions
for kk = 1:length(Qd)
xf   = Qd(1,kk);  yf    = Qd(2,kk);   zf = Qd(3,kk);
rhof = Qd(4,kk);  alfaf = Qd(5,kk);   betaf = Qd(6,kk);

x2(kk) = xf + rhof*cos(alfaf)*cos(betaf);
y2(kk) = yf + rhof*sin(alfaf)*cos(betaf);
z2(kk) = zf + rhof*sin(betaf);
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
sl= 1; %'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

% Posição dos robôs
% Parameters
scale  = 1;     % robot model plot scale
Pcolor = 'k';   % robot A color
step   = 200;   % model plot step
h      = 0.1;   % line height

figure(1);
axis equal
axis ([-4 4 -4 4 0 4])
set(gca,'Box','on')

% ax = axis;
% set(gca,'xticklabel',[0 1])
% set(gca,'yticklabel',[])
% set(gca,'zticklabel',[])
% set(gca,'xticklabel',[])

hold on, grid on;

% Initial positions
ps1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'c^','MarkerSize',5,'LineWidth',3);
ps2 = plot3(AX(1,1),AX(2,1),AX(3,1),'c^','MarkerSize',5,'LineWidth',3);

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
pd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'k.','MarkerSize',20,'LineWidth',2);hold on % pioneer
pd2 = plot3(x2,y2,z2,'k.','MarkerSize',20,'LineWidth',2);     % drone

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
subplot(121), plot(time,PUd(1,:),'--','LineWidth',sl), hold on;
plot(time,PU(1,:),'LineWidth',sl);
legend({'$u_{d}$','$u_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('Velocity [m/s]','interpreter','Latex');
title('(a)','Interpreter','latex');
subplot(122), plot(time,PUd(2,:),'--','LineWidth',sl), hold on;
plot(time,PU(2,:),'LineWidth',sl)
legend({'$\omega_{d}$','$\omega_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('Velocity [rad/s]','interpreter','Latex');
title('(b)','Interpreter','latex');

% ArDrone
figure;
subplot(221),plot(time,AUd(1,:),'--','LineWidth',sl);hold on;
plot(time,AU(1,:),'LineWidth',sl);
legend({'$\phi_{d}$','$\phi_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\phi$ [rad]','interpreter','Latex');
title('(a)','Interpreter','latex');

subplot(222),plot(time,AUd(2,:),'--','LineWidth',sl);hold on;
plot(time,AU(2,:),'LineWidth',sl);
legend({'$\theta_{d}$','$\theta_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\theta$ [rad]','interpreter','Latex');
title('(b)','Interpreter','latex');

subplot(223),plot(time,AUd(3,:),'--','LineWidth',sl);hold on;
plot(time,AU(3,:),'LineWidth',sl);
legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{z}$ [m/s]','interpreter','Latex');
title('(c)','Interpreter','latex');

subplot(224),plot(time,AUd(4,:),'--','LineWidth',sl);hold on;
plot(time,AU(4,:),'LineWidth',sl);
legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{\psi}$ [rad/s]','interpreter','Latex');
title('(d)','Interpreter','latex');


% % % Salva a imagem
% % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velLinear.png');
%
%
% Erro da formação ....................................................................
figure;
% Position
subplot(211),plot(time,Qtil(1,:),'b--','LineWidth',sl),hold on;
plot(time,Qtil(2,:),'r-','LineWidth',sl);
plot(time,Qtil(4,:),'g-.','LineWidth',.5);
xlabel('Time [s]','interpreter','Latex'),ylabel('Error [m]','interpreter','Latex');
xlim([0 time(end)]);
title('(a)','Interpreter','latex');

lg3 = legend('$x_f$','$y_f$','$\rho_f$');
lg3.FontSize = 10;
lg3.Location = 'NorthEast';
set(lg3,'Interpreter','latex');
grid on;


% Angles
subplot(212),plot(time,Qtil(5,:),'b--','LineWidth',sl),hold on;
plot(time,Qtil(6,:),'r-','LineWidth',sl);
xlabel('Time [s]','interpreter','Latex'),ylabel('Error [rad]','interpreter','Latex');
xlim([0 time(end)]);
title('(b)','Interpreter','latex');

% lg3 = legend('$x_f [m]$','$y_f [m]$','$z_f [m]$','$\rho_f [m]$','$\alpha_f [rad]$','$\beta_f [rad]$');
lg4 = legend('$\alpha_f$','$\beta_f$');
lg4.FontSize = 10;
lg4.Location = 'NorthEast';
set(lg4,'Interpreter','latex');
grid on
%
% % % Salva a imagem
% % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\Erroformacao.png');
%
%%
% % % Sinal de controle - velocidade linear
% % figure;
% % hold on, grid on;
% % plot(time,Ud(:,1),'LineWidth',sl);
% % plot(time,AUd(:,1),'LineWidth',sl);
% % title('Sinal de controle (Velocidade Linear)','fontSize',st);
% % xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [m/s]','interpreter','Latex');
% % legend('Robô 1', 'Robô 2');
%
%
% % Velocidades Angulares
% figure;
% hold on, grid on;
% plot(time,U(:,2),'r','LineWidth',sl);
% plot(time,AU(:,2),'b','LineWidth',sl);
% % title('Velocidade Angular','fontSize',st);
% xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Speed$ [rad/s]','interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
% xlim([0 time(end)]);
%
% % % Salva a imagem
% % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velAngular.png');
%
%
% % % Sinal de controle - velocidade angular
% % figure;
% % hold on, grid on;
% % plot(time,Ud(:,2),'LineWidth',sl);
% % plot(time,AUd(:,2),'LineWidth',sl);
% % title('Sinal de controle (Velocidade Angular)','fontSize',st);
% % xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [rad/s]','interpreter','Latex');
% % legend('Robô 1', 'Robô 2');
%
% %% Erros
% % % Erro de posição
% % figure;
% % hold on, grid on;
% % plot(time,Xtil(:,[1 2]),'LineWidth',sl);
% % plot(time,AXtil(:,[1 2]),'LineWidth',sl);
% % % title('Erro de posição','fontSize',st);
% % xlabel('$Time$ [s]','interpreter','Latex'),ylabel('Error [m]','interpreter','Latex');
% % xlim([0 time(end)]);
% %
% % lg2 = legend('$x_{robot 1}$','$y_{robot 1}$', '$x_{robot 2}$','$y_{robot 2}$');
% % lg2.FontSize = 12;
% % lg2.Location = 'SouthEast';
% % set(lg2,'Interpreter','latex');
% %
% % % % Salva a imagem
% % % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\ErroPosicao.png');
%
