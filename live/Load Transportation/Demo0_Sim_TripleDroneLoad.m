
% Inicialização
close all
clear
clc

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
cd('D:\Dropbox\AuRoRA 2018')
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

gains = [1 2 1 2 2 15;
    2 13 2 15 1 5];

c3 = cos(30*pi/180); c4 = cos(45*pi/180); c6 = cos(60*pi/180);
s3 = sin(30*pi/180); s4 = sin(45*pi/180); s6 = sin(60*pi/180);

%%
% Robot initialization

% Robot initialization
A{1} = ArDrone;
A{2} = ArDrone;
A{3} = ArDrone;

A{1}.pPar.uSat(1:2) = A{1}.pPar.uSat(1:2)*0.75;
A{1}.pPar.uSat(3) = A{1}.pPar.uSat(3)*0.75;
A{1}.pPar.uSat(4) = A{1}.pPar.uSat(4)*0.75;
A{2}.pPar.uSat = A{1}.pPar.uSat;
A{3}.pPar.uSat = A{1}.pPar.uSat;

mCADcolor(A{1},[1 0 0])
mCADcolor(A{2},[0 1 0])
mCADcolor(A{2},[0 0 1])
L{1} = Load;
L{2} = Load;
L{3} = Load;
l = L{1}.pPar.l;
%% Figure
L{1}.pPos.X(1:3) = [0 0 0]; L{2}.pPos.X = L{1}.pPos.X; L{3}.pPos.X = L{1}.pPos.X;
x0 = L{1}.pPos.X(1); y0 = L{1}.pPos.X(2); z0 = L{1}.pPos.X(3);
% A{1}.pPos.X(1:3) = [x0+l*c4*s3 y0+l*c4*s6 z0+l*c4];
% A{2}.pPos.X(1:3) = [x0+l*c4*s3 y0-l*c4*s6 z0+l*c4];
% A{3}.pPos.X(1:3) = [x0-l*c4 y0 z0+l*c4];

A{1}.pPos.X(1:3) = [x0+l*c4*s3 y0+l*c4*s6 0];
A{2}.pPos.X(1:3) = [x0+l*c4*s3 y0-l*c4*s6 0];
A{3}.pPos.X(1:3) = [x0-l*c4 y0            0];

ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[10 10 1200/1.3 800/1.3],'PaperPositionMode','auto')

axis image
ObjF.xlim = [-1 1.0]*3.0;
ObjF.ylim = [-1 1.0]*3.0;
ObjF.zlim = [ 0 2.5];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])

grid on

L{1}.mCADCreate; L{2}.mCADCreate; L{3}.mCADCreate
hold on
A{1}.mCADplot; A{2}.mCADplot; A{3}.mCADplot;
L{1}.mCADPlot(A{1}); L{2}.mCADPlot(A{2}); L{3}.mCADPlot(A{3});
% Linhas entre os VANTs
pd(1) = plot3([A{1}.pPos.X(1) A{2}.pPos.X(1)],[A{1}.pPos.X(2) A{2}.pPos.X(2)],[A{1}.pPos.X(3) A{2}.pPos.X(3)],'--');
pd(2) = plot3([A{1}.pPos.X(1) A{3}.pPos.X(1)],[A{1}.pPos.X(2) A{3}.pPos.X(2)],[A{1}.pPos.X(3) A{3}.pPos.X(3)],'--');
pd(3) = plot3([A{2}.pPos.X(1) A{3}.pPos.X(1)],[A{2}.pPos.X(2) A{3}.pPos.X(2)],[A{2}.pPos.X(3) A{3}.pPos.X(3)],'--');

%%
Xd = [0   0.0    0  0  5.0;
      0   1.0   .5  0  10.0;
      1   1.0   .5  0  15.0;
      1   0.0  1.0  0  20.0;
      0   0.0  1.0  0  25.0
      0   0.0  0.5  0  30.0];

%% Time variables
tsim = Xd(end,end);        % Simulation time [s]
tout = 100;              % maximum simulation duration [s]
tc = tic;                % drone frequency
tp = tic;                % graph refresh rate
tt = tic;                % trajectory time
t = tic;                 % simulation current time
td = tic;
ta = [];
dt = 1/30;
cont = 1;
XX{1} = [];      % position data
XX{2} = [];      % position data
XX{3} = [];      % position data
XX{4} = [];      % position data
%
while toc(t) < tsim
    if toc(tc) > A{1}.pPar.Ts
        ta = [ta toc(tc)];
        tc = tic;
        
        % Desired Position
        if toc(t) > Xd(cont,5)
            cont = cont + 1;
        end
        
        L{1}.pPos.Xd(1:3) = [Xd(cont,1) Xd(cont,2) Xd(cont,3)]; L{2}.pPos.Xd = L{1}.pPos.Xd; L{3}.pPos.Xd = L{1}.pPos.Xd;
        x0 = L{1}.pPos.Xd(1); y0 = L{1}.pPos.Xd(2); z0 = L{1}.pPos.Xd(3);
        A{1}.pPos.Xd(1:3) = [x0+l*c4*s3 y0+l*c4*s6 z0+l*c4];
        A{2}.pPos.Xd(1:3) = [x0+l*c4*s3 y0-l*c4*s6 z0+l*c4];
        A{3}.pPos.Xd(1:3) = [x0-l*c4 y0 z0+l*c4];
        
        % ----------------------------------------------------------
        %  Get current rigid body information from optitrack
        A{1}.rGetSensorData
        A{2}.rGetSensorData
        A{3}.rGetSensorData

        %%
        %Atualizar a carga
        % Distância entre os drones
        d(1) = norm(A{1}.pPos.X(1:3) - A{2}.pPos.X(1:3));
        d(2) = norm(A{1}.pPos.X(1:3) - A{3}.pPos.X(1:3));
        d(3) = norm(A{2}.pPos.X(1:3) - A{3}.pPos.X(1:3));

        beta = [acos((d(1)^2+d(2)^2-d(3)^2)/(2*d(1)*d(2)))  %Ângulos entre os veículos
                acos((d(1)^2-d(2)^2+d(3)^2)/(2*d(1)*d(3)))
                acos((-d(1)^2+d(2)^2+d(3)^2)/(2*d(2)*d(3)))]';

        pf = [(A{1}.pPos.X(1)+A{2}.pPos.X(1)+A{3}.pPos.X(1))/3
              (A{1}.pPos.X(2)+A{2}.pPos.X(2)+A{3}.pPos.X(2))/3];
        
        dAp = [norm(A{1}.pPos.X(1:2)-pf)
               norm(A{2}.pPos.X(1:2)-pf)
               norm(A{3}.pPos.X(1:2)-pf)];
           
        gamma = [acos(dAp(1)/1)
                 acos(dAp(2)/1)
                 acos(dAp(3)/1)];
        
        alpha = [pi/2-beta(2),pi/2-beta(3);  pi/2-beta(1),pi/2-beta(2);  pi/2-beta(1),pi/2-beta(2)];
        alphal = [-pi/2-alpha(1,1) pi/2+alpha(2,1) alpha(3,1)-alpha(3,2)];
        
        alphal = [-120 120 0]*pi/180;        
        gamma = [45 45 45]*pi/180;
        
        G = [0 0 L{1}.pPar.m*L{1}.pPar.g]';
        
        B = [cos(gamma(1))*sin(alphal(1)) cos(gamma(2))*sin(alphal(2)) cos(gamma(3))*sin(alphal(3))
             cos(gamma(1))*cos(alphal(1)) cos(gamma(2))*cos(alphal(2)) cos(gamma(3))*cos(alphal(3))
             sin(gamma(1))                sin(gamma(2))                sin(gamma(3))
            ];
        TT = inv(B)*G;
        
        T(1:3,1) = TT(1)*[cos(gamma(1))*sin(alphal(1))  cos(gamma(1))*cos(alphal(1))  sin(gamma(1))];
        T(1:3,2) = TT(2)*[cos(gamma(2))*sin(alphal(2))  cos(gamma(2))*cos(alphal(2))  sin(gamma(2))];
        T(1:3,3) = TT(3)*[cos(gamma(3))*sin(alphal(3))  cos(gamma(3))*cos(alphal(3))  sin(gamma(3))];
        
        z = 1*sin(gamma(1));
        L{1}.pPos.X(1:3) = [pf(1) pf(2) A{1}.pPos.X(3)-z]; L{2}.pPos.X = L{1}.pPos.X; L{3}.pPos.X = L{1}.pPos.X;

%         T(1:3,1) = [];
        %%
        %         A.pPar.D(3) = 0;
        A{1}.pPar.D(1:3) = -T(1:3,1)/2;
        A{2}.pPar.D(1:3) = T(1:3,2)/2;
        A{3}.pPar.D(1:3) = T(1:3,3)/2;
        
        if L{1}.pPos.X(3) <= 0
        L{1}.pPos.X(3) = 0;
        L{2}.pPos.X(3) = 0;
        L{3}.pPos.X(3) = 0;
        A{1}.pPar.D(1:3) = 0;
        A{2}.pPar.D(1:3) = 0;
        A{3}.pPar.D(1:3) = 0;
        end
        
        
        A{1} = cUnderActuatedControllerMexido(A{1},gains);
        A{2} = cUnderActuatedControllerMexido(A{2},gains);
        A{3} = cUnderActuatedControllerMexido(A{3},gains);
        
        A{1}.rSendControlSignals;
        A{2}.rSendControlSignals;
        A{3}.rSendControlSignals;

        % Save Variables
        XX{1} = [XX{1} [A{1}.pPos.Xd; A{1}.pPos.X; toc(t)]];
        XX{2} = [XX{2} [A{2}.pPos.Xd; A{2}.pPos.X; toc(t)]];
        XX{3} = [XX{3} [A{3}.pPos.Xd; A{3}.pPos.X; toc(t)]];
        XX{4} = [XX{4} [L{1}.pPos.Xd; L{1}.pPos.X; toc(t)]];
    end
%     disp(L{1}.pPos.X(1:3)')
    if toc(tp) > 0.05
        tp = tic;
        A{1}.mCADplot;
        A{2}.mCADplot;
        A{3}.mCADplot;
        %
        L{1}.mCADPlot(A{1});
        L{2}.mCADPlot(A{2});
        L{3}.mCADPlot(A{3});
        
        % Atualiza as linhas entre os drones
        pd(1).XData = [A{1}.pPos.X(1) A{2}.pPos.X(1)]; pd(1).YData = [A{1}.pPos.X(2) A{2}.pPos.X(2)]; pd(1).ZData = [A{1}.pPos.X(3) A{2}.pPos.X(3)];
        pd(2).XData = [A{1}.pPos.X(1) A{3}.pPos.X(1)]; pd(2).YData = [A{1}.pPos.X(2) A{3}.pPos.X(2)]; pd(2).ZData = [A{1}.pPos.X(3) A{3}.pPos.X(3)];
        pd(3).XData = [A{2}.pPos.X(1) A{3}.pPos.X(1)]; pd(3).YData = [A{2}.pPos.X(2) A{3}.pPos.X(2)]; pd(3).ZData = [A{2}.pPos.X(3) A{3}.pPos.X(3)];
        drawnow
    end
end
disp('Fim!')
close all

%% Trajectory 3D
figure(1)
plot3(XX{1}(1,:)', XX{1}(2,:)',XX{1}(3,:)','r--'); hold on
plot3(XX{1}(13,:)',XX{1}(14,:)',XX{1}(15,:)','r-'); 

plot3(XX{2}(1,:)', XX{2}(2,:)', XX{2}(3,:)','b--'); hold on
plot3(XX{2}(13,:)',XX{2}(14,:)',XX{2}(15,:)','b-'); 

plot3(XX{3}(1,:)', XX{3}(2,:)', XX{3}(3,:)','k--','Color',[1 .9 0]); hold on
plot3(XX{3}(13,:)',XX{3}(14,:)',XX{3}(15,:)','-','Color',[1 .9 0]); 

plot3(XX{4}(1,:)', XX{4}(2,:)', XX{4}(3,:)','--','Color',[0 0.45 0.74]); hold on
plot3(XX{4}(13,:)',XX{4}(14,:)',XX{4}(15,:)','-','Color',[0 0.45 0.74]);
grid on
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
legend('Ardrone_{1D}','Ardrone_1','Ardrone_{2D}','Ardrone_2','Ardrone_{3D}','Ardrone_3','Load_{D}','Load')
%% X and Y
figure(2)

a(1) = subplot(411);plot(XX{1}(end,:)',XX{1}(1,:)','k--'); hold on;
a(2) = subplot(411);plot(XX{1}(end,:)',XX{1}(13,:)','r-');
grid on
a(1).YLim = [0 2];
legend('Ardrone_{1D}','Ardrone_1','Location','southeast')
ylabel('x [m]')

a(3) = subplot(412);plot(XX{2}(end,:)',XX{2}(1,:)','k--'); hold on;
a(4) = subplot(412);plot(XX{2}(end,:)',XX{2}(13,:)','b-');
grid on
a(3).YLim = [0 2];
legend('Ardrone_{2D}','Ardrone_2','Location','southeast')
ylabel('x [m]')

a(5) = subplot(413);plot(XX{3}(end,:)',XX{3}(1,:)','k--'); hold on;
a(6) = subplot(413);plot(XX{3}(end,:)',XX{3}(13,:)','-','Color',[1 .9 0]); 
grid on
a(5).YLim = [-1 1];
legend('Ardrone_{3D}','Ardrone_3','Location','southeast')
ylabel('x [m]')

a(7) = subplot(414);plot(XX{4}(end,:)',XX{3}(1,:)','k--');
a(8) = subplot(414);plot(XX{4}(end,:)',XX{3}(13,:)','-','Color',[0 0.45 0.74]);  
grid on
a(7).YLim = [-1 1];
legend('Load_{D}','Load','Location','southeast')
ylabel('x [m]')
xlabel('t [s]')
%
figure(3)

a(1) = subplot(411);plot(XX{1}(end,:)',XX{1}(2,:)','k--'); hold on;
a(2) = subplot(411);plot(XX{1}(end,:)',XX{1}(14,:)','r-');
grid on
a(1).YLim = [0 2];
legend('Ardrone_{1D}','Ardrone_1','Location','southeast')
ylabel('y [m]')

a(3) = subplot(412);plot(XX{2}(end,:)',XX{2}(2,:)','k--'); hold on;
a(4) = subplot(412);plot(XX{2}(end,:)',XX{2}(14,:)','b-');
grid on
a(3).YLim = [-1 1];
legend('Ardrone_{2D}','Ardrone_2','Location','southeast')
ylabel('y [m]')

a(5) = subplot(413);plot(XX{3}(end,:)',XX{3}(2,:)','k--'); hold on;
a(6) = subplot(413);plot(XX{3}(end,:)',XX{3}(14,:)','-','Color',[1 .9 0]); 
grid on
a(5).YLim = [-1 1];
legend('Ardrone_{3D}','Ardrone_3','Location','southeast')
ylabel('y [m]')

a(7) = subplot(414);plot(XX{4}(end,:)',XX{3}(2,:)','k--'); hold on;
a(8) = subplot(414);plot(XX{4}(end,:)',XX{3}(14,:)','-'); 
grid on
a(7).YLim = [-1 1];
legend('Load_{D}','Load','Location','southeast')
ylabel('y [m]')
xlabel('t [s]')

figure(4)

a(1) = subplot(411);plot(XX{1}(end,:)',XX{1}(3,:)','k--'); hold on;
a(2) = subplot(411);plot(XX{1}(end,:)',XX{1}(15,:)','r-');
grid on
a(1).YLim = [0 2];
legend('Ardrone_{1D}','Ardrone_1','Location','southeast')
ylabel('z [m]')

a(3) = subplot(412);plot(XX{2}(end,:)',XX{2}(3,:)','k--'); hold on;
a(4) = subplot(412);plot(XX{2}(end,:)',XX{2}(15,:)','b-');
grid on
a(3).YLim = [0 2];
legend('Ardrone_{2D}','Ardrone_2','Location','southeast')
ylabel('z [m]')

a(5) = subplot(413);plot(XX{3}(end,:)',XX{3}(3,:)','k--'); hold on;
a(6) = subplot(413);plot(XX{3}(end,:)',XX{3}(15,:)','-','Color',[1 .9 0]); 
grid on
a(5).YLim = [0 2];
legend('Ardrone_{3D}','Ardrone_3','Location','southeast')
ylabel('z [m]')

a(7) = subplot(414);plot(XX{4}(end,:)',XX{4}(3,:)','k--'); hold on;
a(8) = subplot(414);plot(XX{4}(end,:)',XX{4}(15,:)','-'); 
grid on
a(7).YLim = [0 2];
legend('Load_{D}','Load','Location','southeast')
ylabel('z [m]')
xlabel('t [s]')

%%
figure(5)

a(1) = subplot(311);plot(XX{1}(end,:)',XX{1}(4,:)'*180/pi,'k--'); hold on;
a(2) = subplot(311);plot(XX{1}(end,:)',XX{1}(16,:)'*180/pi,'r-');
grid on
legend('Ardrone_{1D}','Ardrone_1','Location','southeast')
ylabel('\phi [^o]')
a(1).YLim = [-10 10];

a(1) = subplot(312);plot(XX{2}(end,:)',XX{2}(4,:)'*180/pi,'k--'); hold on;
a(2) = subplot(312);plot(XX{2}(end,:)',XX{2}(16,:)'*180/pi,'b-');
grid on
legend('Ardrone_{2D}','Ardrone_2','Location','southeast')
ylabel('\phi [^o]')
a(1).YLim = [-10 10];

a(1) = subplot(313);plot(XX{3}(end,:)',XX{3}(4,:)'*180/pi,'k--'); hold on;
a(2) = subplot(313);plot(XX{3}(end,:)',XX{3}(16,:)'*180/pi,'-','Color',[1 .9 0]); 
grid on
legend('Ardrone_{3D}','Ardrone_3','Location','southeast')
ylabel('\phi [^o]')
xlabel('t [s]')
a(1).YLim = [-10 10];

figure(6)

a(1) = subplot(311);plot(XX{1}(end,:)',XX{1}(5,:)'*180/pi,'k--'); hold on;
a(2) = subplot(311);plot(XX{1}(end,:)',XX{1}(17,:)'*180/pi,'r-');
grid on
legend('Ardrone_{1D}','Ardrone_1','Location','southeast')
ylabel('\theta [^o]')
a(1).YLim = [-10 10];

a(1) = subplot(312);plot(XX{2}(end,:)',XX{2}(5,:)'*180/pi,'k--'); hold on;
a(2) = subplot(312);plot(XX{2}(end,:)',XX{2}(17,:)'*180/pi,'b-');
grid on
legend('Ardrone_{2D}','Ardrone_2','Location','southeast')
ylabel('\theta [^o]')
a(1).YLim = [-10 10];

a(1) = subplot(313);plot(XX{3}(end,:)',XX{3}(5,:)'*180/pi,'k--'); hold on;
a(2) = subplot(313);plot(XX{3}(end,:)',XX{3}(17,:)'*180/pi,'-','Color',[1 .9 0]); 
grid on
legend('Ardrone_{3D}','Ardrone_3','Location','southeast')
ylabel('\theta [^o]')
xlabel('t [s]')
a(1).YLim = [-10 10];

%%
salvar = false;
if salvar == true
    %%
    cd('D:\Dropbox\!Ufes\Artigos\2019 - ICUAS - Pizetta - 3 Drones\figures')
    
    identf = 'Coord';
    strin{1} = [identf, '_3D'];
    strin{2} = [identf, '_X'];
    strin{3} = [identf, '_Y'];
    strin{4} = [identf, '_Z'];
    strin{5} = [identf, '_phi'];
    strin{6} = [identf, '_theta'];
    
    for idx = 1:6
        figure(idx)
        saveas(gca,strin{idx},'epsc')
        disp(['File ',strin{idx},'.eps Saved']);
    end
end
disp('!!! Fim !!!')
