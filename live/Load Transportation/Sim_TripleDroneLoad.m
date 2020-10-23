%% Simulação de Voo Cooperativo com carga em 3 dimensões

% Inicialização
close all; clear; clc

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Drones initialization
L = Load; L.pPar.m = 0.2;
A{1} = ArDrone; mCADcolor(A{1},[1 0 0])
A{2} = ArDrone; mCADcolor(A{2},[0 1 0])
A{3} = ArDrone; mCADcolor(A{2},[0 0 1])
C{1} = Cable;
C{2} = Cable;
C{3} = Cable;

% A{1}.pPar.uSat(1:2) = A{1}.pPar.uSat(1:2)*2.0;
% A{1}.pPar.uSat(3)   = A{1}.pPar.uSat(3)*1.0;
% A{1}.pPar.uSat(4)   = A{1}.pPar.uSat(4)*1.0;
% A{2}.pPar.uSat      = A{1}.pPar.uSat;
% A{3}.pPar.uSat      = A{1}.pPar.uSat;

A{1}.pPar.D = [0 0 0 0 0 0]';
A{2}.pPar.D = [0 0 0 0 0 0]';
A{3}.pPar.D = [0 0 0 0 0 0]';

gains = [1.2 2.6 1.2 2.6 2.2 15;
         2 15 2 15 1 10];
     

c3 = cos(30*pi/180);
c4 = cos(45*pi/180);
c6 = cos(60*pi/180);

s3 = sin(30*pi/180);
s4 = sin(45*pi/180);
s6 = sin(60*pi/180);

L.pPos.X(1:3) = [0 0 0];
A{1}.pPos.X(1:3) = [L.pPos.X(1)+L.pPar.l*c4*s3 L.pPos.X(2)+L.pPar.l*c4*s6 L.pPos.X(3)+L.pPar.l*c4];
A{2}.pPos.X(1:3) = [L.pPos.X(1)+L.pPar.l*c4*s3 L.pPos.X(2)-L.pPar.l*c4*s6 L.pPos.X(3)+L.pPar.l*c4];
A{3}.pPos.X(1:3) = [L.pPos.X(1)-L.pPar.l*c4 L.pPos.X(2) L.pPos.X(3)+L.pPar.l*c4];


%% Open file to save data
% OptData = CreateOptDataFile(A{1},A{2},L,'DesvioObs','Log_Optitrack');
% 
%%
Xd = [0    0  1.3  0  10.0;
      1    0  1.3  0  15.0
      1    1  1.3  0  20.0
      0    1  1.3  0  25.0
      0    0  1.3  0  30.0
      ];
  


%% Create Figure
ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')
grid on 
axis image
ObjF.xlim = [-5 5]; ObjF.ylim = [-3 3]; ObjF.zlim = [ 0 2.5];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])
L.mCADCreate;
C{1}.mCADCreate(L,A{1});
C{2}.mCADCreate(L,A{2});
C{3}.mCADCreate(L,A{2});

%% Time variables

time = 15;  % target hover time  [s]  
tsim = Xd(end,end);    % Simulation time [s]
cont = 1;     % counter
angulos = [];
XX = [];      % position data
tout = 100;   % maximum simulation duration [s]
tc = tic;     % drone frequency
tp = tic;     % graph refresh rate
tt = tic;     % trajectory time
t = tic;      % simulation current time
td = tic;
ta = [];
dt = 1/30;

%% Simulation loop
while toc(t) < tsim
    if toc(tc) > A{1}.pPar.Ts
        ta = [ta toc(tc)];
        tc = tic;
        
        % Desired Position
        if toc(t) > Xd(cont,5)
            cont = cont + 1;
        end
        
        A{1}.pPos.Xd(1:3) = [Xd(cont,1)+L.pPar.l*c4*s3 Xd(cont,2)+L.pPar.l*c4*s6    Xd(cont,3)+L.pPar.l*c4];
        A{2}.pPos.Xd(1:3) = [Xd(cont,1)+L.pPar.l*c4*s3 Xd(cont,2)-L.pPar.l*c4*s6    Xd(cont,3)+L.pPar.l*c4];
        A{3}.pPos.Xd(1:3) = [Xd(cont,1)-L.pPar.l*c4    Xd(cont,2)                   Xd(cont,3)+L.pPar.l*c4];
        % ----------------------------------------------------------

        % Controlador
        A{1}.rGetSensorData;
        A{2}.rGetSensorData;
        A{3}.rGetSensorData;
        
        % ----------------------------------------------------------
        
        
        for nv = 1:2
            C{nv}.pPos.Xa = C{nv}.pPos.X;
            C{nv}.pPos.X(1) = atan(norm(norm(A{1}.pPos.X(1)- L.pPos.X(1)))/norm(A{1}.pPos.X(2)- L.pPos.X(2)));
            C{nv}.pPos.X(2) = abs(atan2(A{nv}.pPos.X(1)-L.pPos.X(1),A{nv}.pPos.X(3)-L.pPos.X(3)));
            C{nv}.pPos.X(4) = norm(A{nv}.pPos.X(1:3)-L.pPos.X( 1:3));
        end
        
        if L.pPos.X(3) < .02
            C{1}.pPos.X(3) = 0;
            C{2}.pPos.X(3) = 0;
        else
            C{1}.pPos.X(3) = L.pPar.m*L.pPar.g*sin(C{2}.pPos.X(2))/(sin(C{1}.pPos.X(2)+C{2}.pPos.X(2)));
            C{2}.pPos.X(3) = L.pPar.m*L.pPar.g*sin(C{1}.pPos.X(2))/(sin(C{1}.pPos.X(2)+C{2}.pPos.X(2)));
        end
        
        Cabo1 = [C{1}.pPos.X(3)*sin(C{1}.pPos.X(2))*cos(C{1}.pPos.X(1));
            -C{1}.pPos.X(3)*sin(C{1}.pPos.X(2))*sin(C{1}.pPos.X(1));
            C{1}.pPos.X(3)*cos(C{1}.pPos.X(2))];
        
        Cabo2 = [C{2}.pPos.X(3)*sin(C{2}.pPos.X(2))*cos(C{2}.pPos.X(1));
            -C{2}.pPos.X(3)*sin(C{2}.pPos.X(2))*sin(C{2}.pPos.X(1));
            C{2}.pPos.X(3)*cos(C{2}.pPos.X(2))];
        
        Cabo1(3) = 0;
        Cabo2(3) = 0;
        
        %         Repulsion = sRepulsionField(A{1},A{2});
        %         A{1}.pPar.D(4:6) =  -Repulsion';
        %         A{2}.pPar.D(4:6) =  Repulsion';
        
        %         A{1}.pPar.D(4:6) =  Cabo1;
        %         A{2}.pPar.D(4:6) =  Cabo2;
        %
        A{1}.pPar.D(1:3) = Cabo1;
        A{2}.pPar.D(1:3) = Cabo2;
        
        
        
        
        
        % ----------------------------------------------------------
        
        
        
        
        
        
        
        
        
        
        
        
        A{1} = cUnderActuatedControllerMexido(A{1},gains);
        A{2} = cUnderActuatedControllerMexido(A{2},gains);
        A{3} = cUnderActuatedControllerMexido(A{3},gains);
        
        A{1}.rSendControlSignals;
        A{2}.rSendControlSignals;
        A{3}.rSendControlSignals;
        % Save data ------------------------------------------
        XX = [XX [A{1}.pPos.Xd; A{1}.pPos.X; toc(t)]];
%         SaveOptData(OptData,A,L ,toc(t));
        % ----------------------------------------------------------------
        
        if toc(tp) > 0.05
            tp = tic;
            A{1}.mCADplot;
            A{2}.mCADplot;
            A{3}.mCADplot;
            L.mCADPlot(A{1});
            C{1}.mCADPlot(L,A{1});
            C{2}.mCADPlot(L,A{2});
            C{3}.mCADPlot(L,A{3});
            drawnow
        end
        
    end

end
%% Close Data Files
% CloseOptDataFile(OptData);


%% Plot Results
% Roll and pitch angles
figure(1)
subplot(311),plot(XX(end,:),XX([4 16],:)'*180/pi); grid;
subplot(312),plot(XX(end,:),XX([5 17],:)'*180/pi); grid;
subplot(313),plot(XX(end,:),XX([6 18],:)'*180/pi); grid;
legend('\psi_{Des}[^o]','\psi_{Atu}[^o]')

% Trajectory 3D
figure(3)
plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)'); grid
%% X and Y
figure(4)
subplot(311),plot(XX(end,:),XX([1 13],:)'); grid
subplot(312),plot(XX(end,:),XX([2 14],:)'); grid
subplot(313),plot(XX(end,:),XX([3 15],:)'); grid
legend('z_{Des}','z_{Atu}','z_{Load}')
%%
% velocities
figure(5)
subplot(311),plot(XX(end,:),XX([7 19],:)'); grid;
axis([0 XX(end,end) -1 1])
subplot(312),plot(XX(end,:),XX([8 20],:)'); grid;
axis( [0 XX(end,end) -1 1])
subplot(313),plot(XX(end,:),XX([9 21],:)'); grid;
legend('dz_{Des}','dz_{Atu}')
axis([0 XX(end,end) -1 1])

disp('!!! Fim !!!')