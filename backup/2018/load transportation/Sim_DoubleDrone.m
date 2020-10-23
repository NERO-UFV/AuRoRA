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
A{1} = ArDrone;
A{2} = ArDrone;

A{1}.pPos.X(1:3) = [.5 0 0.015]';
A{2}.pPos.X(1:3) = [-.5 0 0.015]';


A{1}.pPar.uSat(1:2) = A{1}.pPar.uSat(1:2)*2.0;
A{1}.pPar.uSat(3)   = A{1}.pPar.uSat(3)*1.0;
A{1}.pPar.uSat(4)   = A{1}.pPar.uSat(4)*1.0;
A{2}.pPar.uSat      = A{1}.pPar.uSat;

A{1}.pPar.D = [0 0 0 0 0 0]';
A{2}.pPar.D = [0 0 0 0 0 0]';

gains = [1.2 2.6 1.2 2.6 2.2 15;
         2 15 2 15 1 10];
     
% Load initialization
L = Load;
L.pPos.X(1:3) = [0 0 0];
L.pPos.X(4:6) = [0 0 0];
L.pPar.m = 0.05;

% Cables
C{1} = Cable;
C{2} = Cable;


%% Open file to save data
% OptData = CreateOptDataFile(A{1},A{2},L,'DesvioObs','Log_Optitrack');
% 
%%
Xd = [0   0.0   1.3  0  10.0;
      0   0.0   1.3  0  30.0];

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

%% Time variables

time = 15;  % target hover time  [s]  
tsim = 40;    % Simulation time [s]
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
        if toc(t) > 30
            A{1}.pPos.Xd(1:3,1) = [0 0 1.5]; else
            if toc(t) > 20
                A{1}.pPos.Xd(1:3,1) = [1 1 1.5]; else
                if toc(t) > 10
                    A{1}.pPos.Xd(1:3,1) = [1 0 1.5];  else
                    A{1}.pPos.Xd(1:3,1) = [0 0 1.5];
                end
            end
        end
        A{2}.pPos.Xd(1:3,1) =  A{1}.pPos.Xd(1:3) - [1 0 0]';
        % ----------------------------------------------------------

        % Controlador
        A{1}.rGetSensorData;
        A{2}.rGetSensorData;
        
        A{1} = cUnderActuatedControllerMexido(A{1},gains);
        A{2} = cUnderActuatedControllerMexido(A{2},gains);
        A{1}.rSendControlSignals;
        A{2}.rSendControlSignals;
        % Save data ------------------------------------------
%         XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; L.pPos.X; toc(t)]];
%         SaveOptData(OptData,A,L ,toc(t));
        % ----------------------------------------------------------------
        
        if toc(tp) > 0.05
            tp = tic;
            A{1}.mCADplot;
            A{2}.mCADplot;
            L.mCADPlot(A{1});
            C{1}.mCADPlot(L,A{1});
            C{2}.mCADPlot(L,A{2});
            drawnow
        end
        
    end

end
%% Close Data Files
% CloseOptDataFile(OptData);


%% Plot Results
% Roll and pitch angles
figure(1)
subplot(311),plot(XX(end,:),XX([4 16 36],:)'*180/pi); grid;
subplot(312),plot(XX(end,:),XX([5 17 37],:)'*180/pi); grid;
subplot(313),plot(XX(end,:),XX([6 18 38],:)'*180/pi); grid;
legend('\psi_{Des}[^o]','\psi_{Atu}[^o]','\psi_{Load}[^o]')

%%
% Trajectory 2D
figure(2)
plot(XX([1,13],:)',XX([2,14],:)'); grid;

% Trajectory 3D
figure(3)
plot3(XX([1,13,33],:)',XX([2,14,34],:)',XX([3,15,35],:)'); grid
%% X and Y
figure(4)
subplot(311),plot(XX(end,:),XX([1 13 33],:)'); grid
subplot(312),plot(XX(end,:),XX([2 14 34],:)'); grid
subplot(313),plot(XX(end,:),XX([3 15 35],:)'); grid
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