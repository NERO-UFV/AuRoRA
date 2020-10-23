%% Test to create a load in the ambient

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

% Robot initialization
A = ArDrone;
L = Load;
L.pPos.X(4:5) = [0 0];
drone.pPar.Cf = 1.14298e-9;
drone.pPar.Ct = 3.12e-12; 
% L = Load;
O(1) = Obstacle;
O(2) = Obstacle;
O(3) = Obstacle;

ForceFieldPar.Forc = [0.00 .02];
ForceFieldPar.Dist = [0.50 1.5];

% A.pPar.uSat(1:2) = A.pPar.uSat(1:2)*2;
% A.pPar.uSat(3) = A.pPar.uSat(3)*1.3;

O(1).pPos.X(1:3) = [-1.2 0 2];
O(2).pPos.X(1:3) = [ 0 0 2];
O(3).pPos.X(1:3) = [ 1 1 2];

gains = [1 2 1 2 2 15;
         2 13 2 15 1 5];

OptData = CreateOptDataFile(A,L,'LoadSim','Log_Optitrack');
%% Variables Initialization
% Moviment o em X
Xd = [-3   0     1.2   0 0;
       3   0     1.2   0 20;
      -3   0     1.2   0 50];
A.pPos.X(1:3) = Xd(1,1:3);
time = 15;  % target hover time  [s]  
%%
ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')

axis image
ObjF.xlim = [-5 5];
ObjF.ylim = [-3 3];
ObjF.zlim = [ 0 2.5];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])

O(1).mCADCreate
O(2).mCADCreate
O(3).mCADCreate

A.mCADplot;
L.mCADCreate
grid on
% view(0,90)
%% Time variables

tsim = 40;   % Simulation time [s]
cont = 1;    % counter
angulos = [];
XX = [];             % position data
tout = 100;              % maximum simulation duration [s]
tc = tic;                % drone frequency
tp = tic;                % graph refresh rate
tt = tic;                % trajectory time
t = tic;                 % simulation current time
td = tic;
ta = [];
dt = 1/30;

%% Simulation loop
while toc(t) < tsim
    if toc(tc) > A.pPar.Ts
        ta = [ta toc(tc)];
        tc = tic;
        
        % Desired Position
        if toc(t) > Xd(cont,5) && cont < size(Xd,1)
            cont = cont + 1;
        end
        
        A.pPos.Xd(1) = Xd(cont,1);   % x
        A.pPos.Xd(2) = Xd(cont,2);   % y
        A.pPos.Xd(3) = Xd(cont,3);   % z
        A.pPos.Xd(6) = Xd(cont,4);
        
        % ----------------------------------------------------------

        % Controlador
        A.rGetSensorData
        D1 = sPotentialField(O(1),A,ForceFieldPar);
        D2 = sPotentialField(O(2),A,ForceFieldPar);
        D3 = sPotentialField(O(3),A,ForceFieldPar);
        ObstD = D1+D2+D3;
        LoadD = mLoadDisturbance(L);
       
        A.pPar.D = [LoadD' 0 0 0 0 ObstD 0]';
        
%         A.pPar.D(1:2) = ForceDrObT(1:2);
        %         A.pSC.D(1:2) = [.1 0];
        % A carga parece nao fazer efeito
        % L.sCalculateAngles(A,gains);
        % A = cUnderActuatedLoadControllerHacked(A,L);
        A = cUnderActuatedControllerMexido(A,gains);
%          A = cUnderActuatedControllerwDisturbance(A,gains);
%         A = cUnderActuatedController(A,gains);        
        % A = J.mControl(A);
                L.sLoadDynamicModel(A);
        A.rSendControlSignals;
        
        % ----------------------------------------------------------
        
        % Save data ------------------------------------------
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; L.pPos.X; toc(t)]];
        SaveOptData(OptData,A,L ,toc(t));
        % ----------------------------------------------------------------
        

        if toc(tp) > 0.05
            tp = tic;
            A.mCADplot;
            L.mCADPlot(A);
            drawnow
        end
        
    end

end
%% Close Data Files
CloseOptDataFile(OptData);

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















