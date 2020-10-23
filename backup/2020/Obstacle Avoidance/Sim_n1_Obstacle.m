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
drone.pPar.Cf = 1.14298e-9;
drone.pPar.Ct = 3.12e-12; 
% L = Load;
O = Obstacle;
O.pPos.X(1:3) = [0 0 1.5];
ForceFieldPar.Forc = [0.00 0.005];
ForceFieldPar.Dist = [0.99 1.0];

% A.pPar.uSat(1:2) = A.pPar.uSat(1:2)*2;
% A.pPar.uSat(3) = A.pPar.uSat(3)*1.3;

OptData = CreateOptDataFile(A,'DesvioObs','Log_Optitrack');
gains = [1 2 1 2 2 15;
         2 13 2 15 1 5];
%% Variables Initialization
% Moviment o em X
Xd = [-1.5   0     1.2   0 0;
       1.5   0     1.2   0 17;
      -1.5   0     1.2   0 35];
  
A.pPos.X(1:3) = Xd(1,1:3);
time = 15;  % target hover time  [s]  
%%
ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')

axis image
ObjF.xlim = [-2 2.0];
ObjF.ylim = [-1.5 1.5];
ObjF.zlim = [ 0 2.5];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])
O.mCADCreate
A.mCADplot;
grid on
view(0,90)
%% Time variables

tsim = 35;   % Simulation time [s]
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
        A.pPos.Xd(6) = Xd(cont,4);   % Psi
        
        % ----------------------------------------------------------

        % Controlador
        A.rGetSensorData
        PF = sPotentialField_New(O,A,ForceFieldPar);
        A.pPar.D = [0 0 0 PF' 0]';
        disp(PF')
        A = cUnderActuatedControllerMexido(A,gains);
        A.rSendControlSignals;
        
        % ----------------------------------------------------------
        
        % Save data ------------------------------------------
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; O.pPos.X; toc(t)]];
        % ----------------------------------------------------------------
        
        if toc(tp) > 0.05
            tp = tic;
            A.mCADplot;
            drawnow
        end
        
    end

end


%% Plot results
close all
if false
    % Roll and pitch angles
    figure(1)
    subplot(311),plot(XX(end,:),XX([4 16 36],:)'*180/pi); grid; ylabel('\psi [^o]'); title('Angles')
    subplot(312),plot(XX(end,:),XX([5 17 37],:)'*180/pi); grid; ylabel('\psi [^o]');
    subplot(313),plot(XX(end,:),XX([6 18 38],:)'*180/pi); grid; ylabel('\psi [^o]'); xlabel('t [s]')
    legend(' _{Des}',' _{Atu}',' _{Load}')
end
%% Trajectory 2D
figure(2)
plot(XX([1,13],:)',XX([2,14],:)'); grid;
axis([-1 1 -1 1]*2)
ylabel('y [m]'); xlabel('x [m]')
O.mCADCreate
%% Trajectory 3D
if false
    figure(3)
    plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)'); grid;
    axis([-2 2 -2 2 0 1]*2)
    zlabel('z [m]'); ylabel('y [m]'); xlabel('x [m]')
    O.mCADCreate
end
%% X and Y
if false
    figure(4)
    subplot(311),plot(XX(end,:),XX([1 13 33],:)'); grid; ylabel('x [m]'); title('Coordinates');
    subplot(312),plot(XX(end,:),XX([2 14 34],:)'); grid; ylabel('y [m]');
    subplot(313),plot(XX(end,:),XX([3 15 35],:)'); grid; ylabel('z [m]'); xlabel('t [s]')
    legend(' _{Des}',' _{Atu}',' _{Load}')
end
%%
% velocities
figure(5)
subplot(311),plot(XX(end,:),XX([7 19],:)'); grid; ylabel('dx [m/s]'); title('Velocities');
axis([0 XX(end,end) -1 1])
subplot(312),plot(XX(end,:),XX([8 20],:)'); grid; ylabel('dy [m/s]');
axis( [0 XX(end,end) -1 1])
subplot(313),plot(XX(end,:),XX([9 21],:)'); grid; ylabel('dz [m/s]'); xlabel('t [s]')
legend(' _{Des}',' _{Atu}')
axis([0 XX(end,end) -1 1])
