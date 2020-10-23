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
O(1) = Obstacle;
O(2) = Obstacle;
O(3) = Obstacle;

O(1).pPos.X(1:3) = [-.5 0 2];
O(2).pPos.X(1:3) = [ .75 0 2];
O(3).pPos.X(1:3) = [ .3 -1.3 2];

ForceFieldPar.Forc = [0.00 0.0058];
ForceFieldPar.Dist = [0.89 0.9];

% A.pPar.uSat(1:2) = A.pPar.uSat(1:2)*2;
% A.pPar.uSat(3) = A.pPar.uSat(3)*1.3;

gains = [1 1.5 1 1.5 2 15;
         2 12 2 12 1 5];
%% Variables Initialization
% Moviment o em X
Xd = [-1.5   0     1.2   0 0;
       1.5   0     1.2   0 20;
      -1.5   0     1.2   0 40];
A.pPos.X(1:3) = Xd(1,1:3);
time = 15;  % target hover time  [s]  
%%

ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')

axis image
ObjF.xlim = [-2 2];
ObjF.ylim = [-1.5 1.5];
ObjF.zlim = [ 0 2.5];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])

O(1).mCADCreate
O(2).mCADCreate
O(3).mCADCreate

A.mCADplot;
grid on
view(0,90)
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
        PF = sPotentialField_New(O,A,ForceFieldPar);
%         D2 = sPotentialField_New(O(2),A,ForceFieldPar);
%         D3 = sPotentialField_New(O(3),A,ForceFieldPar);
%         PF = D1+D2+D3;
        A.pPar.D = [0 0 0 PF' 0]';
%         disp(PF)
        A = cUnderActuatedControllerMexido(A,gains);
%         A = cUnderActuatedController(A,gains);        
        A.rSendControlSignals;
        
        % ----------------------------------------------------------
        
        % Save data ------------------------------------------
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; O(1).pPos.X; toc(t)]];
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
% Roll and pitch angles
figure(1)
subplot(311),plot(XX(end,:),XX([4 16 36],:)'*180/pi); grid; ylabel('\psi [^o]'); title('Angles')
subplot(312),plot(XX(end,:),XX([5 17 37],:)'*180/pi); grid; ylabel('\psi [^o]'); 
subplot(313),plot(XX(end,:),XX([6 18 38],:)'*180/pi); grid; ylabel('\psi [^o]'); xlabel('t [s]')
legend(' _{Des}',' _{Atu}',' _{Load}')

%% Trajectory 2D
figure(2)
plot(XX([1,13],:)',XX([2,14],:)'); grid;
axis([-1 1 -1 1]*2)
ylabel('y [m]'); xlabel('x [m]')

O(1).mCADCreate
O(2).mCADCreate
O(3).mCADCreate
%% Trajectory 3D
figure(3)
plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)'); grid;
axis([-2 2 -2 2 0 1]*2)
zlabel('z [m]'); ylabel('y [m]'); xlabel('x [m]')
O(1).mCADCreate
O(2).mCADCreate
O(3).mCADCreate

%% X and Y
figure(4)
subplot(311),plot(XX(end,:),XX([1 13 33],:)'); grid; ylabel('x [m]'); title('Coordinates');
subplot(312),plot(XX(end,:),XX([2 14 34],:)'); grid; ylabel('y [m]');
subplot(313),plot(XX(end,:),XX([3 15 35],:)'); grid; ylabel('z [m]'); xlabel('t [s]')
legend(' _{Des}',' _{Atu}',' _{Load}')
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
















