% Inicialização
close all
clear
clc
 
try
    fclose(instrfindall);
catch
end
 
% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))
 
gains = [1 2 1 2 2 15;
         2 13 2 15 1 5];
%%
% Robot initialization
 
% Robot initialization
A{1} = ArDrone;
A{2} = ArDrone;
A{3} = ArDrone;
 
mCADcolor(A{1},[1 0 0])
mCADcolor(A{2},[0 1 0])
mCADcolor(A{2},[0 0 1])
L{1} = Load;
L{2} = Load;
L{3} = Load;
 
L.pPar.l = 873/1000;
L.pPos.X(4:5) = [0 0];
 
%% Figure
ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')
 
axis image
ObjF.xlim = [-3 3.0];
ObjF.ylim = [-3 3.0];
ObjF.zlim = [ 0 2.5];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])
 
grid on
 
L.mCADCreate
%% Time variables
tmax = 30;
t = tic;
tc = tic;
tp = tic;
XX = [];    
%%
while toc(t) < tmax 
    if toc(tc) > 1/30
        tc = tic;
        if toc(t) > 30
            A.pPos.Xd(1:3,1) = [0 0 1.5]; else
            if toc(t) > 20
                A.pPos.Xd(1:3,1) = [1 1 1.5]; else
                if toc(t) > 10
                    A.pPos.Xd(1:3,1) = [1 0 1.5];  else
                    A.pPos.Xd(1:3,1) = [0 0 1.5];
                end
            end
        end
         
        % ----------------------------------------------------------
        %  Get current rigid body information from optitrack
        A.rGetSensorData
%         A = cUnderActuatedLoadControllerHacked(A,L,gains);
%           A = cUnderActuatedLoadControllerv1(A,L,gains);
%         A = cUnderActuatedLoadController(A,L);
%         A = cUnderActuatedController(A,gains);
        LD = dLoadDisturbance(L);
%         A.pPar.D(3) = 0;
        A.pPar.D = [LD' 0 0 0 0 0 0]';
        A = cUnderActuatedControllerMexido(A,gains);
         
         
 
        A.rSendControlSignals;
        L.sLoadDynamicModel(A);
         
        % Save Variables
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; L.pPos.X; toc(t)]];
    end
    if toc(tp) > 0.05
        tp = tic;
        A.mCADplot;
        L.mCADPlot(A);
        drawnow
    end
end
 
disp('Fim!')
%%
close all
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