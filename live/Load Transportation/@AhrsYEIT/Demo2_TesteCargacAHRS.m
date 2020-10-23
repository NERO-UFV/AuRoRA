clear; close all; clc
try cd('D:\Dropbox\AuRoRA 2018'); end
% Inicialização das Pastas e Arquivos
% Lendo funcao principal
% Carregando todas as pastas
addpath(genpath(pwd));
A = ArDrone;
B = AhrsYEIT;
C = Load;
%%
% B.mIniciarCommBluetoothAHRS;
B.mIniciarCommSerialAHRS;
%%
h = figure(1);
set(h,'units','pix','pos',[900 250 1000 700],'PaperPositionMode','auto')
set(h,'renderer','opengl');
% Criar mundo
hold on
view(25,22)
axis equal
axis image
axis([-2 2 -2 2 -.5 2])
grid on
%%
% B.mCriarFiguraTracoAng;
C.pPos.X(1:3) =  [0 0 0.5];
A.pPos.X(1:3) =  [0 0 1.5];
C.mCADcreate;
A.mCADplot('on');
%% Configurações da Ahrs
B.mSetRunningAverageMode(1);
B.mSetFiltermode(1);
%%
B.mSetLedColor('0,1,0');
%%
B.pPos.Traco.index = 0;
% B.pPos.Euler = zeros(B.pCAD.TamanhoTraco,3);
t = tic;
while true %toc(t) < 500
    %%
    tT = toc(t);
    B.mAHRSCaptureEuler;    
    
    RotX = [1 0 0; 0 cos(C.pPar.Eulerangs(1)) -sin(C.pPar.Eulerangs (1)); 0 sin(C.pPar.Eulerangs(1)) cos(C.pPar.Eulerangs(1))];
    RotY = [cos(C.pPar.Eulerangs(2)) 0 sin(C.pPar.Eulerangs(2)); 0 1 0; -sin(C.pPar.Eulerangs(2)) 0 cos(C.pPar.Eulerangs(2))];
    RotZ = [cos(C.pPar.Eulerangs(3)) -sin(C.pPar.Eulerangs(3)) 0; sin(C.pPar.Eulerangs(3)) cos(C.pPar.Eulerangs(3)) 0; 0 0 1];
    Rot = RotZ*RotY*RotX;
    
    A.mCADplot('on');
    q(1) = C.pPar.l*sin(C.pPar.Eulerangs(1));
    q(2) = C.pPar.l*sin(C.pPar.Eulerangs(2));
    q(3) = A.pPos.X(3)-C.pPar.l*cos(C.pPar.Eulerangs(1));
    
    p = RotZ*[1 0 0; 0 1 0; 0 0 1]*q';
    set(Carga,'Xdata',(x*r+p(1)),'Ydata',(y*r+p(2)),'Zdata',(z*r+p(3)));
    ds.XData = [0 p(1)]; ds.YData = [0 p(2)]; ds.ZData = [A.pPos.X(3) p(3)];
    
%     B.mAtualizarBloco;
%     B.mAtualizarTraco;
    drawnow
end
%%
B.mTerminarCommAHRS;