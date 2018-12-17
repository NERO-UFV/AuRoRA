clear; close all; clc
% Inicialização das Pastas e Arquivos
% Lendo funcao principal
% Carregando todas as pastas
addpath(genpath(pwd));
B = AhrsYEIT;
%%
% B.mIniciarCommBluetoothAHRS;
B.mIniciarCommSerialAHRS;
%%
B.mCriarBloco;
B.mCriarFiguraTracoAng;
%% Configurações
B.mSetRunningAverageMode(1);
B.mSetFiltermode(1);
%%
B.mSetLedColor('1,1,1');
%%
B.pPos.Traco.index = 0;
B.pPos.Euler = zeros(B.pCAD.TamanhoTraco,3);
t = tic;
while true %toc(t) < 500
    %%
    tT = toc(t);
    B.mAHRSCaptureEuler;
    B.mAtualizarBloco;
    B.mAtualizarTraco;
    drawnow
end
%%
B.mTerminarCommAHRS;