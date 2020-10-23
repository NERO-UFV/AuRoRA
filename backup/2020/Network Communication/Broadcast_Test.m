clear
close all
clc

try
    fclose(instrfindall)
end

addpath(genpath(pwd))

ID = input('ID: ');
P = Pioneer3DX(ID);

Rede = InfoSharing;

%% 
P.pPos.X(1:2) = [1 ; 0];
P.pPos.Ur(1:2) = [1 ; 0];

Rede.mSendMsg(P);

%%
Rede.mReceiveMsg;


