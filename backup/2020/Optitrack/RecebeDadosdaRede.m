%% Script para receber dados do PC com o Optitrack

clear
close all
clc
try
    fclose(instrfindall);
catch
end
%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Window configuration
fig = figure(1);
axis([-2 2 -2 2])
% pause(1)

%% Cria classes
P = Pioneer3DX;
A = ArDrone;
Rede = NetDataShare;

tm = tic;
% Verifica se alguma informação da rede foi recebida;
% O programa só continuará caso alguma mensagem seja recebida.
while isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P);
    if toc(tm) > 0.1
        tm = tic;
        Rede.mReceiveMsg;
        disp('Waiting for message......')
    end
end
clc
disp('Data received......');

%%
k = 0;   % contador de pacotes perdidos
tp = tic;
while true
    %     try
    % Desenha os robôs
    if toc(tp) > 1/30
        tp = tic;
        Rede.mReceiveMsg;
        if isempty(Rede.pMSG.getFrom)
            k = k+1;  % número de vezes que não leu nada
        else
            try
                P.pPos.X  = Rede.pMSG.getFrom{1}(14+(1:12));
            end
            %P.pPos.X([1 2 6])  = Rede.pMSG.getFrom{1}(3:5);
            
            % Calculate control point position
            P.pPos.Xc([1 2 6]) = P.pPos.X([1 2 6]) - ...
                [P.pPar.a*cos(P.pPos.X(6)); P.pPar.a*sin(P.pPos.X(6)); 0];
            
        end
        
        try
            P.mCADdel
            %                 delete(fig);
            
        catch
        end
        P.mCADplot2D('b')
        %                                         P.mCADplot(1,'k')
        %             plot(P.pPos.Xc(1),P.pPos.Xc(2),'x','LineWidth',2)
        axis([-2 2 -2 2])
        hold on
        grid on
        drawnow
        
    end
    %     catch
    %     end
end
