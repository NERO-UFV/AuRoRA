%% SIMULAÇÃO
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Iniciando as variaveis
tmax = 5;
ta = 0.1;

tp = tic;
t = tic;

while toc(t) < tmax
    if toc(tp) > ta
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Get data from robot and Optitrack
        tp = tic;
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);       % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        rb = OPT.RigidBody;             % read optitrack
        P = getOptData(rb(idP),P);
        
        % Pegando os dados do robo
        P.rGetSensorData;
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Trajetória
        P.pPos.Xd(1) = 0.6;
        P.pPos.Xd(2) = 0.6;
        P.pPos.Xd(7) = 0;
        P.pPos.Xd(8) = 0;
%         plot(Xt,Yt,'.g','MarkerSize',15)
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Pegando os dados do robo
%         P.rGetSensorData;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%        
        % Controlador Dinâmico        
        % Ganhos pré-definidos
%         cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];
        cgains = [ 0.10  0.10  0.75  0.75  0.75  0.75  0.10  0.05 ];
        
        P = fDynamicController(P,cgains);     % Pioneer Dynamic Compensator
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Enviando sinais para o robo
        Rede.mSendMsg(P);
%         P.rSendControlSignals;
    end
end