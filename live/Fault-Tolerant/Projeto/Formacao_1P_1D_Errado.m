close all
clear all
clc

try
    fclose(instrfindall);
catch
end
%% ADD TO PATH
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% INICIO DO PROGRAMA
%-------------------------------------------------------------------------%
% Definindo os robôs
P = Pioneer3DX;
% A = ArDrone;
A = ArDrone;

%-------------------------------------------------------------------------%
% Formação Inicial
Q = [0 0 0 1 0 pi/2]';
dQ = [0 0 0 0 0 0]';
Qd = Q;
dQd = dQ;

% Posição dos Robôs
X = inversaF1(Q);
Xr = X;
P.pPos.X(1:3) = X(1:3);
A.pPos.X(1:3) = X(4:6);

% Pesos
L1 = 10*diag(ones(1,6));
L2 = 2*diag(ones(1,6));
K1 = 5*diag([0.5, 0.5]);
K2 = 10*diag([0.5, 0.5]);

%-------------------------------------------------------------------------%
% Figura da simulação
figure
H = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)]);
hold on
grid on
A.mCADplot;
P.mCADplot(1,'b');
axis([-3 3 -3 3 0 10])
view([0 90])

%-------------------------------------------------------------------------%
% Variáveis iniciais
Tmax = 60;
ta = 0.1;
tp = 0.1;
t1 = tic;
t2 = tic;
% A.pPar.ti = tic;

pause
t = tic;
%% SIMULAÇÃO
while toc(t) < Tmax
    if toc(t1) > ta
        t1 = tic;
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % TRAJETORIA DA FORMAÇÃO
        Qd(1) = sin(toc(t)*2*pi/Tmax);
        Qd(2) = sin(toc(t)*2*2*pi/Tmax);
        dQd(1) = 2*pi/Tmax*cos(toc(t)*2*pi/Tmax);
        dQd(2) = 2*2*pi/Tmax*cos(toc(t)*2*2*pi/Tmax);
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % PEGANDO OS DADOS DOS SENSORES
        P.rGetSensorData;
        A.rGetSensorData;
        
        Xd = inversaF1(Qd);
        P.pPos.Xd(1:3) = Xd(1:3);
        A.pPos.Xd(1:3) = Xd(4:6);
        
        dXr = dQd;
        P.pPos.dXd(1:3) = dXr(1:3);
        A.pPos.Xd(7:9) = dXr(4:6);
        
%         dXr = dXd + L1*tanh(L2*Xtil);
%         
%         dQtil = Qd - Q;
%         dQr = dQd + L2*dQtil;
%         
%         Qtil = dQtil*.1;
%         Qr = Qd + L2*Qtil;
%         
%         dXd = jacobianoInvF1(Qr,dQr);
%         Xd = inversaF1(Qr);
%         
%         
%         P.pPos.Xd(1:3) = Xd(1:3);
%         A.pPos.Xd(1:3) = Xd(4:6);
%         P.pPos.Xd(7:9) = dXd(1:3);
%         A.pPos.Xd(7:9) = dXd(4:6);
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%% CONTROLADOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % PIONEER3DX
        % Controlador Dinâmico        
        % Ganhos pré-definidos
%         cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];
%         % cgains = [ 0.10  0.10  0.75  0.75  0.75  0.75  0.10  0.05 ];
%         
%         P = fDynamicController(P,cgains);     % Pioneer Dynamic Compensator
%         P.rSendControlSignals
        
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        K = [cos(P.pPos.X(6)) -P.pPar.a*sin(P.pPos.X(6));
            sin(P.pPos.X(6)) P.pPar.a*cos(P.pPos.X(6))];
        A1 = tanh(K2*P.pPos.Xtil(1:2));
        P.pSC.Ud = K\(P.pPos.Xd([7,8]) + K1*A1);
        
        P.rSendControlSignals
%-------------------------------------------------------------------------%
        % ARDRONE
%         A = cInverseDynamicController(A);
        A = cUnderActuatedController(A);
%         A.pPar.ti = tic;
        A.rSendControlSignals;
        

%-------------------------------------------------------------------------%
        % Ajustando a formação
        X(1:3) = P.pPos.X(1:3);
        X(4:6) = A.pPos.X(1:3);
        Q = diretaF1(X);
    end
    if toc(t2) > tp
        t2 = tic;
        
        try
%             A.mCADdel
            P.mCADdel
            delete(H)
        end
        A.mCADplot;
        P.mCADplot(1,'b');
        H = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b');
        plot3(A.pPos.Xd(1),A.pPos.Xd(2),A.pPos.Xd(3),'.r');
        
        drawnow
    end
end
