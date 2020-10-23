clear all
close all
clc

A{1} = ArDrone(1);
A{2} = ArDrone(2);
P = Pioneer3DX;

TF = TriangularFormationBaju;

TF.pPos.Qd = [0; -1; 0; 0;    0; pi/2; 1.5; 1.5; pi/4];
%             x   y  z  theta phi psi    p    q  beta

TF.tInvTrans;

P.pPos.X(1:3) = TF.pPos.Xd(1:3);
A{1}.pPos.X(1:3) = TF.pPos.Xd(4:6);
A{2}.pPos.X(1:3) = TF.pPos.Xd(7:9);

P.rSetPose(P.pPos.X([1:3 6])');

figure
hold on
grid on
P.mCADplot(0.75,'k');
A{1}.mCADcolor([0 0 1]);
A{1}.mCADplot;
A{2}.mCADcolor([0 1 0]);
A{2}.mCADplot;
axis([-3 3 -3 3 0 3])
view(40,40)

H(1) = plot3([P.pPos.X(1) A{1}.pPos.X(1)],[P.pPos.X(2) A{1}.pPos.X(2)],[P.pPos.X(3) A{1}.pPos.X(3)],'b',...
    'LineWidth',2);
H(2) = plot3([P.pPos.X(1) A{2}.pPos.X(1)],[P.pPos.X(2) A{2}.pPos.X(2)],[P.pPos.X(3) A{2}.pPos.X(3)],'b',...
    'LineWidth',2);
H(3) = plot3([A{1}.pPos.X(1) A{2}.pPos.X(1)],[A{1}.pPos.X(2) A{2}.pPos.X(2)],[A{1}.pPos.X(3) A{2}.pPos.X(3)],'r',...
    'LineWidth',2);

drawnow

pause;


TF.pPar.K1 = 10*diag([1 1 1 1 1 1 1 1 1]);
TF.pPar.K2 = 1*diag([1 1 1 1 1 1 1 1 1]);
TF.pPos.X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
TF.tDirTrans;
TF.tFormationError;



T_AMOSTRAGEM = A{1}.pPar.Ts;
T_PLOT = 0.5;
TMAX = 10;

T = tic;
Ta = tic;
Tp = tic;

while toc(T) < TMAX
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           TRAJETÓRIA DESEJADA                           %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%    
    
    TF.pPos.Qd = [P.pPos.X(1:3);
                  -pi/4*toc(T)/TMAX;
                  0;
                  pi/2;
                  1.5;
                  1.5;
                  pi/4];
    TF.pPos.dQd = [0; 0; 0;
                   -pi/4/TMAX;
                   0; 0;
                   0; 0; 0];

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                          CONTROLE DE FORMAÇÃO                           %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%               
               
    TF.pPos.X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
    
    TF.tFormationControl;
    
    TF.tInvTrans;
    
    P.pPos.Xd(1:3) = TF.pPos.Xd(1:3);
    A{1}.pPos.Xd(1:3) = TF.pPos.Xd(4:6);
    A{2}.pPos.Xd(1:3) = TF.pPos.Xd(7:9);
    
    P.pPos.Xd(7:9) = TF.pPos.dXr(1:3);
    A{1}.pPos.Xr(7:9) = TF.pPos.dXr(4:6);
    A{2}.pPos.Xr(7:9) = TF.pPos.dXr(7:9);
    
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           CONTROLE DOS ROBOS                            %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%    

    P.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
    P = fDynamicController(P);    % Pioneer Dynamic Compensator        
    P.rSendControlSignals;           % Enviar sinal de controle para o robô

%     A{1}.rGetSensorData;                    % Adquirir dados dos sensores - ArDrone
%     A{1} = cUnderActuatedController(A{1});  % Controlador 
%     A{1}.rSendControlSignals;               % Enviar sinal de controle para o robô
    A{1}.pPos.X(1:3) = A{1}.pPos.Xr(7:9)*toc(Ta) + A{1}.pPos.X(1:3);
%     A{1}.pPos.X(1:3) = A{1}.pPos.Xd(1:3);
%     A{2}.rGetSensorData;                    % Adquirir dados dos sensores - ArDrone
%     A{2} = cUnderActuatedController(A{2});  % Controlador 
%     A{2}.rSendControlSignals;               % Enviar sinal de controle para o robô
    A{2}.pPos.X(1:3) = A{2}.pPos.Xr(7:9)*toc(Ta) + A{2}.pPos.X(1:3);
%     A{2}.pPos.X(1:3) = A{2}.pPos.Xd(1:3);
end
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                             ROTINA DE PLOT                              %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%    
if toc(Tp) > T_PLOT
    Tp = tic;


    try
      P.mCADdel;
      delete(H);
    catch
    end

    P.mCADplot(0.75,'k');
    A{1}.mCADplot;
    A{2}.mCADplot;

    H(1) = plot3([P.pPos.X(1) A{1}.pPos.X(1)],[P.pPos.X(2) A{1}.pPos.X(2)],[P.pPos.X(3) A{1}.pPos.X(3)],'b',...
        'LineWidth',2);
    H(2) = plot3([P.pPos.X(1) A{2}.pPos.X(1)],[P.pPos.X(2) A{2}.pPos.X(2)],[P.pPos.X(3) A{2}.pPos.X(3)],'b',...
        'LineWidth',2);
    H(3) = plot3([A{1}.pPos.X(1) A{2}.pPos.X(1)],[A{1}.pPos.X(2) A{2}.pPos.X(2)],[A{1}.pPos.X(3) A{2}.pPos.X(3)],'r',...
        'LineWidth',2);

    drawnow
end 
end