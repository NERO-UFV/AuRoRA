clear all
close all
clc

A = ArDrone;
P = Pioneer3DX;

LF = LineFormationBaju;

LF.pPos.Qd = [0; -1; 0; 1; 0;    0];
%             x   y  z  r  alpha delta

LF.fInvTrans;

P.pPos.X(1:3) = LF.pPos.Xd(1:3);
A.pPos.X(1:3) = LF.pPos.Xd(4:6);

P.rSetPose(P.pPos.X([1:3 6])');

figure
hold on
grid on
P.mCADplot(0.75,'k');
A.mCADcolor([0 0 1]);
A.mCADplot;
axis([-3 3 -3 3 0 3])
view(40,40)

H = plot3([P.pPos.X(1) A.pPos.X(1)],[P.pPos.X(2) A.pPos.X(2)],[P.pPos.X(3) A.pPos.X(3)],'b',...
    'LineWidth',2);

drawnow

pause;


LF.pPar.K1 = 100*diag([1 1 1 1 1 1]);        % kinematic control gain  - controls amplitude
LF.pPar.K2 = 1*diag([1 1 1 1 1 1]);                % kinematic control gain - control saturation
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
LF.fDirTrans;
LF.fFormationError;



T_AMOSTRAGEM = A.pPar.Ts;
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
    
    LF.pPos.Qd = [P.pPos.X(1:3);
                  1;
                  0;
                  pi/4*toc(T)/TMAX];
    LF.pPos.dQd = [0; 0; 0;
                   0; 0; pi/4/TMAX];

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                          CONTROLE DE FORMAÇÃO                           %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%               
               
    LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
    
    LF.fFormationControl;
    
    LF.fInvTrans;
    
    P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);
%     A.pPos.Xd(1:3) = LF.pPos.Xd(4:6);
    
    P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);
    A.pPos.Xr(7:9) = LF.pPos.dXr(4:6);
    A.pSC.Kinematics_control = 1;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           CONTROLE DOS ROBOS                            %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%    

    P.rGetSensorData;                % Adquirir dados dos sensores - Pioneer
    P = fDynamicController(P);    % Pioneer Dynamic Compensator        
    P.rSendControlSignals;           % Enviar sinal de controle para o robô

%     A.rGetSensorData;                    % Adquirir dados dos sensores - ArDrone
%     A = cUnderActuatedController(A);  % Controlador 
%     A = cInverseDynamicController_Compensador_ArDrone(A);
%     A.pSC.Ud(1:3) = A.pPos.Xr(7:9);
    A.pPos.X(1:3) = A.pPos.Xr(7:9)*toc(Ta) + A.pPos.X(1:3);
%     A.rSendControlSignals;               % Enviar sinal de controle para o robô
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
    A.mCADplot;

    H = plot3([P.pPos.X(1) A.pPos.X(1)],[P.pPos.X(2) A.pPos.X(2)],[P.pPos.X(3) A.pPos.X(3)],'b',...
        'LineWidth',2);

    drawnow
end 
end