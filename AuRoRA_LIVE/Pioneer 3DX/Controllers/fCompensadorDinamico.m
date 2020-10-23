% Calculate Dynamic Control Velocity
function robot = fCompensadorDinamico(robot,cGains)


% cgains is a 4 elements vector input with  dynamic gains:

% Case do not have input gains
if nargin < 2
%     disp('Gains not given. Using standard ones.');
    cGains = [0.75 0.75 0.12 0.035];
end

Kdin1 = diag(cGains(1:2));    % maximum value gain
Kdin2 = diag(cGains(3:4));    % saturation gain

% % Dynamic compensator gains
% % Kdin1 = diag([1 .01]);
% % Kdin2 = diag([1.2 0.35]);


H = [robot.pPar.theta(1)        0; ...
    0           robot.pPar.theta(2)];

C = [robot.pPar.theta(4)                -robot.pPar.theta(3)*robot.pSC.U(2); ...
    robot.pPar.theta(5)*robot.pSC.U(2)            robot.pPar.theta(6)     ];

% Kalman Filter
robot.pSC.dUd = (robot.pSC.U - robot.pSC.Ua)/0.1;  % calcula aceleração

% Creating observed variable
% % Inovation Gain
% GanhoK = (robot.pSC.filtro.MSE + robot.pSC.filtro.VARN)\robot.pSC.filtro.MSE;
% % Prediction
% robot.pSC.filtro.Ypred = robot.pSC.filtro.Ypred + GanhoK*(robot.pSC.dUd - robot.pSC.filtro.Ypred);
% % Mean Square Error Estimation
% robot.pSC.filtro.MSE = (eye(2)-GanhoK)*robot.pSC.filtro.MSE + robot.pSC.filtro.VARW;
% robot.pSC.dUd = robot.pSC.filtro.Ypred;

% nu = Kdin1*tanh(Kdin2*(robo.pSC.Ur(1:2)-robo.pSC.U));
% nu = 0.1*tanh(10*robot.pSC.dUd) + Kdin1*tanh(Kdin2*(robot.pSC.Ur(1:2)-robot.pSC.U));
nu = 0.1*tanh(.1*robot.pSC.dUd) + Kdin1*tanh(Kdin2*(robot.pSC.Ur(1:2)-robot.pSC.U));
% nu = .6*tanh(robot.pSC.dUd) + Kdin1*tanh(Kdin2*(robot.pSC.Ur(1:2)-robot.pSC.U));

% Reference control Signal
robot.pSC.Ud = H*nu + C*robot.pSC.Ur(1:2);
% robot.pSC.Ud = 1.2*robot.pSC.Ur(1:2); % % Teste 7
% Save last velocity
robot.pSC.Ua = robot.pSC.Ur(1:2);
