function robot = fControladorCinematico(robot)

% Control gains
Kp1 = diag([0.35, 0.35]);
Kp2 = diag([0.8, 0.8]);
% 
% Kp1 = diag([0.35, 0.35]);
% Kp2 = diag([0.8, 0.001]);

K = [ cos(robot.pPos.X(6)), -robot.pPar.a*sin(robot.pPos.X(6)); ...
    sin(robot.pPos.X(6)), +robot.pPar.a*cos(robot.pPos.X(6))];

robot.pPos.Xtil = robot.pPos.Xd - robot.pPos.X;

robot.pSC.Ur = K\(robot.pPos.Xd(7:8) + Kp1*tanh(Kp2*robot.pPos.Xtil(1:2)));

% Saturação do sinal de controle, baseado na folha de dados do Pioneer 3DX
if abs(robot.pSC.Ur(1)) > 1
    robot.pSC.Ur(1) = sign(robot.pSC.Ur(1))*1;
end
if abs(robot.pSC.Ur(2)) > 1
    robot.pSC.Ur(2) = sign(robot.pSC.Ur(2))*1;
end
