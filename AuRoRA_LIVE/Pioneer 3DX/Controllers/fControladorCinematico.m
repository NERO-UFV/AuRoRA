function robot = fControladorCinematico(robot,pgains)

    % Control gains
    % pgains = [1.5 1 1.5 1]; % Ganhos Daniel

    if nargin < 2

    pgains = [0.35 0.35 0.8 0.8];

    end 

    Kp1 = diag([pgains(1), pgains(2)]);
    Kp2 = diag([pgains(3), pgains(4)]);


    K = [ cos(robot.pPos.X(6)), -robot.pPar.a*sin(robot.pPos.X(6)); ...
        sin(robot.pPos.X(6)), +robot.pPar.a*cos(robot.pPos.X(6))];

    robot.pPos.Xtil = robot.pPos.Xd - robot.pPos.X;

    robot.pSC.Ur = K\(robot.pPos.Xd(7:8) + Kp1*tanh(Kp2*robot.pPos.Xtil(1:2)));

    % Saturação do sinal de controle, baseado na folha de dados do Pioneer 3DX
    if abs(robot.pSC.Ur(1)) > 0.75
        robot.pSC.Ur(1) = sign(robot.pSC.Ur(1))*0.75;
    end
    if abs(robot.pSC.Ur(2)) > 1
        robot.pSC.Ur(2) = sign(robot.pSC.Ur(2))*1;
    end

    robot.pSC.Ud = robot.pSC.Ur;

end