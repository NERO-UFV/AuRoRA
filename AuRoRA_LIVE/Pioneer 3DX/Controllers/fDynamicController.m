function robot = fDynamicController(robot,cgains)
    % This function calculates the control signal or pioneer 3dx considering
    % its dynamic variables.
    %
    % cgains is a 8 elements vector input with kinematic and dynamic gains:
    % the first four are the kinematic gains Kcin1 and Kcin2
    % the last four are the dynamic gains Kdin1 and Kdin2

    % Case do not have input gains
    if nargin < 2
    %     disp('Gains not given. Using standard ones.');
        cgains = [0.35 0.35 0.8 0.8 0.75 0.75 0.12 0.035];  
    end

    % PARAMETERS ##########################################################
    % Kinematic controller gains
        Kcin1 = diag(cgains(1:2));    % maximum value gain
        Kcin2 = diag(cgains(3:4));    % saturation gain
     % Dynamic controller gain

        Kdin1 = diag(cgains(5:6));    % maximum value gain
        Kdin2 = diag(cgains(7:8));    % saturation gain

    % Pioneer3DX initial parameters
    theta = [0.5338; 0.2168; -0.0134; 0.9560; -0.0843; 1.0590];
    % theta = [0.2604; 0.2509; -0.0005; 0.9965; 0.0026; 1.0768];

    % Dynamic matrices
    H = [theta(1) 0; 0 theta(2)];
    C = [theta(4) -theta(3)*robot.pSC.U(2); theta(5)*robot.pSC.U(2) theta(6)];

    % KINEMATIC CONTROL ##################################################

    % Desired velocity - kinematic controller
    robot.pPos.Xtil = robot.pPos.Xd - robot.pPos.X; 

    % Kinematic Model
    K = [ cos(robot.pPos.X(6)), -robot.pPar.a*sin(robot.pPos.X(6)); ...
        sin(robot.pPos.X(6)), +robot.pPar.a*cos(robot.pPos.X(6))];

    % robot.pSC.Uda = robot.pSC.Ud;
    % Kinematic controller
    if robot.pSC.Kinematics_control ==0
        robot.pSC.Ur = K\(robot.pPos.Xd(7:8) + Kcin1*tanh(Kcin2*robot.pPos.Xtil(1:2)));
    else
        robot.pSC.Ur = robot.pPos.Xr(7:8);
    end


    % Signal control saturation [Based on Pioneer 3DX datasheet]
    if abs(robot.pSC.Ur(1)) > 1
        robot.pSC.Ur(1) = sign(robot.pSC.Ur(1))*1;
    end
    if abs(robot.pSC.Ur(2)) > 1
        robot.pSC.Ur(2) = sign(robot.pSC.Ur(2))*1;
    end

    % FILTERS ###########################################################

    % Filtro Esquecimento
    % alpha = 0.25;
    % robo.pSC.dUd = (1-alpha)*robo.pSC.dUd + alpha*(robo.pSC.Ud - robo.pSC.Uda)/0.1; 

    %  Kalman Filter
    % robot.pSC.dUd = (robot.pSC.Ud - robot.pSC.Uda)/0.1; 
    % robot.pSC.dUd = (robot.pSC.U - robot.pSC.Ua)/0.1;  % calcula aceleração
    % 
    % UU = robo.pSC.dUd; 
    % Gerando variavel observada
    % Calculo do ganho de inovacao
    % GanhoK = (robo.pSC.filtro.MSE + robo.pSC.filtro.VARN)\robo.pSC.filtro.MSE;
    % % Predicao
    % robo.pSC.filtro.Ypred = robo.pSC.filtro.Ypred + GanhoK*(robo.pSC.dUd - robo.pSC.filtro.Ypred);
    % % Estimacao do erro medio quadratico
    % robo.pSC.filtro.MSE = (eye(2)-GanhoK)*robo.pSC.filtro.MSE + robo.pSC.filtro.VARW;
    % robo.pSC.dUd = robo.pSC.filtro.Ypred;

    % robo.pSC.dUUd = [robo.pSC.dUUd [UU; robo.pSC.dUd]];

    % DYNAMIC COMPENSATION ##############################################
    % robot.pSC.dUd = (robot.pSC.U - robot.pSC.Ua)/0.1;  % acceleration
    robot.pSC.dUd = (robot.pSC.U - robot.pSC.Ua)/robot.pPar.Ts;  % acceleration VALENTIM



    % nu = Kdin1*tanh(Kdin2*(robo.pSC.Ud-robo.pSC.U));
    nu = 0.1*tanh(0.1*robot.pSC.dUd) + Kdin1*tanh(Kdin2*(robot.pSC.Ur(1:2)-robot.pSC.U));

    % Reference control signal
    robot.pSC.Ud = H*nu + C*robot.pSC.Ur(1:2);

    % Signal control saturation [Based on Pioneer 3DX datasheet]
    if abs(robot.pSC.Ud(1)) > 1
        robot.pSC.Ud(1) = sign(robot.pSC.Ud(1))*1;
    end
    if abs(robot.pSC.Ud(2)) > 1
        robot.pSC.Ud(2) = sign(robot.pSC.Ud(2))*1;
    end

    % Save last velocity
    robot.pSC.Ua = robot.pSC.U(1:2);
    
end

