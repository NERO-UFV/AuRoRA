function sInvKinematicModel(robot,dXr)
% #####################################
%  Modelo simplificado, não sei se funciona!
% 
% #####################################

% Determine the robot pose, based on the control signal
%        +-------------+
% dXr -> | InvKinematic|  ->  Ur
%        | Model       |
%        +-------------+
% robot.pPos.dX([1 2 3]) = dXr;
% psi = robot.pPos.X(6);
% psi_a = robot.pPos.Xa(6);

% Velocities
% dx = dXr(1);
% dy = dXr(2);
% dz = dXr(3);
% dpsi = (psi - psi_a)/(robot.pPar.Ts);

% Acceleration [acc = (dX - dXa)/dT]

% robot.pPos.dX([7 8 9 12]) = (robot.pPos.X([7 8 9 12]) - robot.pPos.Xa([7 8 9 12]))/robot.pPar.Ts;

% Direct Kinematic Model
% % k = [5 0.3 5.5 0.1 0.55 0.7 1.45 0.7]; % parameters (Sarcinelli's class notes 2017)
%   k = [12.63 1.43 7.61 0.84 6.63 7.56 1.89 0.54];
% K1 = [k(1)*cos(psi), -k(3)*sin(psi), 0,    0;
%       k(1)*sin(psi),  k(3)*cos(psi), 0,    0;
%       0,                   0,        k(5), 0;
%       0,                   0,        0,    k(7)];
% 
% K2 = [k(2)*cos(psi), -k(4)*sin(psi), 0,    0;
%       k(2)*sin(psi), k(4)*cos(psi),  0,    0;
%         0,                   0,      k(6), 0;
%         0,                   0,      0,    k(8)];
% 
% robot.pSC.Ur = K1\(robot.pPos.dX([7 8 9 12]) + K2*[dx;dy;dz;dpsi]);

Kinv = [cos(robot.pPos.X(6)), sin(robot.pPos.X(6)),  0  ;
    -sin(robot.pPos.X(6)),    cos(robot.pPos.X(6)),  0  ;
    0,                          0,                   1  ;
    0,                          0,                   0  ];
% Reference control signal
robot.pSC.Ur = Kinv*dXr;

% save previous data
% robot.pPos.Xa = robot.pPos.X;