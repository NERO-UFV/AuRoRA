function drone = cUnderActuatedController(drone,cgains)
% Case do not have input gains
if nargin < 2
    cgains = [0.5 2 0.5 2 5 2; 1 15 1 15 1 2.5];  
    % disp('Gains not given. Using standard ones.');
%     cgains = [2 5 2 5 15 10; 20 5 20 5 15 10];
end
% Controllers Gains.
% The Gains must be given in the folowing order
% kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
% cgains = [.5 2 .5 2 5 2; 1 20 1 15 1 2.5];


% ---------------------------------------------------------
% % Seguimento de Trajetória: Ganhos mais elevados 
% % Ganhos para simulação (funcionando 17_07_18 - Marcos)
% Ganhos.kx1 = 1.5;
% Ganhos.kx2 = 3.2;
% Ganhos.kx3 = sqrt(4*Ganhos.kx1);
% Ganhos.kx4 = sqrt(4*Ganhos.kx1*Ganhos.kx2)/Ganhos.kx3;
% 
% Ganhos.ky1 = 1.0;
% Ganhos.ky2 = 3.2;
% Ganhos.ky3 = sqrt(4*Ganhos.ky1);
% Ganhos.ky4 = sqrt(4*Ganhos.ky1*Ganhos.ky2)/Ganhos.ky3;
% 
% Ganhos.kz1 = 1.5;%20;
% Ganhos.kz2 = 2;
% Ganhos.kz3 = sqrt(4*Ganhos.kz1);
% Ganhos.kz4 = sqrt(4*Ganhos.kz1*Ganhos.kz2)/Ganhos.kz3;
% 
% % phi
% Ganhos.kp1 = 7;% 25z
% Ganhos.kp2 = 3;
% Ganhos.kp3 = sqrt(4*Ganhos.kp1);
% Ganhos.kp4 = sqrt(4*Ganhos.kp1*Ganhos.kp2)/Ganhos.kp3;
% % theta
% Ganhos.kt1 = 10;
% Ganhos.kt2 = 5;
% Ganhos.kt3 = sqrt(4*Ganhos.kt1);
% Ganhos.kt4 = sqrt(4*Ganhos.kt1*Ganhos.kt2)/Ganhos.kt3;
% %psi
% Ganhos.ks1 = 1;
% Ganhos.ks2 = 5;
% Ganhos.ks3 = sqrt(4*Ganhos.ks1);
% Ganhos.ks4 = sqrt(4*Ganhos.ks1*Ganhos.ks2)/Ganhos.ks3;


% % Teste ganho drone real - marcos
% Ganhos.kx1 = .5;
% Ganhos.kx2 = 1.8;
% Ganhos.kx3 = sqrt(4*Ganhos.kx1);
% Ganhos.kx4 = sqrt(4*Ganhos.kx1*Ganhos.kx2)/Ganhos.kx3;
% 
% Ganhos.ky1 = 1;
% Ganhos.ky2 = 1.5;
% Ganhos.ky3 = sqrt(4*Ganhos.ky1);
% Ganhos.ky4 = sqrt(4*Ganhos.ky1*Ganhos.ky2)/Ganhos.ky3;
% 
% Ganhos.kz1 = 4.8;%20;
% Ganhos.kz2 = 1.9;%10;
% Ganhos.kz3 = sqrt(4*Ganhos.kz1);
% Ganhos.kz4 = sqrt(4*Ganhos.kz1*Ganhos.kz2)/Ganhos.kz3;
% 
% % phi
% Ganhos.kp1 = 1.2;%1;% 25z
% Ganhos.kp2 = 15;
% Ganhos.kp3 = sqrt(4*Ganhos.kp1);
% Ganhos.kp4 = sqrt(4*Ganhos.kp1*Ganhos.kp2)/Ganhos.kp3;
% % theta
% Ganhos.kt1 = 1.1;
% Ganhos.kt2 = 8;
% Ganhos.kt3 = sqrt(4*Ganhos.kt1);
% Ganhos.kt4 = sqrt(4*Ganhos.kt1*Ganhos.kt2)/Ganhos.kt3;
% %psi
% Ganhos.ks1 = 1; %2.5
% Ganhos.ks2 = 2.5; %2.75
% Ganhos.ks3 = sqrt(4*Ganhos.ks1);
% Ganhos.ks4 = sqrt(4*Ganhos.ks1*Ganhos.ks2)/Ganhos.ks3;



% Vector example with the controllers gains.
% Must be used after declaring the drone
% cgains = [.5 2 .5 2 5 2; 1 20 1 15 1 2.5];

Ganhos.kx1 = cgains(1,1);
Ganhos.kx2 = cgains(1,2);
Ganhos.kx3 = sqrt(4*Ganhos.kx1);
Ganhos.kx4 = sqrt(4*Ganhos.kx1*Ganhos.kx2)/Ganhos.kx3;

Ganhos.ky1 = cgains(1,3);
Ganhos.ky2 = cgains(1,4);
Ganhos.ky3 = sqrt(4*Ganhos.ky1);
Ganhos.ky4 = sqrt(4*Ganhos.ky1*Ganhos.ky2)/Ganhos.ky3;

Ganhos.kz1 = cgains(1,5);
Ganhos.kz2 = cgains(1,6);
Ganhos.kz3 = sqrt(4*Ganhos.kz1);
Ganhos.kz4 = sqrt(4*Ganhos.kz1*Ganhos.kz2)/Ganhos.kz3;

% phi
Ganhos.kp1 = cgains(2,1);
Ganhos.kp2 = cgains(2,2);
Ganhos.kp3 = sqrt(4*Ganhos.kp1);
Ganhos.kp4 = sqrt(4*Ganhos.kp1*Ganhos.kp2)/Ganhos.kp3;
% theta
Ganhos.kt1 = cgains(2,3);
Ganhos.kt2 = cgains(2,4);
Ganhos.kt3 = sqrt(4*Ganhos.kt1);
Ganhos.kt4 = sqrt(4*Ganhos.kt1*Ganhos.kt2)/Ganhos.kt3;
%psi
Ganhos.ks1 = cgains(2,5);
Ganhos.ks2 = cgains(2,6);
Ganhos.ks3 = sqrt(4*Ganhos.ks1);
Ganhos.ks4 = sqrt(4*Ganhos.ks1*Ganhos.ks2)/Ganhos.ks3;

% % ---------------------------------------------------------
drone.pPos.Xda = drone.pPos.Xd;

% Calculando erro de posição
drone.pPos.Xtil = drone.pPos.Xd - drone.pPos.X;
% display(drone.pPos.Xtil(4:6)');
for ii = 4:6
    if abs(drone.pPos.Xtil(ii)) > pi
        drone.pPos.Xtil(ii) = -2*pi + drone.pPos.Xtil(ii);
    end
    if drone.pPos.Xtil(ii) < -pi
        drone.pPos.Xtil(ii) = 2*pi + drone.pPos.Xtil(ii);
    end
end

% Matriz de rotação
Rx = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
Ry = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
Rz = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];

R = (Rz*Ry*Rx);

%-------------------------------
% Controle Cinematico
%-------------------------------
etax = drone.pPos.dXd(7) + Ganhos.kx1*tanh(Ganhos.kx2*drone.pPos.Xtil(1)) + Ganhos.kx3*tanh(Ganhos.kx4*drone.pPos.Xtil(7));
etay = drone.pPos.dXd(8) + Ganhos.ky1*tanh(Ganhos.ky2*drone.pPos.Xtil(2)) + Ganhos.ky3*tanh(Ganhos.ky4*drone.pPos.Xtil(8));
etaz = drone.pPos.dXd(9) + Ganhos.kz1*tanh(Ganhos.kz2*drone.pPos.Xtil(3)) + Ganhos.kz3*tanh(Ganhos.kz4*drone.pPos.Xtil(9));

etap = drone.pPos.dXd(10) + Ganhos.kp1*tanh(Ganhos.kp2*drone.pPos.Xtil(4)) + Ganhos.kp3*tanh(Ganhos.kp4*drone.pPos.Xtil(10));
etat = drone.pPos.dXd(11) + Ganhos.kt1*tanh(Ganhos.kt2*drone.pPos.Xtil(5)) + Ganhos.kt3*tanh(Ganhos.kt4*drone.pPos.Xtil(11));
etas = drone.pPos.dXd(12) + Ganhos.ks1*tanh(Ganhos.ks2*drone.pPos.Xtil(6)) + Ganhos.ks3*tanh(Ganhos.ks4*drone.pPos.Xtil(12));
% disp('%-------------------------------')

% Referência de Rolagem e Arfagem (Inserir Filtragem)
drone.pPos.Xd(4) =  atan2((etax*sin(drone.pPos.X(6))-etay*cos(drone.pPos.X(6)))*cos(drone.pPos.X(5)),(etaz+drone.pPar.g));
drone.pPos.Xd(5) =  atan2((etax*cos(drone.pPos.X(6))+etay*sin(drone.pPos.X(6))),(etaz+drone.pPar.g));


% display(drone.pPos.Xd(4:5));
% Linear Kalman filter
GainK = (drone.pPar.LKF.msed + drone.pPar.LKF.varnd)\drone.pPar.LKF.msed;
drone.pPar.LKF.xd  = drone.pPar.LKF.xd + GainK*(drone.pPos.Xd(4:5) - drone.pPar.LKF.xd);
drone.pPar.LKF.msed = (eye(2)-GainK)*drone.pPar.LKF.msed + drone.pPar.LKF.varwd;
drone.pPos.Xd(4:5)  = drone.pPar.LKF.xd;

% Criar fitro de orientação
% alpha = 0.7;
% drone.pPos.Xd(4) = alpha*drone.pPos.Xda(4) + (1-alpha)*drone.pPos.Xd(4);
% drone.pPos.Xd(5) = alpha*drone.pPos.Xda(5) + (1-alpha)*drone.pPos.Xd(5);
% drone.mFiltroArfagemRolagem;

% =========================================================================
% Parte Translacional
Mt = drone.pPar.m*eye(3,3);              % Inertia matrix
Ct = zeros(3,3);                         % Coriolis matrix
Gt = [0; 0; drone.pPar.m*drone.pPar.g];  % Gravity matrix

% =========================================================================
% Rotational inertia matrix
Mr = [drone.pPar.Ixx, ...
    drone.pPar.Ixy*cos(drone.pPos.X(4)) - drone.pPar.Ixz*sin(drone.pPos.X(4)), ...
    -drone.pPar.Ixx*sin(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5));
    
    drone.pPar.Ixy*cos(drone.pPos.X(4)) - drone.pPar.Ixz*sin(drone.pPos.X(4)), ...
    drone.pPar.Iyy*cos(drone.pPos.X(4))^2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)),...
    drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5));
    
    -drone.pPar.Ixx*sin(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)), ...
    drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5)),...
    drone.pPar.Ixx*sin(drone.pPos.X(5))^2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - 2*drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - 2*drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2
    ];

% Rotational Coriolis matrix
Cr = [ 0, ...
    drone.pPos.X(11)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(11)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2);
    
    drone.pPos.X(10)*(-drone.pPar.Ixy*sin(drone.pPos.X(4)) - drone.pPar.Ixz*cos(drone.pPos.X(4))) + drone.pPos.X(11)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2),...
    drone.pPos.X(10)*(drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)));
    
    drone.pPos.X(10)*(drone.pPar.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2),...
    drone.pPos.X(10)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))) + drone.pPos.X(12)*(drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2) + drone.pPos.X(11)*(drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)))
    ];

% Gravity vector
Gr = [0; 0; 0];

% Modelo no formato: M \ddot{q} + C \dot{q} + G = F
Z = zeros(3,3);

MM = [Mt Z; Z Mr];   % Matriz de Inércia

CC = [Ct Z; Z Cr];   % Matriz de Coriolis

GG = [Gt; Gr];       % Vetor de Forças Gravitacionais

% Matriz de Acoplamento e Matriz dos Braços de Forcas
% [F1 F2 F3]' = R*At*[fx fy fz fytr]'
At = R*[0 0 0 0; 0 0 0 0; 1 1 1 1];

% [L M N]' = Ar*[fx fy fz fytr]'
Ar = [ drone.pPar.k1  drone.pPar.k1 -drone.pPar.k1  -drone.pPar.k1;
      -drone.pPar.k1  drone.pPar.k1  drone.pPar.k1  -drone.pPar.k1;
       drone.pPar.k2 -drone.pPar.k2  drone.pPar.k2  -drone.pPar.k2];

A = [At;Ar];

% Matriz Pseudo-Inversa de A: A-sharp
As = pinv(A); %(A'*A)\A';

% Montagem da matriz sub-atuada ativa
% Matriz de Inérica
Ma = As*MM;
Map = Ma(:,1:2); % Passive variables X and Y
Maa = Ma(:,3:6); % Active variables Z, PHI, THETA and PSI

% Matriz de Coriolis e Vetor de Forças Gravitacionais Ativa
Ea  = As*(CC*drone.pPos.X(7:12) + GG + [drone.pPar.D(1:3)' 0 0 0]');

% Escrita das matrizes passivas
MP = R'*Mt;
GP = R'*Gt;

Mpp =  MP(1:2,1:2);
Mpa = [MP(1:2,3) zeros(2,3)];

Ep = GP(1:2,1);

%==========================================================================
% Representação na forma sub-atuada
% M = [Mpp Mpa; Map Maa];
% E = [Ep; Ea];

D = Maa - Map*(Mpp\Mpa);
H = Ea - Map*(Mpp\Ep);

eta = [etaz; etap; etat; etas];

% Vetor de Forças de referência aplicado no referencial do veículo
Fr = D*eta + H;

% Verificando se forças sobre o referência do veículo
% ocorrem somente na direção Z
fTau = A*Fr;

% ------------------------------------
% Forçando valores possíveis: 30% do valor da gravidade
if real(fTau(3)) < 0
    fTau(3) = drone.pPar.m*drone.pPar.g*0.3;
end
% ------------------------------------

% Considerando a situação mais simples de que a força de propulsão
% solicitada aos motores é imediatamente atendida
% NÂO considera modelo da bateria
% Modelo Inverso do Atuador: Forças desejada nos propulsores
Fd = As*fTau;

% Caso a força do propulsor seja negativa, assume-se propulsão igual a zero
for ii = 1:4
    if Fd(ii) < 0
        Fd(ii) = 0;
    end
end

% 1: Fr -> Wr
Wda = drone.pSC.Wd;
drone.pSC.Wd = sqrt(Fd/drone.pPar.Cf);

% 2: Wr -> V 
Vr = -drone.pPar.Vo + drone.pPar.Jm*drone.pPar.R/drone.pPar.Km*(drone.pSC.Wd-Wda)/drone.pPar.Ts + ...
    (drone.pPar.Bm*drone.pPar.R/drone.pPar.Km + drone.pPar.Kb)*drone.pSC.Wd + ...
    drone.pPar.Ct*drone.pPar.R/drone.pPar.Km/drone.pPar.r*drone.pSC.Wd.^2;


% 3: V -> Xr
drone.pSC.Xr(4) = drone.pPos.X(4) + 1/(drone.pPar.kdp+drone.pPar.kpp*drone.pPar.Ts)*...
    (drone.pPar.kdp*(drone.pSC.Xr(4)-drone.pPos.X(4)) + 1/4*drone.pPar.Ts*([1 1 -1 -1]*Vr));

drone.pSC.Xr(5) = drone.pPos.X(5) + 1/(drone.pPar.kdt+drone.pPar.kpt*drone.pPar.Ts)*...
    (drone.pPar.kdt*(drone.pSC.Xr(5)-drone.pPos.X(5)) + 1/4*drone.pPar.Ts*([-1 1 1 -1]*Vr));

drone.pSC.Xr(9) = drone.pPos.X(9) + 1/(drone.pPar.kdz+drone.pPar.kpz*drone.pPar.Ts)*...
    (drone.pPar.kdz*(drone.pSC.Xr(9)-drone.pPos.X(9)) + 1/4*drone.pPar.Ts*([1 1 1 1]*Vr));

drone.pSC.Xr(12) = drone.pPos.X(12) + 1/(drone.pPar.kds+drone.pPar.kps*drone.pPar.Ts)*...
    (drone.pPar.kds*(drone.pSC.Xr(12)-drone.pPos.X(12)) + 1/4*drone.pPar.Ts*([1 -1 1 -1]*Vr));

% 4: Xr -> U
drone.pSC.Ud(1) =  drone.pSC.Xr(4)/drone.pPar.uSat(1);   % Phi
drone.pSC.Ud(2) = -drone.pSC.Xr(5)/drone.pPar.uSat(2);   % Theta
drone.pSC.Ud(3) =  drone.pSC.Xr(9)/drone.pPar.uSat(3);   % dZ
drone.pSC.Ud(4) = -drone.pSC.Xr(12)/drone.pPar.uSat(4);  % dPsi

drone.pSC.Ud = tanh(drone.pSC.Ud);

end