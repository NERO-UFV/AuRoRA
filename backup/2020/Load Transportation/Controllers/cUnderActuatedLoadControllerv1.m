function drone = cUnderActuatedLoadControllerv1(drone,load,Cgains)

% Teste ganho drone real - para 1/30
Ganhos.kx1 = Cgains(1,1);
Ganhos.kx2 = Cgains(1,2);
Ganhos.kx3 = sqrt(4*Ganhos.kx1);
Ganhos.kx4 = sqrt(4*Ganhos.kx1*Ganhos.kx2)/Ganhos.kx3;

Ganhos.ky1 = Cgains(1,3);
Ganhos.ky2 = Cgains(1,4);
Ganhos.ky3 = sqrt(4*Ganhos.ky1);
Ganhos.ky4 = sqrt(4*Ganhos.ky1*Ganhos.ky2)/Ganhos.ky3;

Ganhos.kz1 = Cgains(1,5);
Ganhos.kz2 = Cgains(1,6);
Ganhos.kz3 = sqrt(4*Ganhos.kz1);
Ganhos.kz4 = sqrt(4*Ganhos.kz1*Ganhos.kz2)/Ganhos.kz3;

% phi
Ganhos.kp1 = Cgains(2,1);
Ganhos.kp2 = Cgains(2,2);
Ganhos.kp3 = sqrt(4*Ganhos.kp1);
Ganhos.kp4 = sqrt(4*Ganhos.kp1*Ganhos.kp2)/Ganhos.kp3;
% theta
Ganhos.kt1 = Cgains(2,3);
Ganhos.kt2 = Cgains(2,4);
Ganhos.kt3 = sqrt(4*Ganhos.kt1);
Ganhos.kt4 = sqrt(4*Ganhos.kt1*Ganhos.kt2)/Ganhos.kt3;
%psi
Ganhos.ks1 = Cgains(2,5);
Ganhos.ks2 = Cgains(2,6);
Ganhos.ks3 = sqrt(4*Ganhos.ks1);
Ganhos.ks4 = sqrt(4*Ganhos.ks1*Ganhos.ks2)/Ganhos.ks3;


%%
drone.pPos.Xda = drone.pPos.Xd;

% Calculando erro de posição
drone.pPos.Xtil = drone.pPos.Xd - drone.pPos.X;

% Matriz de rotação
Rx = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
Ry = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
Rz = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];

R = Rz*Ry*Rx;

%-------------------------------
etax = drone.pPos.dXd(7) + Ganhos.kx1*tanh(Ganhos.kx2*drone.pPos.Xtil(1)) + Ganhos.kx3*tanh(Ganhos.kx4*drone.pPos.Xtil(7));
etay = drone.pPos.dXd(8) + Ganhos.ky1*tanh(Ganhos.ky2*drone.pPos.Xtil(2)) + Ganhos.ky3*tanh(Ganhos.ky4*drone.pPos.Xtil(8));
etaz = drone.pPos.dXd(9) + Ganhos.kz1*tanh(Ganhos.kz2*drone.pPos.Xtil(3)) + Ganhos.kz3*tanh(Ganhos.kz4*drone.pPos.Xtil(9));

etap = drone.pPos.dXd(10) + Ganhos.kp1*tanh(Ganhos.kp2*drone.pPos.Xtil(4)) + Ganhos.kp3*tanh(Ganhos.kp4*drone.pPos.Xtil(10));
etat = drone.pPos.dXd(11) + Ganhos.kt1*tanh(Ganhos.kt2*drone.pPos.Xtil(5)) + Ganhos.kt3*tanh(Ganhos.kt4*drone.pPos.Xtil(11));
etas = drone.pPos.dXd(12) + Ganhos.ks1*tanh(Ganhos.ks2*drone.pPos.Xtil(6)) + Ganhos.ks3*tanh(Ganhos.ks4*drone.pPos.Xtil(12));
%-------------------------------

% Referência de Rolagem e Arfagem (Inserir Filtragem)
drone.pPos.Xd(4) = atan2((etax*sin(drone.pPos.X(6))-etay*cos(drone.pPos.X(6)))*cos(drone.pPos.X(5)),(etaz+drone.pPar.g));
drone.pPos.Xd(5) = atan2((etax*cos(drone.pPos.X(6))+etay*sin(drone.pPos.X(6))),(etaz+drone.pPar.g));

% drone.mFiltroArfagemRolagem;

% Não linearidade inserida para que o valor máximo desejado de arfagem e
% rolagem seja de pi/4
if abs(drone.pPos.Xd(4)) > pi/4
    drone.pPos.Xd(4) = sign(drone.pPos.Xd(4))*pi/4;
end

if abs(drone.pPos.Xd(5)) > pi/4
    drone.pPos.Xd(5) = sign(drone.pPos.Xd(5))*pi/4;
end

% Ajuste para evitar problemas durante a passagem do segundo para o
% terceiro quadrante e vice-versa
if abs(drone.pPos.Xtil(6)) > pi
    drone.pPos.Xtil(6) = -2*pi + drone.pPos.Xtil(6);
end
if drone.pPos.Xtil(6) < -pi
    drone.pPos.Xtil(6) = 2*pi + drone.pPos.Xtil(6);
end

% FiltroOrientacao(Robo)

drone.pPos.Xd(10:11) = (drone.pPos.Xd(4:5) - drone.pPos.Xda(4:5))/drone.pPar.Ts;


% =========================================================================
% Parte Translacional
% Mt = R'*(drone.pPar.m+load.pPar.m)*eye(3,3); Alterado em 26/08
Mt = R'*(drone.pPar.m)*eye(3,3);
Ct = R'*zeros(3,3);
Gt = R'*[0; 0; drone.pPar.m*drone.pPar.g];

% =========================================================================
% Matriz de inércia rotacional
Mr =[drone.pPar.Ixx, ...
    drone.pPar.Ixy*cos(drone.pPos.X(4)) - drone.pPar.Ixz*sin(drone.pPos.X(4)), ...
    -drone.pPar.Ixx*sin(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5));
    
    drone.pPar.Ixy*cos(drone.pPos.X(4)) - drone.pPar.Ixz*sin(drone.pPos.X(4)), ...
    drone.pPar.Iyy*cos(drone.pPos.X(4))^2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)),...
    drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5));
    
    -drone.pPar.Ixx*sin(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)), ...
    drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5)),...
    drone.pPar.Ixx*sin(drone.pPos.X(5))^2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - 2*drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - 2*drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2
    ];

% Matriz de Coriolis e Forças Centrífugas rotacional
Cr =[ 0, ...
    drone.pPos.X(11)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(11)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2);
    
    drone.pPos.X(10)*(-drone.pPar.Ixy*sin(drone.pPos.X(4)) - drone.pPar.Ixz*cos(drone.pPos.X(4))) + drone.pPos.X(11)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2),...
    drone.pPos.X(10)*(drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)));
    
    drone.pPos.X(10)*(drone.pPar.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2),...
    drone.pPos.X(10)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))) + drone.pPos.X(12)*(drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2) + drone.pPos.X(11)*(drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)))
    ];

% Vetor Gravidade
Gr = [0; 0; 0];

% Retirada da Carga das matrizes para ser tratada como distúrbio

% Modelo no formato: \barM \ddot{q} + \barC \dot{q} + \barG = F
Z = zeros(3,3);

MM = [Mt Z; Z Mr];   % Matriz de Inércia

CC = [Ct Z; Z Cr];   % Matriz de Coriolis

GG = [Gt; Gr];       % Vetor de Forças Gravitacionais

mDa = -load.pPar.m*load.pPar.l*cos(load.pPos.X(5))*load.pPos.dX(11) + load.pPar.m*load.pPar.l*sin(load.pPos.X(5))*load.pPos.X(11)^2;
mDb = -load.pPar.m*load.pPar.l*cos(load.pPos.X(4))*load.pPos.dX(10) + load.pPar.m*load.pPar.l*sin(load.pPos.X(4))*load.pPos.X(10)^2;
mDc = + load.pPar.m*drone.pPar.g + load.pPar.m*load.pPar.l*(sin(load.pPos.X(5))*load.pPos.dX(11) + cos(load.pPos.X(5))*load.pPos.X(11)^2)+ load.pPar.m*load.pPar.l*(sin(load.pPos.X(4))*load.pPos.dX(10) + cos(load.pPos.X(4))*load.pPos.X(10)^2);

mD = [mDa;...
      mDb;...
      mDc;...
      0;...
      0;...
      0];
  
% Matriz de Acoplamento e Matriz dos Braços de Forcas
% [F1 F2 F3]' = R*At*[fx fy fz fytr]'
At = R*[0 0 0 0; 0 0 0 0; 1 1 1 1];

% [L M N]' = Ar*[fx fy fz fytr]'
Ar = [drone.pPar.k1  drone.pPar.k1 -drone.pPar.k1  -drone.pPar.k1;
    -drone.pPar.k1  drone.pPar.k1  drone.pPar.k1  -drone.pPar.k1;
    drone.pPar.k2 -drone.pPar.k2  drone.pPar.k2  -drone.pPar.k2];

A = [At;Ar];

% Matriz Pseudo-Inversa de A
As = pinv(A); %(A'*A)\A';

% Montagem da matriz sub-atuada ativa
% Matriz de Inérica
Ma = As*MM;
Map = Ma(:,1:2);
Maa = Ma(:,3:6);

% Matriz de Coriolis e Vetor de Forças Gravitacionais Ativa
Ea  = As*(CC*drone.pPos.X(7:12) + GG + mD);

% Escrita das matrizes passivas
Mpp =  drone.pPar.m*R(1:2,1:2);
Mpa = [drone.pPar.m*R(1:2,3) zeros(2,3)];

Ep = zeros(2,6)*drone.pPos.X(7:12) + drone.pPar.m*drone.pPar.g*R(1:2,3);
%==========================================================================
% Representação na forma sub-atuada
% M = [Mpp Mpa; Map Maa];
%
% E = [Ep; Ea];

D = Maa - Map/Mpp*Mpa;
H = Ea - Map/Mpp*Ep;

eta = [etaz; etap; etat; etas];

% Vetor de Forças de referência aplicado no referencial do veículo
Fr = D*eta + H;

% Verificando se forças sobre o referência do veículo
% ocrrem somente na direção Z
fTau = A*Fr;

% ------------------------------------
% Forçando valores possíveis: 30% do valor da gravidade
if real(fTau(3)) < 0
    fTau(3) = drone.pPar.m*drone.pPar.g*0.3;
end
% drone.pSC.fTau(1) = 0;
% drone.pSC.fTau(2) = 0;
% ------------------------------------

% Considerando a situação mais simples de que a força de propulsão
% solicitada aos motores é imediatamente atendida

% Modelo Inverso do Atuador
Fd = As*fTau;

% Caso a força do propulsor seja negativa, assume-se propulsão igual a zero
for ii = 1:4
    if Fr(ii) < 0
        Fr(ii) = 0;
    end
end

% 1: Fr -> Wd
Wda = drone.pSC.Wd;
drone.pSC.Wd = sqrt(Fd/drone.pPar.Cf);


% 2: Wd -> V % -8.65*ones(4,1)
% drone.Vo = 1/drone.Km*((drone.Bm*drone.R+drone.Km*drone.Kb)*sqrt(drone.Parametros.m*drone.Parametros.g/4/drone.Cf) + drone.R*drone.Ct*drone.Parametros.m*drone.Parametros.g/4/drone.Cf);
drone.pSC.Vo = (drone.pPar.R*drone.pPar.Bm/drone.pPar.Km + drone.pPar.Kb)*sqrt(drone.pPar.m*drone.pPar.g/4/drone.pPar.Cf) + drone.pPar.R*drone.pPar.Ct/drone.pPar.Km*drone.pPar.m*drone.pPar.g/4/drone.pPar.Cf;

% drone.Vr = - drone.Vo + 1/drone.Km*(drone.Jm*drone.R/drone.Ts*(drone.pSC.Wd-drone.pSC.Wd(:,drone.na-1)) + ...
%    (drone.Bm*drone.R+drone.Km*drone.Kb)*drone.pSC.Wd + drone.R*drone.Ct*drone.pSC.Wd.^2);
drone.pSC.Vr = - drone.pSC.Vo + drone.pPar.R/drone.pPar.Km*(drone.pPar.Jm*(drone.pSC.Wd-Wda)/drone.pPar.Ts + ...
    (drone.pPar.Bm+drone.pPar.Km*drone.pPar.Kb/drone.pPar.R)*drone.pSC.Wd + drone.pPar.Ct*Wda.^2);

% 3: V -> Xr
drone.pSC.Xr(4) = drone.pPos.X(4) + 1/(drone.pPar.kdp+drone.pPar.kpp*drone.pPar.Ts)*...
    (drone.pPar.kdp*(drone.pSC.Xr(4)-drone.pPos.Xa(4)) + 1/4*drone.pPar.Ts*([1 1 -1 -1]*drone.pSC.Vr));

drone.pSC.Xr(5) = drone.pPos.X(5) + 1/(drone.pPar.kdt+drone.pPar.kpt*drone.pPar.Ts)*...
    (drone.pPar.kdt*(drone.pSC.Xr(5)-drone.pPos.Xa(5)) + 1/4*drone.pPar.Ts*([-1 1 1 -1]*drone.pSC.Vr));

drone.pSC.Xr(12) = drone.pPos.X(12) + 1/(drone.pPar.kds+drone.pPar.kps*drone.pPar.Ts)*...
    (drone.pPar.kds*(drone.pSC.Xr(12)-drone.pPos.Xa(12)) + 1/4*drone.pPar.Ts*([1 -1 1 -1]*drone.pSC.Vr));

drone.pSC.Xr(9) = drone.pPos.X(9) + 1/(drone.pPar.kdz+drone.pPar.kpz*drone.pPar.Ts)*...
    (drone.pPar.kdz*(drone.pSC.Xr(9)-drone.pPos.Xa(9)) + 1/4*drone.pPar.Ts*([1 1 1 1]*drone.pSC.Vr));

% Modelo inverso do Joystick
% Comandos de referência enviados pelo joystick
drone.pSC.Ud(1) =  drone.pSC.Xr(4)/drone.pPar.uSat(1);   % Phi
drone.pSC.Ud(2) = -drone.pSC.Xr(5)/drone.pPar.uSat(2);   % Theta
drone.pSC.Ud(3) =  drone.pSC.Xr(9)/drone.pPar.uSat(3);   % dZ
drone.pSC.Ud(4) = -drone.pSC.Xr(12)/drone.pPar.uSat(4);  % dPsi


for ii = 1:4
    if abs(drone.pSC.Ud(ii)) > 1
        drone.pSC.Ud(ii) = sign(drone.pSC.Ud(ii));
    end
end

drone.pSC.Ud = tanh(drone.pSC.Ud);

end



% % Modelo da Carga
% 
% Mc = [0 ...
%     carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(5)); %m18
%     carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4)) ... %m27
%     0;
%     -carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5)) ... %m37
%     -carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))]; %m38
% 
% Mcc = [carga.pPar.Ic + carga.pPar.m*carga.pPar.l^2*(cos(carga.pPos.Xs(4)))^2 + carga.pPar.m*carga.pPar.l^2*(sin(carga.pPos.Xs(4)))^2*cos((carga.pPos.Xs(5)))^2 ...
%         carga.pPar.m*carga.pPar.l^2*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5));
%         carga.pPar.m*carga.pPar.l^2*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5))...
%         carga.pPar.Ic + carga.pPar.m*carga.pPar.l^2*(cos(carga.pPos.Xs(5)))^2 + carga.pPar.m*carga.pPar.l^2*(cos(carga.pPos.Xs(4)))^2*(sin(carga.pPos.Xs(5)))^2];
% 
% Cc = [0 ...
%     -carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(5))*carga.pPos.Xs(11); %c18
%     -carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*carga.pPos.Xs(10) ... %c27
%     0;
%     carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*carga.pPos.Xs(11) - carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(10) ... %c37
%     carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*carga.pPos.Xs(10) - carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(11)]; %c38
%     
% Ccc = [carga.pPar.m*carga.pPar.l^2*(-3 - cos(carga.pPos.Xs(5)) + 2*(cos(carga.pPos.Xs(5)))^2)*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*carga.pPos.Xs(10) - 2*carga.pPar.m*carga.pPar.l^2*(sin(carga.pPos.Xs(4)))^2*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(11) ...
%         carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*drone.pPos.X(9) - carga.pPar.m*carga.pPar.l^2*((sin(carga.pPos.Xs(4)))^2*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5)) + ((cos(carga.pPos.Xs(4)))^2-(sin(carga.pPos.Xs(4)))^2)*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5)))*carga.pPos.Xs(10) + carga.pPar.m*carga.pPar.l^2*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*((cos(carga.pPos.Xs(5)))^2-(sin(carga.pPos.Xs(5)))^2)*carga.pPos.Xs(11);
%         carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*drone.pPos.X(9) - carga.pPar.m*carga.pPar.l^2*((sin(carga.pPos.Xs(4)))^2*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5)) + ((cos(carga.pPos.Xs(4)))^2-(sin(carga.pPos.Xs(4)))^2)*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5)))*carga.pPos.Xs(10) + carga.pPar.m*carga.pPar.l^2*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*((cos(carga.pPos.Xs(5)))^2-(sin(carga.pPos.Xs(5)))^2)*carga.pPos.Xs(11)...
%         carga.pPar.m*carga.pPar.l^2*(-sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5))-(sin(carga.pPos.Xs(5)))^2+(cos(carga.pPos.Xs(5)))^2)*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*carga.pPos.Xs(10)   + carga.pPar.m*carga.pPar.l^2*(-3 + 2*(cos(carga.pPos.Xs(4)))^2)*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(11)];    
% 
% Gc = [carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*drone.pPar.g
%       carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*drone.pPar.g];
% 
% c37 = carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*carga.pPos.Xs(11) - carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(10); %c37
% c38 = carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*carga.pPos.Xs(10) - carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(11); %c38
% 
% DD = [carga.pPar.m*drone.pPos.dXs(7) + carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(5))*carga.pPos.dXs(11)  - carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(5))*carga.pPos.Xs(11)*carga.pPos.Xs(11);...
%       carga.pPar.m*drone.pPos.dXs(8) + carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*carga.pPos.dXs(10)  - carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*carga.pPos.Xs(10)*carga.pPos.Xs(10);...
%       carga.pPar.m*drone.pPos.dXs(9) + carga.pPar.m*drone.pPar.g - carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*carga.pPos.dX(10) - carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*carga.pPos.dXs(11) + c37*carga.pPos.Xs(10) + c38*carga.pPos.Xs(11);...
%       0;...
%       0;...
%       0];
%   
% % Modelo no formato: M \ddot{q} + C \dot{q} + G = F
% Z = zeros(3,3);
% Zt = zeros(3,2);
% 
% MMC = [Mt  Z   Mc;
%       Z   Mr  Zt;
%       Mc' Zt' Mcc];   % Matriz de Inércia Completa
% 
% CCC = [Ct   Z   Cc; 
%       Z   Cr  Zt;
%       Cc' Zt' Ccc];   % Matriz de Coriolis Completa
% 
% GGC = [Gt; Gr; Gc];    % Vetor de Forças Gravitacionais Completa
