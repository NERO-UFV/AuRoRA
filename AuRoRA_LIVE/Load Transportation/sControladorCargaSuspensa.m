function obj = sControladorCargaSuspensa(obj,carga)


if ~obj.pControlador.Ganhos.OK
    obj.mControladorSubatuadoGanhos;
end
%%
obj.pPos.Xda = obj.pPos.Xd;

% Calculando erro de posição
obj.pPos.Xtil = obj.pPos.Xd - obj.pPos.X;

% Matriz de rotação
Rx = [1 0 0; 0 cos(obj.pPos.Xs(4)) -sin(obj.pPos.Xs(4)); 0 sin(obj.pPos.Xs(4)) cos(obj.pPos.Xs(4))];
Ry = [cos(obj.pPos.Xs(5)) 0 sin(obj.pPos.Xs(5)); 0 1 0; -sin(obj.pPos.Xs(5)) 0 cos(obj.pPos.Xs(5))];
Rz = [cos(obj.pPos.Xs(6)) -sin(obj.pPos.Xs(6)) 0; sin(obj.pPos.Xs(6)) cos(obj.pPos.Xs(6)) 0; 0 0 1];

R = Rz*Ry*Rx;

%-------------------------------
etax = obj.pPos.dXd(7) + obj.pControlador.Ganhos.kx1*tanh(obj.pControlador.Ganhos.kx2*obj.pPos.Xtil(1)) + obj.pControlador.Ganhos.kx3*tanh(obj.pControlador.Ganhos.kx4*obj.pPos.Xtil(7));
etay = obj.pPos.dXd(8) + obj.pControlador.Ganhos.ky1*tanh(obj.pControlador.Ganhos.ky2*obj.pPos.Xtil(2)) + obj.pControlador.Ganhos.ky3*tanh(obj.pControlador.Ganhos.ky4*obj.pPos.Xtil(8));
etaz = obj.pPos.dXd(9) + obj.pControlador.Ganhos.kz1*tanh(obj.pControlador.Ganhos.kz2*obj.pPos.Xtil(3)) + obj.pControlador.Ganhos.kz3*tanh(obj.pControlador.Ganhos.kz4*obj.pPos.Xtil(9));

etap = obj.pPos.dXd(10) + obj.pControlador.Ganhos.kp1*tanh(obj.pControlador.Ganhos.kp2*obj.pPos.Xtil(4)) + obj.pControlador.Ganhos.kp3*tanh(obj.pControlador.Ganhos.kp4*obj.pPos.Xtil(10));
etat = obj.pPos.dXd(11) + obj.pControlador.Ganhos.kt1*tanh(obj.pControlador.Ganhos.kt2*obj.pPos.Xtil(5)) + obj.pControlador.Ganhos.kt3*tanh(obj.pControlador.Ganhos.kt4*obj.pPos.Xtil(11));
etas = obj.pPos.dXd(12) + obj.pControlador.Ganhos.ks1*tanh(obj.pControlador.Ganhos.ks2*obj.pPos.Xtil(6)) + obj.pControlador.Ganhos.ks3*tanh(obj.pControlador.Ganhos.ks4*obj.pPos.Xtil(12));
%-------------------------------

% Referência de Rolagem e Arfagem (Inserir Filtragem)
obj.pPos.Xd(4) = atan2((etax*sin(obj.pPos.Xs(6))-etay*cos(obj.pPos.Xs(6)))*cos(obj.pPos.Xs(5)),(etaz+obj.pPar.Corpo.g));
obj.pPos.Xd(5) = atan2((etax*cos(obj.pPos.Xs(6))+etay*sin(obj.pPos.Xs(6))),(etaz+obj.pPar.Corpo.g));

% obj.mFiltroArfagemRolagem;

% Não linearidade inserida para que o valor máximo desejado de arfagem e
% rolagem seja de pi/4
if abs(obj.pPos.Xd(4)) > pi/4
    obj.pPos.Xd(4) = sign(obj.pPos.Xd(4))*pi/4;
end

if abs(obj.pPos.Xd(5)) > pi/4
    obj.pPos.Xd(5) = sign(obj.pPos.Xd(5))*pi/4;
end

% Ajuste para evitar problemas durante a passagem do segundo para o
% terceiro quadrante e vice-versa
if abs(obj.pPos.Xtil(6)) > pi
    obj.pPos.Xtil(6) = -2*pi + obj.pPos.Xtil(6);
end
if obj.pPos.Xtil(6) < -pi
    obj.pPos.Xtil(6) = 2*pi + obj.pPos.Xtil(6);
end

% FiltroOrientacao(Robo)

obj.pPos.Xd(10:11) = (obj.pPos.Xd(4:5) - obj.pPos.Xda(4:5))/obj.pTempo.Ts;


% =========================================================================
% Parte Translacional
Mt = R'*(obj.pPar.Corpo.m+carga.pPar.m)*eye(3,3);
Ct = R'*zeros(3,3);
Gt = R'*[0; 0; obj.pPar.Corpo.m*obj.pPar.Corpo.g];

% =========================================================================
% Matriz de inércia rotacional
Mr =[obj.pPar.Corpo.Ixx, ...
    obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4)) - obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4)), ...
    -obj.pPar.Corpo.Ixx*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5));
    
    obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4)) - obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4)), ...
    obj.pPar.Corpo.Iyy*cos(obj.pPos.Xs(4))^2 + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))^2 - 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4)),...
    obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5));
    
    -obj.pPar.Corpo.Ixx*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)), ...
    obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5)),...
    obj.pPar.Corpo.Ixx*sin(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))^2 - 2*obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - 2*obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) + 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2
    ];

% Matriz de Coriolis e Forças Centrífugas rotacional
Cr =[ 0, ...
    obj.pPos.Xs(11)*(obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4)) + obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2 - obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2) + obj.pPos.Xs(12)*(-obj.pPar.Corpo.Ixx*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Iyy*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))),...
    obj.pPos.Xs(11)*(-obj.pPar.Corpo.Ixx*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Iyy*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))) + obj.pPos.Xs(12)*(-obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))^2);
    
    obj.pPos.Xs(10)*(-obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4)) - obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))) + obj.pPos.Xs(11)*(-obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4)) + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4)) - obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2 + obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2) + obj.pPos.Xs(12)*(obj.pPar.Corpo.Ixx*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Iyy*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) - 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))),...
    obj.pPos.Xs(10)*(-obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4)) + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4)) - obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2 + obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2),...
    obj.pPos.Xs(10)*(obj.pPar.Corpo.Ixx*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Iyy*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) - 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))) + obj.pPos.Xs(12)*(-obj.pPar.Corpo.Ixx*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))^2 + 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)));
    
    obj.pPos.Xs(10)*(obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))) + obj.pPos.Xs(11)*(-obj.pPar.Corpo.Ixx*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Iyy*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))) + obj.pPos.Xs(12)*(obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))^2),...
    obj.pPos.Xs(10)*(-obj.pPar.Corpo.Ixx*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Iyy*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))/2 - 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))) + obj.pPos.Xs(11)*(-obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) + obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2*sin(obj.pPos.Xs(5)) - obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2*sin(obj.pPos.Xs(5))) + obj.pPos.Xs(12)*(obj.pPar.Corpo.Ixx*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))^2 - 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5))),...
    obj.pPos.Xs(10)*(obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Izz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Ixy*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Ixz*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) + obj.pPar.Corpo.Iyz*cos(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))^2*cos(obj.pPos.Xs(5))^2) + obj.pPos.Xs(11)*(obj.pPar.Corpo.Ixx*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Iyy*sin(obj.pPos.Xs(4))^2*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Izz*cos(obj.pPos.Xs(4))^2*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)) - obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Ixy*sin(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))^2 - obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*cos(obj.pPos.Xs(5))^2 + obj.pPar.Corpo.Ixz*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))^2 - 2*obj.pPar.Corpo.Iyz*sin(obj.pPos.Xs(4))*cos(obj.pPos.Xs(4))*sin(obj.pPos.Xs(5))*cos(obj.pPos.Xs(5)))
    ];

% Vetor Gravidade
Gr = [0; 0; 0];

% Retirada da Carga das matrizes para ser tratada como distúrbio

% Modelo no formato: \barM \ddot{q} + \barC \dot{q} + \barG = F
Z = zeros(3,3);

MM = [Mt Z; Z Mr];   % Matriz de Inércia

CC = [Ct Z; Z Cr];   % Matriz de Coriolis

GG = [Gt; Gr];       % Vetor de Forças Gravitacionais

% Modelo da carga de forma simplificada
% mDa = carga.pPar.m*obj.pPos.dXs(7) -carga.pPar.m*carga.pPar.l*cos(carga.pPos.X(5))*carga.pPos.dX(11) + carga.pPar.m*carga.pPar.l*sin(carga.pPos.X(5))*carga.pPos.X(11)^2;
% mDb = carga.pPar.m*obj.pPos.dXs(8) -carga.pPar.m*carga.pPar.l*cos(carga.pPos.X(4))*carga.pPos.dX(10) + carga.pPar.m*carga.pPar.l*sin(carga.pPos.X(4))*carga.pPos.X(10)^2;
% mDc = carga.pPar.m*obj.pPos.dXs(9) + carga.pPar.m*obj.pPar.Corpo.g + carga.pPar.m*carga.pPar.l*(sin(carga.pPos.X(5))*carga.pPos.dX(11) + cos(carga.pPos.X(5))*carga.pPos.X(11)^2)+ carga.pPar.m*carga.pPar.l*(sin(carga.pPos.X(4))*carga.pPos.dX(10) + cos(carga.pPos.X(4))*carga.pPos.X(10)^2);

mDa = -carga.pPar.m*carga.pPar.l*cos(carga.pPos.X(5))*carga.pPos.dX(11) + carga.pPar.m*carga.pPar.l*sin(carga.pPos.X(5))*carga.pPos.X(11)^2;
mDb = -carga.pPar.m*carga.pPar.l*cos(carga.pPos.X(4))*carga.pPos.dX(10) + carga.pPar.m*carga.pPar.l*sin(carga.pPos.X(4))*carga.pPos.X(10)^2;
mDc = + carga.pPar.m*obj.pPar.Corpo.g + carga.pPar.m*carga.pPar.l*(sin(carga.pPos.X(5))*carga.pPos.dX(11) + cos(carga.pPos.X(5))*carga.pPos.X(11)^2)+ carga.pPar.m*carga.pPar.l*(sin(carga.pPos.X(4))*carga.pPos.dX(10) + cos(carga.pPos.X(4))*carga.pPos.X(10)^2);

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
Ar = [obj.pPar.Propulsor.k1  obj.pPar.Propulsor.k1 -obj.pPar.Propulsor.k1  -obj.pPar.Propulsor.k1;
    -obj.pPar.Propulsor.k1  obj.pPar.Propulsor.k1  obj.pPar.Propulsor.k1  -obj.pPar.Propulsor.k1;
    obj.pPar.Propulsor.k2 -obj.pPar.Propulsor.k2  obj.pPar.Propulsor.k2  -obj.pPar.Propulsor.k2];

A = [At;Ar];

% Matriz Pseudo-Inversa de A
As = pinv(A); %(A'*A)\A';

% Montagem da matriz sub-atuada ativa
% Matriz de Inérica
Ma = As*MM;
Map = Ma(:,1:2);
Maa = Ma(:,3:6);

% Matriz de Coriolis e Vetor de Forças Gravitacionais Ativa
Ea  = As*(CC*obj.pPos.Xs(7:12) + GG + mD);

% Escrita das matrizes passivas
Mpp =  obj.pPar.Corpo.m*R(1:2,1:2);
Mpa = [obj.pPar.Corpo.m*R(1:2,3) zeros(2,3)];

Ep = zeros(2,6)*obj.pPos.Xs(7:12) + obj.pPar.Corpo.m*obj.pPar.Corpo.g*R(1:2,3);
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
obj.pSC.fTau = A*Fr;

% ------------------------------------
% Forçando valores possíveis: 30% do valor da gravidade
if real(obj.pSC.fTau(3)) < 0
    obj.pSC.fTau(3) = obj.pPar.Corpo.m*obj.pPar.Corpo.g*0.3;
end
% obj.pSC.fTau(1) = 0;
% obj.pSC.fTau(2) = 0;
% ------------------------------------

% Considerando a situação mais simples de que a força de propulsão
% solicitada aos motores é imediatamente atendida

% Modelo Inverso do Atuador
obj.pSC.Fr = As*obj.pSC.fTau;

% Caso a força do propulsor seja negativa, assume-se propulsão igual a zero
for ii = 1:4
    if obj.pSC.Fr(ii) < 0
        obj.pSC.Fr(ii) = 0;
    end
end

% 1: Fr -> Wr
obj.pSC.Wra = obj.pSC.Wr;
obj.pSC.Wr = sqrt(obj.pSC.Fr/obj.pPar.Atuador.Cf);


% 2: Wr -> V % -8.65*ones(4,1)
% obj.Vo = 1/obj.Atuador.Km*((obj.Atuador.Bm*obj.Atuador.R+obj.Atuador.Km*obj.Atuador.Kb)*sqrt(obj.Parametros.m*obj.Parametros.g/4/obj.Atuador.Cf) + obj.Atuador.R*obj.Atuador.Ct*obj.Parametros.m*obj.Parametros.g/4/obj.Atuador.Cf);
obj.pSC.Vo = (obj.pPar.Atuador.R*obj.pPar.Atuador.Bm/obj.pPar.Atuador.Km + obj.pPar.Atuador.Kb)*sqrt(obj.pPar.Corpo.m*obj.pPar.Corpo.g/4/obj.pPar.Atuador.Cf) + obj.pPar.Atuador.R*obj.pPar.Atuador.Ct/obj.pPar.Atuador.Km*obj.pPar.Corpo.m*obj.pPar.Corpo.g/4/obj.pPar.Atuador.Cf;

% obj.Vr = - obj.Vo + 1/obj.Atuador.Km*(obj.Atuador.Jm*obj.Atuador.R/obj.Ts*(obj.pSC.Wr-obj.pSC.Wr(:,obj.na-1)) + ...
%    (obj.Atuador.Bm*obj.Atuador.R+obj.Atuador.Km*obj.Atuador.Kb)*obj.pSC.Wr + obj.Atuador.R*obj.Atuador.Ct*obj.pSC.Wr.^2);
obj.pSC.Vr = - obj.pSC.Vo + obj.pPar.Atuador.R/obj.pPar.Atuador.Km*(obj.pPar.Atuador.Jm*(obj.pSC.Wr-obj.pSC.Wra)/obj.pTempo.Ts + ...
    (obj.pPar.Atuador.Bm+obj.pPar.Atuador.Km*obj.pPar.Atuador.Kb/obj.pPar.Atuador.R)*obj.pSC.Wr + obj.pPar.Atuador.Ct*obj.pSC.Wra.^2);

% 3: V -> Xr
obj.pPos.Xr(4) = obj.pPos.Xs(4) + 1/(obj.pPar.Atuador.kdp+obj.pPar.Atuador.kpp*obj.pTempo.Ts)*...
    (obj.pPar.Atuador.kdp*(obj.pPos.Xr(4)-obj.pPos.Xsa(4)) + 1/4*obj.pTempo.Ts*([1 1 -1 -1]*obj.pSC.Vr));

obj.pPos.Xr(5) = obj.pPos.Xs(5) + 1/(obj.pPar.Atuador.kdt+obj.pPar.Atuador.kpt*obj.pTempo.Ts)*...
    (obj.pPar.Atuador.kdt*(obj.pPos.Xr(5)-obj.pPos.Xsa(5)) + 1/4*obj.pTempo.Ts*([-1 1 1 -1]*obj.pSC.Vr));

obj.pPos.Xr(12) = obj.pPos.Xs(12) + 1/(obj.pPar.Atuador.kds+obj.pPar.Atuador.kps*obj.pTempo.Ts)*...
    (obj.pPar.Atuador.kds*(obj.pPos.Xr(12)-obj.pPos.Xsa(12)) + 1/4*obj.pTempo.Ts*([1 -1 1 -1]*obj.pSC.Vr));

obj.pPos.Xr(9) = obj.pPos.Xs(9) + 1/(obj.pPar.Atuador.kdz+obj.pPar.Atuador.kpz*obj.pTempo.Ts)*...
    (obj.pPar.Atuador.kdz*(obj.pPos.Xr(9)-obj.pPos.Xsa(9)) + 1/4*obj.pTempo.Ts*([1 1 1 1]*obj.pSC.Vr));

% Modelo inverso do Joystick
% Comandos de referência enviados pelo joystick
obj.pSC.Joystick.Ar(1) =  -obj.pPos.Xr(5)/obj.pPar.Joystick.AngMax;
obj.pSC.Joystick.Ar(2) =   obj.pPos.Xr(4)/obj.pPar.Joystick.AngMax;
obj.pSC.Joystick.Ar(3) =  -obj.pPos.Xr(12)/obj.pPar.Joystick.MaxRatePsi;
obj.pSC.Joystick.Ar(4) =   obj.pPos.Xr(9)/obj.pPar.Joystick.MaxRateZ;


for ii = 1:4
    if abs(obj.pSC.Joystick.Ar(ii)) > 1
        obj.pSC.Joystick.Ar(ii) = sign(obj.pSC.Joystick.Ar(ii));
    end
end



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
%         carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*obj.pPos.Xs(9) - carga.pPar.m*carga.pPar.l^2*((sin(carga.pPos.Xs(4)))^2*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5)) + ((cos(carga.pPos.Xs(4)))^2-(sin(carga.pPos.Xs(4)))^2)*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5)))*carga.pPos.Xs(10) + carga.pPar.m*carga.pPar.l^2*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*((cos(carga.pPos.Xs(5)))^2-(sin(carga.pPos.Xs(5)))^2)*carga.pPos.Xs(11);
%         carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*obj.pPos.Xs(9) - carga.pPar.m*carga.pPar.l^2*((sin(carga.pPos.Xs(4)))^2*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5)) + ((cos(carga.pPos.Xs(4)))^2-(sin(carga.pPos.Xs(4)))^2)*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5)))*carga.pPos.Xs(10) + carga.pPar.m*carga.pPar.l^2*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*((cos(carga.pPos.Xs(5)))^2-(sin(carga.pPos.Xs(5)))^2)*carga.pPos.Xs(11)...
%         carga.pPar.m*carga.pPar.l^2*(-sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5))-(sin(carga.pPos.Xs(5)))^2+(cos(carga.pPos.Xs(5)))^2)*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(4))*carga.pPos.Xs(10)   + carga.pPar.m*carga.pPar.l^2*(-3 + 2*(cos(carga.pPos.Xs(4)))^2)*sin(carga.pPos.Xs(5))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(11)];    
% 
% Gc = [carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*obj.pPar.Corpo.g
%       carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*obj.pPar.Corpo.g];
% 
% c37 = carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*carga.pPos.Xs(11) - carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(10); %c37
% c38 = carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*carga.pPos.Xs(10) - carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*carga.pPos.Xs(11); %c38
% 
% DD = [carga.pPar.m*obj.pPos.dXs(7) + carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(5))*carga.pPos.dXs(11)  - carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(5))*carga.pPos.Xs(11)*carga.pPos.Xs(11);...
%       carga.pPar.m*obj.pPos.dXs(8) + carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*carga.pPos.dXs(10)  - carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*carga.pPos.Xs(10)*carga.pPos.Xs(10);...
%       carga.pPar.m*obj.pPos.dXs(9) + carga.pPar.m*obj.pPar.Corpo.g - carga.pPar.m*carga.pPar.l*sin(carga.pPos.Xs(4))*cos(carga.pPos.Xs(5))*carga.pPos.dX(10) - carga.pPar.m*carga.pPar.l*cos(carga.pPos.Xs(4))*sin(carga.pPos.Xs(5))*carga.pPos.dXs(11) + c37*carga.pPos.Xs(10) + c38*carga.pPos.Xs(11);...
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
