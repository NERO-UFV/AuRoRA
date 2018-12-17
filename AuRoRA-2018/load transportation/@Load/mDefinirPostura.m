function mDefinirPostura(carga,drone)

% mDefinirPostura determina a postura do ArDrone baseado em seu modelo
% dinâmico obtido segundo as equações de Euler Lagrange

% =================================================================
% =================================================================
% comando pitch (desde Matlab, entre -1 y 1, +-12 grados)
% comando roll (desde Matlab, entre -1 y 1, +-12 grados)
% comando yaw (desde Matlab, entre -1 y 1, +-100 grados/segundo)
% comando gaz (desde Matlab, entre -1 y 1, +- 0.7 m/s)

% Robo = ArDrone_Joystick(Robo)
% Esta funcao recebe os comandos obtidos através de um joystick e envia
% como parametros de referencia para o ArDrone
%
% Os comando sao enviados através da alavancas analogicas e sao dados por
% [pitch roll yaw_rate z_dot]
%
% Os comandos do joystick são normalizados entre [-1 1], que equivalem
% pitch:   [-12 12] graus
% roll:    [-12 12] graus
% dot_yaw: [-10 10] graus/s
% dot_z:   [0.7 0.7] m/s

% Os dados de referencia enviados pelo Joystick sao representados no
% sistema de referencia da aeronave
%%
% Comandos de referência enviados pelo joystick

drone.pPar.Xra = drone.pPar.Xr;

drone.pPar.Xr(4)  =  drone.pSC.Ud(1)*drone.pPar.uSat(1);
drone.pPar.Xr(5)  = -drone.pSC.Ud(2)*drone.pPar.uSat(2);
drone.pPar.Xr(9)  =  drone.pSC.Ud(3)*drone.pPar.uSat(3);
drone.pPar.Xr(12) = -drone.pSC.Ud(4)*drone.pPar.uSat(4);

% Recebe erros de referências e retorna força a ser aplicada ao modelo de
% corpo rígido do veículo

% 1: erro -> V
drone.pSC.Vor = (drone.pPar.Atuador.R*drone.pPar.Atuador.Bm/drone.pPar.Atuador.Km + drone.pPar.Atuador.Kb)*...
    sqrt(drone.pPar.Corpo.m*drone.pPar.Corpo.g/4/drone.pPar.Atuador.Cf) + ...
    drone.pPar.Atuador.R*drone.pPar.Atuador.Ct/drone.pPar.Atuador.Km*drone.pPar.Corpo.m*drone.pPar.Corpo.g/4/drone.pPar.Atuador.Cf;

drone.pSC.V = drone.pSC.Vor + [1 -1 1 1; 1 1 -1 1; -1 1 1 1; -1 -1 -1 1]*...
    [drone.pPar.Atuador.kdp*(drone.pPar.Xr(4)-drone.pPos.X(4)  - drone.pPar.Xra(4) +drone.pPos.Xa(4) )/drone.pTempo.Ts  + drone.pPar.Atuador.kpp*(drone.pPar.Xr(4)-drone.pPos.X(4));
    drone.pPar.Atuador.kdt*(drone.pPar.Xr(5) -drone.pPos.X(5)  - drone.pPar.Xra(5) +drone.pPos.Xa(5) )/drone.pTempo.Ts  + drone.pPar.Atuador.kpt*(drone.pPar.Xr(5)-drone.pPos.X(5));
    drone.pPar.Atuador.kds*(drone.pPar.Xr(12)-drone.pPos.X(12) - drone.pPar.Xra(12)+drone.pPos.Xa(12))/drone.pTempo.Ts  + drone.pPar.Atuador.kps*(drone.pPar.Xr(12)-drone.pPos.X(12));
    drone.pPar.Atuador.kdz*(drone.pPar.Xr(9) -drone.pPos.X(9)  - drone.pPar.Xra(9) +drone.pPos.Xa(9) )/drone.pTempo.Ts  + drone.pPar.Atuador.kpz*(drone.pPar.Xr(9)-drone.pPos.X(9))];

% Saturação dos motores dado pelos limites da fonte de energia
drone.pSC.V = (drone.pSC.V>0).*drone.pSC.V;
drone.pSC.V = (drone.pSC.V<=11.1).*drone.pSC.V + (drone.pSC.V>11.1).*11.1;

% 2: V -> W
%obo.W = 1/(obj.AtuadorRef.Jm*obj.AtuadorRef.R+obj.Ts*(obj.AtuadorRef.Bm*obj.AtuadorRef.R+obj.AtuadorRef.Km*obj.AtuadorRef.Kb))*...
%     (obj.AtuadorRef.Jm*obj.AtuadorRef.R*obj.W(:,obj.na-1)+obj.Ts*(obj.AtuadorRef.Km*obj.V)-obj.Ts*obj.Atuador.R*obj.Atuador.Ct*obj.W(:,obj.na-1).^2);
drone.pSC.W = 1/(drone.pPar.Atuador.Jm+drone.pTempo.Ts*(drone.pPar.Atuador.Bm+drone.pPar.Atuador.Km*drone.pPar.Atuador.Kb/drone.pPar.Atuador.R))*...
    (drone.pPar.Atuador.Jm*drone.pSC.Wa+drone.pTempo.Ts*(drone.pPar.Atuador.Km/drone.pPar.Atuador.R*drone.pSC.V-drone.pPar.Atuador.Ct*drone.pSC.Wa.^2));
drone.pSC.dW = (drone.pSC.W - drone.pSC.Wa)/drone.pTempo.Ts;

% 3: W -> F
% Deslocando valores passados
drone.pSC.F = drone.pPar.Atuador.Cf*drone.pSC.W.^2;
drone.pSC.dF = (drone.pSC.F-drone.pSC.Fa)/drone.pTempo.Ts;

% Utilizando modelo de Euler-Lagrange
drone.pSC.Fa = drone.pSC.F;
drone.pPos.Xa = drone.pPos.X;

Rx = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
Ry = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
Rz = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];

R = Rz*Ry*Rx;

% W = [1 0 -sin(obj.pPos.X(5)); 0 cos(obj.pPos.X(4)) sin(obj.pPos.X(4))*cos(obj.pPos.X(5)); 0 -sin(obj.pPos.X(4)) cos(obj.pPos.X(4))*cos(obj.pPos.X(5))];

% =========================================================================
% Obtendo a postura do helicóptero
% Matriz de inércia translacional
Mt = (drone.pPar.Corpo.m+carga.pPar.m)*eye(3,3);

% Vetor de forças gravitacionais
G = [0; 0; drone.pPar.Corpo.m*drone.pPar.Corpo.g];


mDa = -carga.pPar.m*carga.pPar.l*cos(carga.pPos.X(5))*carga.pPos.dX(11) + carga.pPar.m*carga.pPar.l*sin(carga.pPos.X(5))*carga.pPos.X(11)^2;
mDb = -carga.pPar.m*carga.pPar.l*cos(carga.pPos.X(4))*carga.pPos.dX(10) + carga.pPar.m*carga.pPar.l*sin(carga.pPos.X(4))*carga.pPos.X(10)^2;
mDc = + carga.pPar.m*drone.pPar.Corpo.g + carga.pPar.m*carga.pPar.l*(sin(carga.pPos.X(5))*carga.pPos.dX(11) + cos(carga.pPos.X(5))*carga.pPos.X(11)^2)+ carga.pPar.m*carga.pPar.l*(sin(carga.pPos.X(4))*carga.pPos.dX(10) + cos(carga.pPos.X(4))*carga.pPos.X(10)^2);

drone.pSC.D = [mDa;...
      mDb;...
      mDc;];
  
  if carga.pPos.X(3) <= 0
      drone.pSC.D = [0;0;0];
  end

% ArDrone
At = [0 0 0 0; 0 0 0 0; 1 1 1 1];

% Vetor de distúrbios adicionado ao vetor de forças que atuam no veículo
ft = R*At*drone.pSC.F - drone.pSC.D;

% mDa = carga.pPar.m*obj.pPos.dX(7) -carga.pPar.m*carga.pPar.l*cos(carga.pPos.X(5))*carga.pPos.dX(11) + carga.pPar.m*carga.pPar.l*sin(carga.pPos.X(5))*carga.pPos.X(11)^2;
% mDb = carga.pPar.m*obj.pPos.dX(8) -carga.pPar.m*carga.pPar.l*cos(carga.pPos.X(4))*carga.pPos.dX(10) + carga.pPar.m*carga.pPar.l*sin(carga.pPos.X(4))*carga.pPos.X(10)^2;
% mDc = carga.pPar.m*obj.pPos.dX(9) + carga.pPar.m*obj.pPar.Corpo.g + carga.pPar.m*carga.pPar.l*(sin(carga.pPos.X(5))*carga.pPos.dX(11) + cos(carga.pPos.X(5))*carga.pPos.X(11)^2)+ carga.pPar.m*carga.pPar.l*(sin(carga.pPos.X(4))*carga.pPos.dX(10) + cos(carga.pPos.X(4))*carga.pPos.X(10)^2);


  
% mD = [0;...
%       0;...
%       0;];
  
% Integracao númerica paras velocides cartesianas
drone.pPos.X(7:9) = Mt\(ft - G)*drone.pTempo.Ts + drone.pPos.X(7:9);
disp(drone.pPos.X(7:9)')
% =========================================================================
% Matriz de inércia rotacional
Mr = [drone.pPar.Corpo.Ixx, ...
    drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4)) - drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4)), ...
    -drone.pPar.Corpo.Ixx*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5));
    
    drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4)) - drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4)), ...
    drone.pPar.Corpo.Iyy*cos(drone.pPos.X(4))^2 + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))^2 - 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)),...
    drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5));
    
    -drone.pPar.Corpo.Ixx*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)), ...
    drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5)),...
    drone.pPar.Corpo.Ixx*sin(drone.pPos.X(5))^2 + drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - 2*drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - 2*drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2
    ];

% Matriz de Coriolis e Forças Centrífugas rotacional
Cr = [ 0, ...
    drone.pPos.X(11)*(drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2 - drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(-drone.pPar.Corpo.Ixx*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(11)*(-drone.pPar.Corpo.Ixx*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2);
    
    drone.pPos.X(10)*(-drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4)) - drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))) + drone.pPos.X(11)*(-drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2 + drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(drone.pPar.Corpo.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(-drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2 + drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2),...
    drone.pPos.X(10)*(drone.pPar.Corpo.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-drone.pPar.Corpo.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)));
    
    drone.pPos.X(10)*(drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-drone.pPar.Corpo.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2),...
    drone.pPos.X(10)*(-drone.pPar.Corpo.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5)) - drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))) + drone.pPos.X(12)*(drone.pPar.Corpo.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Corpo.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Corpo.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Corpo.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2) + drone.pPos.X(11)*(drone.pPar.Corpo.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Corpo.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Corpo.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*drone.pPar.Corpo.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)))
    ];

% Matriz de acoplamento rotacional

% ArDrone
Ar = [drone.pPar.Propulsor.k1  drone.pPar.Propulsor.k1 -drone.pPar.Propulsor.k1  -drone.pPar.Propulsor.k1;
    -drone.pPar.Propulsor.k1  drone.pPar.Propulsor.k1  drone.pPar.Propulsor.k1  -drone.pPar.Propulsor.k1;
    drone.pPar.Propulsor.k2 -drone.pPar.Propulsor.k2  drone.pPar.Propulsor.k2  -drone.pPar.Propulsor.k2];

% Momento devido ao arraste aerodinâmico das pás do rotor
T = Ar*drone.pSC.F + drone.pSC.Q;

%--------------------------------------------
% Integração discreta do movimento de rotação
drone.pPos.X(10:12) = Mr\(T - Cr*drone.pPos.X(10:12))*drone.pTempo.Ts + drone.pPos.X(10:12);

% Postura do ArDrone - Integracao discreta
for ii = 1:6
    drone.pPos.X(ii) = drone.pPos.X(ii+6)*drone.pTempo.Ts + drone.pPos.X(ii);
    if ii > 3
        if drone.pPos.X(ii) > pi
            drone.pPos.X(ii) = -2*pi + drone.pPos.X(ii);
        end
        if drone.pPos.X(ii) < -pi
            drone.pPos.X(ii) = 2*pi + drone.pPos.X(ii);
        end
    end
end

% Velocidade angular do veiculo com respeito ao proprio sistema de
% referencia do ArDrone
drone.pPos.Y = [1 0 -sin(drone.pPos.X(5));
    0 cos(drone.pPos.X(4)) sin(drone.pPos.X(4))*cos(drone.pPos.X(5));
    0 -sin(drone.pPos.X(4)) cos(drone.pPos.X(4))*cos(drone.pPos.X(5))]*drone.pPos.X(10:12);

if drone.pPos.X(3) < 0
    drone.pPos.X(3) = 0;
end

%% Postura da carga
drone.pPos.dX = (drone.pPos.X - drone.pPos.Xa)/drone.pTempo.Ts;


carga.pPos.dX(11) = 1/(carga.pPar.Iyy+carga.pPar.m*carga.pPar.l^2)*(carga.pPar.m*carga.pPar.l*(...
    cos(carga.pPos.X(5))*( drone.pPos.dX(7) - 2*carga.pPos.X(11)*drone.pPos.X(9)) + ...
    sin(carga.pPos.X(5))*(-drone.pPos.dX(9) - 2*carga.pPos.X(11)*drone.pPos.X(7) - drone.pPar.Corpo.g)));

carga.pPos.X(11) = carga.pPos.X(11) + carga.pPos.dX(11)*drone.pTempo.Ts;
carga.pPos.X(5) = carga.pPos.X(5) + carga.pPos.X(11)*drone.pTempo.Ts;


carga.pPos.dX(10) = 1/(carga.pPar.Iyy+carga.pPar.m*carga.pPar.l^2)*(carga.pPar.m*carga.pPar.l*(...
    cos(carga.pPos.X(4))*( drone.pPos.dX(8) - 2*carga.pPos.X(10)*drone.pPos.X(9)) + ...
    sin(carga.pPos.X(4))*(-drone.pPos.dX(9) - 2*carga.pPos.X(10)*drone.pPos.X(8) - drone.pPar.Corpo.g)));

carga.pPos.X(10) = carga.pPos.X(10) + carga.pPos.dX(10)*drone.pTempo.Ts;
carga.pPos.X(4) = carga.pPos.X(4) + carga.pPos.X(10)*drone.pTempo.Ts;


% Postura da carga
carga.pPos.X(1) = drone.pPos.X(1) - carga.pPar.l*sin(carga.pPos.X(5));
carga.pPos.X(2) = drone.pPos.X(2) - carga.pPar.l*sin(carga.pPos.X(4));
carga.pPos.X(3) = drone.pPos.X(3) - carga.pPar.l*cos(carga.pPos.X(4))*cos(carga.pPos.X(5));

if carga.pPos.X(3) < 0
    carga.pPos.X(3) = 0;
end

end