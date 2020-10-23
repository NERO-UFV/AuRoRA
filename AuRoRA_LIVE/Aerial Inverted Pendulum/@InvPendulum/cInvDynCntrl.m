function cInvDynCntrl(p,Hcgains)
%UAVwInvPend Control algorithm for an inverted p on p
%   Guarantees stability for a inverted aerial p. 
if nargin < 2
    Hcgains = [-2 -5 2 5 2 .5 2 .5;-15 -1 15 1 2.5  1 20 1];
    disp('Gains not given. Using default.');
end

%% GAINLOGS                                                                          a  b  ph  th ps x  y  z  a  b  ph th  ps x  y z 
% Gains from '21/01/19 17:10' ->                                           cgains = [1  1  1   1  1  1  1  1; 1  1  1  1   1  1  1 1];  
Kp = Hcgains(1,:);                                                                           
Kd = Hcgains(2,:);
   
%% Dynamic model
% Active variables:
q1 = p.pPos.Q(1); % alpha
q2 = p.pPos.Q(2); % beta
q3 = p.pPos.Q(3); % phi
q4 = p.pPos.Q(4); % theta

% Constants:
g  = p.pPar.g; 
m  = p.pPar.m;
mc = p.pPar.mc;
r  = p.pPar.r;

Ix = p.pPar.Ix; 
Iy = p.pPar.Iy;
Iz = p.pPar.Iz;
I  = p.pPar.I;

% Rotation matrix:
RotX = [1 0 0; 0 cos(p.pPos.Q(3)) -sin(p.pPos.Q(3)); 0 sin(p.pPos.Q(3)) cos(p.pPos.Q(3))];
RotY = [cos(p.pPos.Q(4)) 0 sin(p.pPos.Q(4)); 0 1 0; -sin(p.pPos.Q(4)) 0 cos(p.pPos.Q(4))];
RotZ = [cos(p.pPos.Q(5)) -sin(p.pPos.Q(5)) 0; sin(p.pPos.Q(5)) cos(p.pPos.Q(5)) 0; 0 0 1];

R = RotZ*RotY*RotX;

% Inertia matrix:
M = [...                                                                                            
  (I*sin(q1)^2 - I + I*sin(q2)^2 - 2*mc*r^2*cos(q1)^2 + mc*r^2*cos(q1)^2*sin(q1)^2 + mc*r^2*cos(q1)^2*sin(q2)^2)/(sin(q1)^2 + sin(q2)^2 - 1),                    0,           0,                                  0,                                                              0, mc*r*cos(q1),            0, -(2^(1/2)*mc*r*cos(q1))/(cos(2*q1) + cos(2*q2))^(1/2); ...
                                                                                                                                           0, I + mc*r^2*cos(q2)^2,           0,                                  0,                                                              0,            0, mc*r*cos(q2),                                                     0; ...
                                                                                                                                           0,                    0,          Ix,                                  0,                                                    -Ix*sin(q4),            0,            0,                                                     0; ...
                                                                                                                                           0,                    0,           0,   Iy - Iy*sin(q3)^2 + Iz*sin(q3)^2,                             -cos(q3)*cos(q4)*sin(q3)*(Iy + Iz),            0,            0,                                                     0; ...
                                                                                                                                           0,                    0, -Ix*sin(q4), -cos(q3)*cos(q4)*sin(q3)*(Iy + Iz), Iz*cos(q3)^2*cos(q4)^2 + Iy*cos(q4)^2*sin(q3)^2 + Ix*sin(q4)^2,            0,            0,                                                     0; ...
                                                                                                                                mc*r*cos(q1),                    0,           0,                                  0,                                                              0,       m + mc,            0,                                                     0; ...
                                                                                                                                           0,         mc*r*cos(q2),           0,                                  0,                                                              0,            0,       m + mc,                                                     0; ...
                                                                                       -(2^(1/2)*mc*r*cos(q1))/(cos(2*q1) + cos(2*q2))^(1/2),                    0,           0,                                  0,                                                              0,            0,            0,                                                m + mc];
% Coriolis matrix:
C = [...
  (mc*r^2*cos(q1)*(2*p.pPos.dQ(1)*cos(q1)^2*sin(q1) + p.pPos.dQ(1)*cos(q2)^2*sin(q1) - p.pPos.dQ(1)*cos(q1)^4*sin(q1) - p.pPos.dQ(1)*cos(q2)^4*sin(q1) - 2*p.pPos.dQ(1)*cos(q1)^2*cos(q2)^2*sin(q1) + p.pPos.dQ(2)*cos(q1)*cos(q2)*sin(q2)))/(sin(q1)^2 + sin(q2)^2 - 1)^2, (p.pPos.dQ(1)*mc*r^2*cos(q1)^2*cos(q2)*sin(q2))/(sin(q1)^2 + sin(q2)^2 - 1)^2 - (2^(1/2)*p.pPos.dQ(8)*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)),                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0, -(2^(1/2)*p.pPos.dQ(2)*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)); ...
                                                                       (2^(1/2)*p.pPos.dQ(8)*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)) - (p.pPos.dQ(1)*mc*r^2*cos(q1)^2*cos(q2)*sin(q2))/(sin(q1)^2 + sin(q2)^2 - 1)^2,                                                                                                                     -(p.pPos.dQ(2)*mc*r^2*sin(2*q2))/2,                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0,  (2^(1/2)*p.pPos.dQ(1)*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)); ...
                                                                                                                                                                                                                   0,                                                                                                                                             0,                                                                                                            0,                                                                                         (p.pPos.dQ(5)*cos(q4)*(Iy*cos(2*q3) - Ix + Iz*cos(2*q3)))/2 + p.pPos.dQ(4)*sin(2*q3)*(Iy/2 - Iz/2),                                                                     (p.pPos.dQ(4)*cos(q4)*(Iy*(2*cos(q3)^2 - 1) - Ix + Iz*(2*cos(q3)^2 - 1)))/2 - p.pPos.dQ(5)*cos(q3)*cos(q4)^2*sin(q3)*(Iy - Iz), 0, 0,                                                                       0; ...
                                                                                                                                                                                                                   0,                                                                                                                                             0,                           - (p.pPos.dQ(4)*sin(2*q3)*(Iy - Iz))/2 - (p.pPos.dQ(5)*cos(q4)*(Iy*cos(2*q3) - Ix + Iz*cos(2*q3)))/2,                                                                                                                                             -(p.pPos.dQ(3)*sin(2*q3)*(Iy - Iz))/2, (cos(q4)*(Ix*p.pPos.dQ(3) - Iy*p.pPos.dQ(3) - Iz*p.pPos.dQ(3) + 2*Iy*p.pPos.dQ(3)*sin(q3)^2 + 2*Iz*p.pPos.dQ(3)*sin(q3)^2 - 2*Ix*p.pPos.dQ(5)*sin(q4) + 2*Iz*p.pPos.dQ(5)*sin(q4) + 2*Iy*p.pPos.dQ(5)*sin(q3)^2*sin(q4) - 2*Iz*p.pPos.dQ(5)*sin(q3)^2*sin(q4)))/2, 0, 0,                                                                       0; ...
                                                                                                                                                                                                                   0,                                                                                                                                             0, p.pPos.dQ(5)*cos(q3)*cos(q4)^2*sin(q3)*(Iy - Iz) - (p.pPos.dQ(4)*cos(q4)*(Ix + Iy*(2*cos(q3)^2 - 1) + Iz*(2*cos(q3)^2 - 1)))/2, p.pPos.dQ(5)*cos(q4)*sin(q4)*(Ix - Iz - Iy*sin(q3)^2 + Iz*sin(q3)^2) - (p.pPos.dQ(3)*cos(q4)*(Ix + Iy*(2*cos(q3)^2 - 1) + Iz*(2*cos(q3)^2 - 1)))/2 + p.pPos.dQ(4)*cos(q3)*sin(q3)*sin(q4)*(Iy + Iz),                                                                            p.pPos.dQ(4)*cos(q4)*sin(q4)*(Ix - Iz - Iy*sin(q3)^2 + Iz*sin(q3)^2) + p.pPos.dQ(3)*cos(q3)*cos(q4)^2*sin(q3)*(Iy - Iz), 0, 0,                                                                       0; ...
                                                                                                                                                                                                   -p.pPos.dQ(1)*mc*r*sin(q1),                                                                                                                                             0,                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0,                                                                       0; ...
                                                                                                                                                                                                                   0,                                                                                                                             -p.pPos.dQ(2)*mc*r*sin(q2),                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0,                                                                       0; ...
                                                                         -(2*2^(1/2)*p.pPos.dQ(1)*mc*r*sin(q1) - 2*2^(1/2)*p.pPos.dQ(1)*mc*r*cos(2*q2)*sin(q1) + 2^(1/2)*p.pPos.dQ(2)*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)),                                                                       -(2^(1/2)*p.pPos.dQ(1)*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)),                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0,                                                                       0    ];

% Gravity force matrix:
G = [...
     -(2^(1/2)*g*mc*r*sin(2*q1))/(2*(cos(2*q1) + cos(2*q2))^(1/2)); ...
     -(2^(1/2)*g*mc*r*sin(2*q2))/(2*(cos(2*q1) + cos(2*q2))^(1/2)); ...
                                                                 0; ...
                                                                 0; ...
                                                                 0; ...
                                                                 0; ...
                                                                 0; ...
                                                         g*(m + mc)    ];
EqM = M*p.pPos.ddQ + C*p.pPos.dQ + G;



%% Find forces and torques:

% Model-coupling matrices:
% if not(isnan(p.pPos.Q(3))||isnan(p.pPos.ddQ(3))||isnan(p.pPos.dQ(3)))
    At = R*[0 0 0 0; 0 0 0 0; 1 1 1 1];
    Ar = [ p.pPar.k1  p.pPar.k1 -p.pPar.k1  -p.pPar.k1; ...
          -p.pPar.k1  p.pPar.k1  p.pPar.k1  -p.pPar.k1; ...
           p.pPar.k2 -p.pPar.k2  p.pPar.k2  -p.pPar.k2];
    % torques in alpha and beta must be the contrary of torques in phi and theta
    Ap = -Ar(1:2,:);  
    A = [Ap;Ar;At];
    As = pinv(A); %(A'*A)\A'; % Matriz Pseudo-Inversa de A: A-sharp
% else
%     A  = zeros(8,4);
%     As = zeros(4,8);
% end
% Ganhos:
cgains = [.5 2 .5 2 5 2; 1 20 1 15 1 2.5];
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


E  = As*(C*p.pPos.dQ + G);
sigma = p.pPos.ddQd +Kd*p.pPos.dQtil + Kp*p.pPos.Qtil; % s = ddQ_d + Kd*dQ_til + Kp*Q_til
% sigma = sigma-p.pPos.ddQ;

% fz = M*sigma +C*p.pPos.dQ + G - EqM;
% p.pPos.ddQd = sigma;

Fr = As*M*sigma + E;
% Verificando se forcas sobre o referencia do veiculo
% ocorrem some% p.pPos.ddQd = sigma;nte na direcao Z
fTau = A*Fr;
% ------------------------------------
% Forcando valores possiveis: 30% do valor da gravidade
if real(fTau(8)) < 0
    fTau(8) = p.pPar.m*p.pPar.g*0.3;
end
% ------------------------------------

% Considerando a situacao mais simples de que a forca de propulsao
% solicitada aos motores e imediatamente atendida
% Nao considera modelo da bateria
% Modelo Inverso do Atuador: Forcas desejada nos propulsores
Fd = As*fTau;
% --------------
% Caso a forca do propulsor seja negativa, assume-se propulsco igual a zero
for ii = 1:4
    if Fd(ii) < 0
        Fd(ii) = 0;
    end
end


%% Low-level UAV dynamic model 
% 1: Fr -> Wr
Wda = p.pSC.Wd;
p.pSC.Wd = sqrt(Fd/p.pPar.Cf);

% 2: Wr -> V 
Vr = -p.pPar.Vo + p.pPar.Jm*p.pPar.R/p.pPar.Km*(p.pSC.Wd-Wda)/p.pPar.Ts + ...
    (p.pPar.Bm*p.pPar.R/p.pPar.Km + p.pPar.Kb)*p.pSC.Wd + ...
    p.pPar.Ct*p.pPar.R/p.pPar.Km/p.pPar.r1*p.pSC.Wd.^2;

% 3: V -> Qr
p.pSC.Qr(3) = p.pPos.Q(3) + 1/(p.pPar.kdp+p.pPar.kpp*p.pPar.Ts)*...
    (p.pPar.kdp*(p.pSC.Qr(3)-p.pPos.Q(3)) + 1/4*p.pPar.Ts*([1 1 -1 -1]*Vr));

p.pSC.Qr(4) = p.pPos.Q(4) + 1/(p.pPar.kdt+p.pPar.kpt*p.pPar.Ts)*...
    (p.pPar.kdt*(p.pSC.Qr(4)-p.pPos.Q(4)) + 1/4*p.pPar.Ts*([-1 1 1 -1]*Vr));

p.pSC.dQr(8) = p.pPos.dQ(8) + 1/(p.pPar.kdz+p.pPar.kpz*p.pPar.Ts)*...
    (p.pPar.kdz*(p.pSC.dQr(8)-p.pPos.dQ(8)) + 1/4*p.pPar.Ts*([1 1 1 1]*Vr));

p.pSC.dQr(5) = p.pPos.dQ(5) + 1/(p.pPar.kds+p.pPar.kps*p.pPar.Ts)*...
    (p.pPar.kds*(p.pSC.dQr(5)-p.pPos.dQ(5)) + 1/4*p.pPar.Ts*([1 -1 1 -1]*Vr));

% 4: Xr -> U
p.pSC.Ud(1) =  p.pSC.Qr(3)/p.pPar.uSat(1);   % Phi
p.pSC.Ud(2) = -p.pSC.Qr(4)/p.pPar.uSat(2);   % Theta
p.pSC.Ud(3) =  p.pSC.dQr(8)/p.pPar.uSat(3);   % dZ
p.pSC.Ud(4) = -p.pSC.dQr(5)/p.pPar.uSat(4);  % dPsi

p.pSC.Ud = tanh(p.pSC.Ud);
end