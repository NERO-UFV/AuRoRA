% Calculate kinematic control velocity
function mFormationControl_J_de_x(obj)
% Direct transformation
obj.mDirTrans;                   % get formation variables

% Formation pose error
obj.mFormationError;

x1 = obj.pPos.X(1);
y1 = obj.pPos.X(2);
z1 = obj.pPos.X(3);

x2 = obj.pPos.X(4);
y2 = obj.pPos.X(5);
z2 = obj.pPos.X(6);

J = [                                                                                                     1,                                                                                                         0,                                                                            0,                                                                                                          0,                                                                                                          0,                                                                           0;
                                                                                                          0,                                                                                                         1,                                                                            0,                                                                                                          0,                                                                                                          0,                                                                           0;
                                                                                                          0,                                                                                                         0,                                                                            1,                                                                                                          0,                                                                                                          0,                                                                           0;
                                                  (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                 (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                    (z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                 -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                 -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                  -(z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
                                                                     -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2),                                                                     (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2),                                                                            0,                                                                      (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2),                                                                     -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2),                                                                           0;
  ((2*x1 - 2*x2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)), ((2*y1 - 2*y2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)), -((x1 - x2)^2 + (y1 - y2)^2)^(1/2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2), -((2*x1 - 2*x2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)), -((2*y1 - 2*y2)*(z1 - z2))/(2*((x1 - x2)^2 + (y1 - y2)^2)^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)), ((x1 - x2)^2 + (y1 - y2)^2)^(1/2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)];
 
%  Inverse Jacobian Matrix  {Formation vector: [xf yf zf rhof alphaf betaf]}

J_1 = J(1:3,1:6);
J_2 = J(4:6,1:6);

Jinv_1 = pinv(J_1);
Jinv_2 = pinv(J_2);

% Robots velocities
obj.pPos.dXr = Jinv_1*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3)))...
    + (eye(6)-Jinv_1*J_1)*(Jinv_2*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6))));

% LF.pPos.dXr = Jinv_1*(LF.pPos.dQd(1:3) + LF.pPar.K1(1:3,1:3)*tanh(LF.pPar.K2(1:3,1:3)*LF.pPos.Qtil(1:3)))...
%     + (eye(6)-Jinv_1*J_1)*(Jinv_2*(LF.pPos.dQd(4:6) + LF.pPar.K1(4:6,4:6)*tanh(LF.pPar.K2(4:6,4:6)*LF.pPos.Qtil(4:6))));
end