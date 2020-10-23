function mAuxPriorityControl(obj)

% DESCRIPTIVE TEXT
% Direct transformation
obj.mDirTrans;                   % Get Formation Variables

% Formation pose error
obj.mFormationError;


% Inverse Jacobian Matrix  {Formation vector: [xf yf zf rhof alphaf betaf]}
rhof   = obj.pPos.Q(4);
alphaf = obj.pPos.Q(5);
betaf  = obj.pPos.Q(6);

% rhof   = LF.pPos.Q(4);
% alphaf = LF.pPos.Q(5);
% betaf  = LF.pPos.Q(6);


J = [                                                                                                                                                                1,                                                                                                                                                                0,                                                   0,                                                                                                                                                                 0,                                                                                                                                                                 0,                                                  0;
    0,                                                                                                                                                                1,                                                   0,                                                                                                                                                                 0,                                                                                                                                                                 0,                                                  0;
    0,                                                                                                                                                                0,                                                   1,                                                                                                                                                                 0,                                                                                                                                                                 0,                                                  0;
    -(cos(alphaf)*cos(betaf))/(cos(alphaf)^2*cos(betaf)^2 + cos(alphaf)^2*sin(betaf)^2 + cos(betaf)^2*sin(alphaf)^2 + sin(alphaf)^2*sin(betaf)^2),                    -(cos(betaf)*sin(alphaf))/(cos(alphaf)^2*cos(betaf)^2 + cos(alphaf)^2*sin(betaf)^2 + cos(betaf)^2*sin(alphaf)^2 + sin(alphaf)^2*sin(betaf)^2),           -sin(betaf)/(cos(betaf)^2 + sin(betaf)^2),                      (cos(alphaf)*cos(betaf))/(cos(alphaf)^2*cos(betaf)^2 + cos(alphaf)^2*sin(betaf)^2 + cos(betaf)^2*sin(alphaf)^2 + sin(alphaf)^2*sin(betaf)^2),                      (cos(betaf)*sin(alphaf))/(cos(alphaf)^2*cos(betaf)^2 + cos(alphaf)^2*sin(betaf)^2 + cos(betaf)^2*sin(alphaf)^2 + sin(alphaf)^2*sin(betaf)^2),           sin(betaf)/(cos(betaf)^2 + sin(betaf)^2);
    sin(alphaf)/(rhof*cos(betaf)*cos(alphaf)^2 + rhof*cos(betaf)*sin(alphaf)^2),                                                                                     -cos(alphaf)/(rhof*cos(betaf)*cos(alphaf)^2 + rhof*cos(betaf)*sin(alphaf)^2),                                                   0,                                                                                      -sin(alphaf)/(rhof*cos(betaf)*cos(alphaf)^2 + rhof*cos(betaf)*sin(alphaf)^2),                                                                                       cos(alphaf)/(rhof*cos(betaf)*cos(alphaf)^2 + rhof*cos(betaf)*sin(alphaf)^2),                                                  0;
    (cos(alphaf)*sin(betaf))/(rhof*cos(alphaf)^2*cos(betaf)^2 + rhof*cos(alphaf)^2*sin(betaf)^2 + rhof*cos(betaf)^2*sin(alphaf)^2 + rhof*sin(alphaf)^2*sin(betaf)^2), (sin(alphaf)*sin(betaf))/(rhof*cos(alphaf)^2*cos(betaf)^2 + rhof*cos(alphaf)^2*sin(betaf)^2 + rhof*cos(betaf)^2*sin(alphaf)^2 + rhof*sin(alphaf)^2*sin(betaf)^2), -cos(betaf)/(rhof*cos(betaf)^2 + rhof*sin(betaf)^2), -(cos(alphaf)*sin(betaf))/(rhof*cos(alphaf)^2*cos(betaf)^2 + rhof*cos(alphaf)^2*sin(betaf)^2 + rhof*cos(betaf)^2*sin(alphaf)^2 + rhof*sin(alphaf)^2*sin(betaf)^2), -(sin(alphaf)*sin(betaf))/(rhof*cos(alphaf)^2*cos(betaf)^2 + rhof*cos(alphaf)^2*sin(betaf)^2 + rhof*cos(betaf)^2*sin(alphaf)^2 + rhof*sin(alphaf)^2*sin(betaf)^2), cos(betaf)/(rhof*cos(betaf)^2 + rhof*sin(betaf)^2)];

J_1 = J(1:3,1:6);
J_2 = J(4:6,1:6);

Jinv_1 = pinv(J_1);
Jinv_2 = pinv(J_2);


% % % Jinv = [1, 0, 0, 0, 0, 0;
% % %         0, 1, 0, 0, 0, 0;
% % %         0, 0, 1, 0, 0, 0;
% % %         1, 0, 0, cos(alphaf)*cos(betaf), -sin(alphaf)*cos(betaf)*rhof, -cos(alphaf)*sin(betaf)*rhof;
% % %         0, 1, 0, sin(alphaf)*cos(betaf), cos(alphaf)*cos(betaf)*rhof,  -sin(alphaf)*sin(betaf)*rhof;
% % %         0, 0, 1, sin(betaf),                      0,                            cos(betaf)*rhof];


% Robots velocities

% Position Priority
% obj.pPos.dXr = Jinv_1*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3)))...
%     + (eye(6)-Jinv_1*J_1)*(Jinv_2*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6))));

% Formation Priority
obj.pPos.dXr = Jinv_2*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6)))...
    + (eye(6)-Jinv_2*J_2)*(Jinv_1*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3))));


obj.pPos.Xr = obj.pPos.Xr + obj.pPos.dXr * obj.SampleTime;

end

