function mAngularLineFormationControl_NSB(obj,type)
% Calculate kinematic control velocity
% Valentim Ernandes 09/01/2020

% Direct transformation
obj.mDirTransAngular;                   % get formation variables

% Formation pose error
obj.pPos.Qtil = obj.pPos.Qd - obj.pPos.Q;

x1 = obj.pPos.X(1);
y1 = obj.pPos.X(2);
z1 = obj.pPos.X(3);

x2 = obj.pPos.X(4);
y2 = obj.pPos.X(5);
z2 = obj.pPos.X(6);

J = [                                                                                                                                                                     1,                                                                                                                                                                    0,                                                                                                                                               0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
    0,                                                                                                                                                                    1,                                                                                                                                               0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
    0,                                                                                                                                                                    0,                                                                                                                                               1,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                0;
    (x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                            (y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                       (z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                          -(x1 - x2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                                          -(y1 - y2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2),                                                                                       -(z1 - z2)/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(1/2);
    -(y1^2 - 2*y1*y2 + y2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                      ((2*y1 - 2*y2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), ((2*z1 - 2*z2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), (y1^2 - 2*y1*y2 + y2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                    -((2*y1 - 2*y2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -((2*z1 - 2*z2)*(x1 - x2))/(2*(1 - (x1 - x2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2));
    ((2*x1 - 2*x2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -(x1^2 - 2*x1*x2 + x2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), ((2*z1 - 2*z2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)),                    -((2*x1 - 2*x2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), (x1^2 - 2*x1*x2 + x2^2 + z1^2 - 2*z1*z2 + z2^2)/((1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2)), -((2*z1 - 2*z2)*(y1 - y2))/(2*(1 - (y1 - y2)^2/((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2))^(1/2)*((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)^(3/2))];

J1 = J(1:3,:);
J2 = J(4:6,:);

%obj.pPos.intQtil = obj.pPos.intQtil + obj.pPos.Qtil;

% Robots velocities
switch type 
    
    case 'N'
        obj.pPos.dXr = J\(obj.pPos.dQd + obj.pPar.K1*tanh(obj.pPar.K2*obj.pPos.Qtil));
        
    case 'P'
%     [(eye(6)-pinv(J1)*J1)]
        obj.pPos.dXr = pinv(J1)*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3)))+ ...
        (eye(6)-pinv(J1)*J1)*pinv(J2)*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6)));

    case 'S'
%       [(eye(6)-pinv(J2)*J2)]
        obj.pPos.dXr = (eye(6)-pinv(J2)*J2)*pinv(J1)*(obj.pPos.dQd(1:3) + obj.pPar.K1(1:3,1:3)*tanh(obj.pPar.K2(1:3,1:3)*obj.pPos.Qtil(1:3)))+ ...
        pinv(J2)*(obj.pPos.dQd(4:6) + obj.pPar.K1(4:6,4:6)*tanh(obj.pPar.K2(4:6,4:6)*obj.pPos.Qtil(4:6)));        
end
    
obj.pPos.Xr = obj.pPos.Xr + obj.pPos.dXr*obj.pPar.Ts;

end