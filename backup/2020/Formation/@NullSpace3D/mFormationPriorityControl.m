% Calculate kinematic control velocity
function mFormationPriorityControl(obj)
%% SECTION TITLE
% DESCRIPTIVE TEXT
% Direct transformation
obj.mDirTrans;                   % Get Formation Variables

% Formation pose error
obj.mFormationError;

% Get Robot Posicions
x1 = obj.pPos.X(1,1);            % Pioneer Variables
y1 = obj.pPos.X(2,1);
z1 = obj.pPos.X(3,1);

x2 = obj.pPos.X(4,1);            % Drone Variables
y2 = obj.pPos.X(5,1);
z2 = obj.pPos.X(6,1);


%% Matriz Jacobiana Direta           
% Linha 4 da Matriz Jacobiana
% J41 = (pos(1,i) - x(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J42 = (pos(2,i) - y(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J43 = (zp - z(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J44 = -(pos(1,i) - x(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J45 = -(pos(2,i) - y(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J46 = -(zp - z(i)) / sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
%     % Linha 5 da Matriz Jacobiana
% J51 = -(pos(2,i) - y(i)) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2);
% J52 =  (pos(1,i) - x(i)) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2);
% J53 =  0;
% J54 =  (pos(2,i) - y(i)) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2);
% J55 = -(pos(1,i) - x(i)) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2);
% J56 =  0;
%     % Linha 6 da Matriz Jacobiana
% J61 =  ((2*pos(1,i) - 2*x(i))*(zp - z(i))) / sqrt(2*((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2)) * ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J62 =  ((2*pos(2,i) - 2*y(i))*(zp - z(i))) / sqrt(2*((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2)) * ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J63 = -sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J64 = -((2*pos(1,i) - 2*x(i))*(zp - z(i))) / sqrt(2*((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2)) * ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J65 = -((2*pos(1,i) - 2*x(i))*(zp - z(i))) / sqrt(2*((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2)) * ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);
% J66 =  sqrt((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2) / ((pos(1,i) - x(i))^2 + (pos(2,i) - y(i))^2 + (zp - z(i))^2);


%% Matriz Jacobiana Direta           
% Linha 4 da Matriz Jacobiana
J41 = (x1 - x2) / sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J42 = (y1 - y2) / sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J43 = (z1 - z2) / sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J44 = -(x1 - x2) / sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J45 = -(y1 - y2) / sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J46 = -(z1 - z2) / sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);

% Linha 5 da Matriz Jacobiana
J51 = -(y1 - y2) / ((x1 - x2)^2 + (y1 - y2)^2);
J52 =  (x1 - x2) / ((x1 - x2)^2 + (y1 - y2)^2);
J53 =  0;
J54 =  (y1 - y2) / ((x1 - x2)^2 + (y1 - y2)^2);
J55 = -(x1 - x2) / ((x1 - x2)^2 + (y1 - y2)^2);
J56 =  0;

% Linha 6 da Matriz Jacobiana
J61 =  ((2*x1 - 2*x2)*(z1 - z2)) / sqrt(2*((x1 - x2)^2 + (y1 - y2)^2)) * ((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J62 =  ((2*y1 - 2*y2)*(z1 - z2)) / sqrt(2*((x1 - x2)^2 + (y1 - y2)^2)) * ((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J63 = -sqrt((x1 - x2)^2 + (y1 - y2)^2) / ((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J64 = -((2*x1 - 2*x2)*(z1 - z2)) / sqrt(2*((x1 - x2)^2 + (y1 - y2)^2)) * ((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J65 = -((2*x1 - 2*x2)*(z1 - z2)) / sqrt(2*((x1 - x2)^2 + (y1 - y2)^2)) * ((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
J66 =  sqrt((x1 - x2)^2 + (y1 - y2)^2) / ((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);


Jacob = [ 1  ,  0  , 0  , 0  , 0  , 0 ;...
          0  ,  1  , 0  , 0  , 0  , 0 ;...
          0  ,  0  , 1  , 0  , 0  , 0 ;...
          J41, J42, J43, J44, J45, J46;...
          J51, J52, J53, J54, J55, J56;...
          J61, J62, J63, J64, J65, J66 ];

      
% Identity matriz with same size of direct Jacobian
I = eye(size(Jacob));
      
% Split Jacobian in Two Tasks
Jp = Jacob(1:3,:);
Jf = Jacob(4:6,:);      

% Get Jacobianas Pseudo-Inverse 
JpInv = pinv(Jp);
JfInv = pinv(Jf);

% Formation velocity
obj.pPos.dQr = obj.pPos.dQd + obj.pPar.K1*tanh(obj.pPar.K2*obj.pPos.Qtil);

% Split Desired Robot Position and Formation
dQrP = obj.pPos.dQr(1:3,1);
dQrF = obj.pPos.dQr(4:6,1);

% Robots velocities For Formation Priority
dXrP = ((I-(JfInv*Jf))*JpInv) * dQrP;
dXrF = JfInv*dQrF;

obj.pPos.dXr = dXrF + dXrP; % Task 01 + Task 02

% Classe
obj.pPos.Xr = obj.pPos.Xr + obj.pPos.dXr * obj.SampleTime;


end