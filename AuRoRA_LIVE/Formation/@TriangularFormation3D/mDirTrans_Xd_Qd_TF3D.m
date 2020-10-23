%% Direct Transformation
% X --->> Q
% Calculate the formation's pose using the robots' position.

%  + ----------------------------------------------+
%  |                 ROBOTS SPACE (X)              |      X_      _X 
%  |                                               |        \____/
%  |         R1            R2            R3        |         |  | 
%  |    | x1 y1 z1 |  | x2 y2 z2 |  | x3 y3 z3 |   |       _/\__/\_ 
%  |                                               |      X        X
%  +-----------------------------------------------+
%                          ||
%                          ||
%                          ||
%                          ||                      R1   |   
%                         \\//                        X___X     
%                          \/                          |_|  \
%  + ---------------------------------------------+   X   X    \
%  |                FORMATION SPACE (Q)           |     |         \  R2
%  |                                              |                X___X
%  |        P             S             O         |     |           |_|
%  |  | xF yF zF | | p q beta | | phi theta psi | |               X   X
%  |                                              |     |         /
%  +----------------------------------------------+   X___X    /
%                                                      |_|  /
%                                                     X   X  
%                                                  R3   |  

function mDirTrans_Xd_Qd_TF3D(obj)

% This function calculates de direct kinematics transformation, it is
% necessery to express the relationship between the formation
% position-shape-orientation and the robots positions.
%
% The robots positions will be represented by X = [ R1 R2 R3 ] and the 
% whole formation by Q = [ P S O ].
%
% It is important to stress that the orientation of the robots is not
% considered in this proposal.
%
% This approach can be used to determine the desired formation references
% from the initial position of the robots. That is, it is initially defined
% where the robots should be and from there the position-shape-orientation
% references of the formation are foud.

%% Loading robots parameters
% Robot 1 (R1)
x1 = obj.pPos.Xd(1);
y1 = obj.pPos.Xd(2);
z1 = obj.pPos.Xd(3);

% Robot 2 (R2)
x2 = obj.pPos.Xd(4);
y2 = obj.pPos.Xd(5);
z2 = obj.pPos.Xd(6);

% Robot 3 (R3)
x3 = obj.pPos.Xd(7);
y3 = obj.pPos.Xd(8);
z3 = obj.pPos.Xd(9);

%% Calculating the transformation
% Position (P)
x = (x1 + x2 + x3)/3;
y = (y1 + y2 + y3)/3;
z = (z1 + z2 + z3)/3;

% Shape (S)
p = sqrt((x1 - x2)^2 + (y1-y2)^2 + (z1-z2)^2);
q = sqrt((x1 - x3)^2 + (y1-y3)^2 + (z1-z3)^2);
r = sqrt((x2 - x3)^2 + (y2-y3)^2 + (z2-z3)^2);
beta = acos((p^2 + q^2 - r^2)/(2*p*q));

% Orientation (O)
phi = atan2((2*z1 - z2 - z3),(2*y1 - y2 - y3));
theta = -atan((2*z1 - z2 - z3)/(2*x1 - x2 - x3));
psi = atan2((2*y1 - y2 - y3),(2*x1 - x2 - x3));

%% Applying the results
% P
obj.pPos.Qd(1) = x;
obj.pPos.Qd(2) = y;
obj.pPos.Qd(3) = z;

% S
obj.pPos.Qd(4) = p;
obj.pPos.Qd(5) = q;
obj.pPos.Qd(6) = beta;

% O
obj.pPos.Qd(7) = phi;
obj.pPos.Qd(8) = theta;
obj.pPos.Qd(9) = psi;

end