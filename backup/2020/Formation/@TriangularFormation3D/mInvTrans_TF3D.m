%% Inverse Transformation (Desired variables)
% Qd --->> Xd
% Calculate the robots' variables using the formation's pose. In this case
% we calculate the desired position for the robots (Xd) based on the pose
% gaved by the trajectory planner.

%                                                  R1   |
%                                                     X___X
%                                                      |_|  \
%  + ---------------------------------------------+   X   X    \
%  |               FORMATION SPACE (Qd)           |     |         \     R2
%  |                                              |                X___X
%  |        P            S              O         |     |           |_|
%  |  | xF yF zF | | p q beta | | phi theta psi | |                X   X
%  |                                              |     |         /
%  +----------------------------------------------+   X___X    /
%                          ||                          |_|  /
%                          ||                         X   X
%                          ||                      R3   |
%                          ||
%                         \\//
%                          \/
%  + ----------------------------------------------+
%  |                  ROBOTS SPACE (Xd)            |       X_      _X
%  |                                               |         \____/
%  |         R1            R2            R3        |          |  |
%  |    | x1 y1 z1 |  | x2 y2 z2 |  | x3 y3 z3 |   |        _/\__/\_
%  |                                               |       X        X
%  +-----------------------------------------------+


function mInvTrans_TF3D(obj)

% This function calculates de inverse kinematics transformation, it is
% necessery to express the relationship between the robots positions and
% the formation position-shape-orientation.
%
% The whole formation will be represented by Q = [ P S O ] and the robots
% positions by X = [ R1 R2 R3 ].
%
% It is important to stress that the orientation of the robots is not
% considered in this proposal.
%
% This transformation can be used to take from Qd to Xd in order to
% obtain the error Xtil_d = Xd - X, where Xd is the reference
% obtained by trajectory planning, unlike the Xref reference obtained by
% integrating the speed reference from the trajectory controller.

%% Loading formation parameters
% Position (P)
x = obj.pPos.Qd(1);
y = obj.pPos.Qd(2);
z = obj.pPos.Qd(3);

% Shape (S)
p = obj.pPos.Qd(4);
q = obj.pPos.Qd(5);
beta = obj.pPos.Qd(6);

% Orientation (O)
phi = obj.pPos.Qd(7);
theta = obj.pPos.Qd(8);
psi = obj.pPos.Qd(9);

%%  Auxiliary parameters

r = sqrt(p^2 + q^2 - 2*p*q*cos(beta));
h1 = sqrt(0.5*(p^2 + q^2 - 0.5*r^2));
h2 = sqrt(0.5*(r^2 + p^2 - 0.5*q^2));
h3 = sqrt(0.5*(q^2 + r^2 - 0.5*p^2));
alpha1 = acos((4*(h1^2 + h2^2) - 9*p^2)/(8*h1*h2));
alpha2 = acos((4*(h1^2 + h3^2) - 9*q^2)/(8*h1*h3));


%%  Calculating the transformation

% Robot 1 (R1)
x1 = 2/3*h1*cos(theta)*cos(psi) + x;
y1 = 2/3*h1*cos(theta)*sin(psi) + y;
z1 = - 2/3*h1*sin(theta) + z;

% Robot 2 (R2)
x2 =  2/3*h2*cos(alpha1)*cos(theta)*cos(psi) ...
    + 2/3*h2*sin(alpha1)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
    + x;
y2 =  2/3*h2*cos(alpha1)*cos(theta)*sin(psi) ...
    + 2/3*h2*sin(alpha1)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
    + y;
z2 = - 2/3*h2*cos(alpha1)*sin(theta) ...
    + 2/3*h2*sin(alpha1)*sin(phi)*cos(theta)...
    + z;

% Robot 3 (R3)
x3 =  2/3*h3*cos(alpha2)*cos(theta)*cos(psi) ...
    - 2/3*h3*sin(alpha2)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
    + x;
y3 =   2/3*h3*cos(alpha2)*cos(theta)*sin(psi) ...
    - 2/3*h3*sin(alpha2)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
    + y;
z3 = - 2/3*h3*cos(alpha2)*sin(theta) ...
    - 2/3*h3*sin(alpha2)*sin(phi)*cos(theta)...
    + z;

%% Applying the results

% R1
obj.pPos.Xd(1) = x1;
obj.pPos.Xd(2) = y1;
obj.pPos.Xd(3) = z1;

% R2
obj.pPos.Xd(4) = x2;
obj.pPos.Xd(5) = y2;
obj.pPos.Xd(6) = z2;

% R3
obj.pPos.Xd(7) = x3;
obj.pPos.Xd(8) = y3;
obj.pPos.Xd(9) = z3;

end