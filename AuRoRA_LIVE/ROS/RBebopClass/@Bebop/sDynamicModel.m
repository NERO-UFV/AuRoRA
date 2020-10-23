function sDynamicModel(obj)

% Dynamic model from
% Brand�o, A. S., M. Sarcinelli-Filho, and R. Carelli. 
% "High-level underactuated nonlinear control for rotorcraft machines." 
% Mechatronics (ICM), 2013 IEEE International Conference on. IEEE, 2013.
%
% ArDrone 2.0 Parameters
% Li, Qianying. "Grey-box system identification of a quadrotor unmanned 
% aerial vehicle." Master of Science Thesis Delft University of
% Technology (2014).
%
% Simulate ArDrone dynamic model
%
%      +----------+  W   +--------+  F   +----------+  T   +-------+
% U -> | Actuator |  ->  | Rotary |  ->  | Forces & |  ->  | Rigid |  -> X
%      | Dynamics |      | Wing   |      | Torques  |      | Body  |
%      +----------+      +--------+      +----------+      +-------+
%


% 1: Receive input signal
%     pitch          | [-1,1] <==> [-15,15] degrees
%     roll           | [-1,1] <==> [-15,15] degrees
%     altitude rate  | [-1,1] <==> [-1,1] m/s
%     yaw rate       | [-1,1] <==> [-100,100] degrees/s

obj.pPar.Xra = obj.pPar.Xr;

obj.pPar.Xr(4)  =  obj.pSC.Ud(1)*obj.pPar.uSat(1);
obj.pPar.Xr(5)  = -obj.pSC.Ud(2)*obj.pPar.uSat(2);
obj.pPar.Xr(9)  =  obj.pSC.Ud(3)*obj.pPar.uSat(3);
obj.pPar.Xr(12) = -obj.pSC.Ud(4)*obj.pPar.uSat(4);

% Receive the reference errors and compute the forces to be applied to the
% rigid body
% 2: Error -> Voltage

uphi   = obj.pPar.kdp*(obj.pPar.Xr(4) -obj.pPos.X(4)  - obj.pPar.Xra(4) +obj.pPos.Xa(4) )/obj.pPar.Ts   + obj.pPar.kpp*(obj.pPar.Xr(4)-obj.pPos.X(4));
utheta = obj.pPar.kdt*(obj.pPar.Xr(5) -obj.pPos.X(5)  - obj.pPar.Xra(5) +obj.pPos.Xa(5) )/obj.pPar.Ts  + obj.pPar.kpt*(obj.pPar.Xr(5)-obj.pPos.X(5)); 
udz    = obj.pPar.kdz*(obj.pPar.Xr(9) -obj.pPos.X(9)  - obj.pPar.Xra(9) +obj.pPos.Xa(9) )/obj.pPar.Ts  + obj.pPar.kpz*(obj.pPar.Xr(9)-obj.pPos.X(9));
udpsi  = obj.pPar.kds*(obj.pPar.Xr(12)-obj.pPos.X(12) - obj.pPar.Xra(12)+obj.pPos.Xa(12))/obj.pPar.Ts  + obj.pPar.kps*(obj.pPar.Xr(12)-obj.pPos.X(12));

obj.pPar.V = obj.pPar.Vo + (11.1-obj.pPar.Vo)*[1 -1 1 1; 1 1 1 -1; -1 1 1 1; -1 -1 1 -1]*...
    [0.15*tanh(uphi); 0.15*tanh(utheta); 0.4*tanh(udz); 0.3*tanh(udpsi)];
    
% Saturation considering the limits of the energy source (battery)
% drone.pPar.V = (drone.pPar.V>0).*drone.pPar.V;
% drone.pPar.V = (drone.pPar.V<=11.1).*drone.pPar.V + (drone.pPar.V>11.1).*11.1;
% disp(drone.pPar.V)

% 2: V -> W
% Motor dynamic model: 4 times faster than ArDrone dynamic model 
for ii = 1:4
obj.pPar.W = 1/(obj.pPar.Jm+obj.pPar.Tsm*(obj.pPar.Bm+obj.pPar.Km*obj.pPar.Kb/obj.pPar.R))*...
    (obj.pPar.Jm*obj.pPar.W+obj.pPar.Tsm*(obj.pPar.Km/obj.pPar.R*obj.pPar.V-obj.pPar.Ct*obj.pPar.W.^2/obj.pPar.r));
end

% 3: W -> F
% Deslocando valores passados
obj.pPar.F  = obj.pPar.Cf*obj.pPar.W.^2;

% Euler-Lagrange model
obj.pPos.Xa = obj.pPos.X;

Rx = [1 0 0; 0 cos(obj.pPos.X(4)) -sin(obj.pPos.X(4)); 0 sin(obj.pPos.X(4)) cos(obj.pPos.X(4))];
Ry = [cos(obj.pPos.X(5)) 0 sin(obj.pPos.X(5)); 0 1 0; -sin(obj.pPos.X(5)) 0 cos(obj.pPos.X(5))];
Rz = [cos(obj.pPos.X(6)) -sin(obj.pPos.X(6)) 0; sin(obj.pPos.X(6)) cos(obj.pPos.X(6)) 0; 0 0 1];

R = Rz*Ry*Rx;

% =========================================================================
% Translational inertial matrix
% Matriz de in�rcia translacional
Mt = obj.pPar.m*eye(3,3);

% Gravitational vector
G = [0; 0; obj.pPar.m*obj.pPar.g];

% ArDrone force matrix 
At = [0 0 0 0; 0 0 0 0; 1 1 1 1];


% Disturbance vector
ft = R*At*obj.pPar.F - obj.pPar.D(1:3);

% Numerical integration for Cartesian velocities
obj.pPos.X(7:9) = Mt\(ft - G)*obj.pPar.Ts + obj.pPos.X(7:9);

% =========================================================================
% Rotational inertia matrix
Mr = [obj.pPar.Ixx, ...
    obj.pPar.Ixy*cos(obj.pPos.X(4)) - obj.pPar.Ixz*sin(obj.pPos.X(4)), ...
    -obj.pPar.Ixx*sin(obj.pPos.X(5)) + obj.pPar.Ixy*sin(obj.pPos.X(4))*cos(obj.pPos.X(5)) + obj.pPar.Ixz*cos(obj.pPos.X(4))*cos(obj.pPos.X(5));
    
    obj.pPar.Ixy*cos(obj.pPos.X(4)) - obj.pPar.Ixz*sin(obj.pPos.X(4)), ...
    obj.pPar.Iyy*cos(obj.pPos.X(4))^2 + obj.pPar.Izz*sin(obj.pPos.X(4))^2 - 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4)),...
    obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5)) - obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5)) - obj.pPar.Ixy*cos(obj.pPos.X(4))*sin(obj.pPos.X(5)) + obj.pPar.Ixz*sin(obj.pPos.X(4))*sin(obj.pPos.X(5)) + obj.pPar.Iyz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5)) - obj.pPar.Iyz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5));
    
    -obj.pPar.Ixx*sin(obj.pPos.X(5)) + obj.pPar.Ixy*sin(obj.pPos.X(4))*cos(obj.pPos.X(5)) + obj.pPar.Ixz*cos(obj.pPos.X(4))*cos(obj.pPos.X(5)), ...
    obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5)) - obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5)) - obj.pPar.Ixy*cos(obj.pPos.X(4))*sin(obj.pPos.X(5)) + obj.pPar.Ixz*sin(obj.pPos.X(4))*sin(obj.pPos.X(5)) + obj.pPar.Iyz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5)) - obj.pPar.Iyz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5)),...
    obj.pPar.Ixx*sin(obj.pPos.X(5))^2 + obj.pPar.Iyy*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))^2 + obj.pPar.Izz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))^2 - 2*obj.pPar.Ixy*sin(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - 2*obj.pPar.Ixz*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) + 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2
    ];

% Rotational Coriolis matrix
Cr = [ 0, ...
    obj.pPos.X(11)*(obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(5)) - obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4)) + obj.pPar.Iyz*cos(obj.pPos.X(4))^2 - obj.pPar.Iyz*sin(obj.pPos.X(4))^2) + obj.pPos.X(12)*(-obj.pPar.Ixx*cos(obj.pPos.X(5))/2 - obj.pPar.Iyy*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Iyy*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Izz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Izz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Ixy*sin(obj.pPos.X(4))*sin(obj.pPos.X(5)) - obj.pPar.Ixz*cos(obj.pPos.X(4))*sin(obj.pPos.X(5)) + 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))),...
    obj.pPos.X(11)*(-obj.pPar.Ixx*cos(obj.pPos.X(5))/2 - obj.pPar.Iyy*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Iyy*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Izz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Izz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Ixy*sin(obj.pPos.X(4))*sin(obj.pPos.X(5)) - obj.pPar.Ixz*cos(obj.pPos.X(4))*sin(obj.pPos.X(5)) + 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))) + obj.pPos.X(12)*(-obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 + obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 + obj.pPar.Ixy*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - obj.pPar.Ixz*sin(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - obj.pPar.Iyz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))^2 + obj.pPar.Iyz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))^2);
    
    obj.pPos.X(10)*(-obj.pPar.Ixy*sin(obj.pPos.X(4)) - obj.pPar.Ixz*cos(obj.pPos.X(4))) + obj.pPos.X(11)*(-obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(4)) + obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4)) - obj.pPar.Iyz*cos(obj.pPos.X(4))^2 + obj.pPar.Iyz*sin(obj.pPos.X(4))^2) + obj.pPos.X(12)*(obj.pPar.Ixx*cos(obj.pPos.X(5))/2 + obj.pPar.Iyy*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Iyy*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Izz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Izz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Ixy*sin(obj.pPos.X(4))*sin(obj.pPos.X(5)) + obj.pPar.Ixz*cos(obj.pPos.X(4))*sin(obj.pPos.X(5)) - 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))),...
    obj.pPos.X(10)*(-obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(4)) + obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4)) - obj.pPar.Iyz*cos(obj.pPos.X(4))^2 + obj.pPar.Iyz*sin(obj.pPos.X(4))^2),...
    obj.pPos.X(10)*(obj.pPar.Ixx*cos(obj.pPos.X(5))/2 + obj.pPar.Iyy*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Iyy*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Izz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Izz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Ixy*sin(obj.pPos.X(4))*sin(obj.pPos.X(5)) + obj.pPar.Ixz*cos(obj.pPos.X(4))*sin(obj.pPos.X(5)) - 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))) + obj.pPos.X(12)*(-obj.pPar.Ixx*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) + obj.pPar.Iyy*sin(obj.pPos.X(4))^2*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) + obj.pPar.Izz*cos(obj.pPos.X(4))^2*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) + obj.pPar.Ixy*sin(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 - obj.pPar.Ixy*sin(obj.pPos.X(4))*sin(obj.pPos.X(5))^2 + obj.pPar.Ixz*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 - obj.pPar.Ixz*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))^2 + 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)));
    
    obj.pPos.X(10)*(obj.pPar.Ixy*cos(obj.pPos.X(4))*cos(obj.pPos.X(5)) - obj.pPar.Ixz*sin(obj.pPos.X(4))*cos(obj.pPos.X(5))) + obj.pPos.X(11)*(-obj.pPar.Ixx*cos(obj.pPos.X(5))/2 + obj.pPar.Iyy*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Iyy*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Izz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Izz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))) + obj.pPos.X(12)*(obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 - obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 - obj.pPar.Ixy*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) + obj.pPar.Ixz*sin(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) + obj.pPar.Iyz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))^2 - obj.pPar.Iyz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))^2),...
    obj.pPos.X(10)*(-obj.pPar.Ixx*cos(obj.pPos.X(5))/2 + obj.pPar.Iyy*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Iyy*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - obj.pPar.Izz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 + obj.pPar.Izz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))/2 - 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))) + obj.pPos.X(11)*(-obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*sin(obj.pPos.X(5)) + obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*sin(obj.pPos.X(5)) - obj.pPar.Ixy*cos(obj.pPos.X(4))*cos(obj.pPos.X(5)) + obj.pPar.Ixz*sin(obj.pPos.X(4))*cos(obj.pPos.X(5)) + obj.pPar.Iyz*sin(obj.pPos.X(4))^2*sin(obj.pPos.X(5)) - obj.pPar.Iyz*cos(obj.pPos.X(4))^2*sin(obj.pPos.X(5))) + obj.pPos.X(12)*(obj.pPar.Ixx*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - obj.pPar.Iyy*sin(obj.pPos.X(4))^2*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - obj.pPar.Izz*cos(obj.pPos.X(4))^2*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - obj.pPar.Ixy*sin(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 + obj.pPar.Ixy*sin(obj.pPos.X(4))*sin(obj.pPos.X(5))^2 - obj.pPar.Ixz*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 + obj.pPar.Ixz*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))^2 - 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5))),...
    obj.pPos.X(10)*(obj.pPar.Iyy*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 - obj.pPar.Izz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 - obj.pPar.Ixy*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) + obj.pPar.Ixz*sin(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) + obj.pPar.Iyz*cos(obj.pPos.X(4))^2*cos(obj.pPos.X(5))^2 - obj.pPar.Iyz*sin(obj.pPos.X(4))^2*cos(obj.pPos.X(5))^2) + obj.pPos.X(11)*(obj.pPar.Ixx*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - obj.pPar.Iyy*sin(obj.pPos.X(4))^2*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - obj.pPar.Izz*cos(obj.pPos.X(4))^2*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)) - obj.pPar.Ixy*sin(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 + obj.pPar.Ixy*sin(obj.pPos.X(4))*sin(obj.pPos.X(5))^2 - obj.pPar.Ixz*cos(obj.pPos.X(4))*cos(obj.pPos.X(5))^2 + obj.pPar.Ixz*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))^2 - 2*obj.pPar.Iyz*sin(obj.pPos.X(4))*cos(obj.pPos.X(4))*sin(obj.pPos.X(5))*cos(obj.pPos.X(5)))
    ];

% ArDrone
Ar = [obj.pPar.k1  obj.pPar.k1 -obj.pPar.k1  -obj.pPar.k1;
    -obj.pPar.k1  obj.pPar.k1  obj.pPar.k1  -obj.pPar.k1;
    obj.pPar.k2 -obj.pPar.k2  obj.pPar.k2  -obj.pPar.k2];

% Aerodynamic thrust 
T = Ar*obj.pPar.F - obj.pPar.Q;

%--------------------------------------------
% Numerical integration of rotational movement
obj.pPos.X(10:12) = Mr\(T - Cr*obj.pPos.X(10:12))*obj.pPar.Ts + obj.pPos.X(10:12);

% ArDrone pose - Numerical integration
for ii = 1:6
    obj.pPos.X(ii) = obj.pPos.X(ii+6)*obj.pPar.Ts + obj.pPos.X(ii);
    if ii > 3
        if obj.pPos.X(ii) > pi
            obj.pPos.X(ii) = -2*pi + obj.pPos.X(ii);
        end
        if obj.pPos.X(ii) < -pi
            obj.pPos.X(ii) = 2*pi + obj.pPos.X(ii);
        end
    end
end