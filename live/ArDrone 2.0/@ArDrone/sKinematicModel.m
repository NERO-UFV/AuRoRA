% function sKinematicModel(drone)

% Determine the robot pose, based on the control signal
%      +----------+  
% U -> | Kinematic|  ->  X
%      | Model    |      
%      +----------+      
% 
%  U = [Roll(phi) Pitch(theta) dAltitude(dz) dYaw(dpsi)]'
%  X = [x y z psi]'
% 
% Kinematic model from
% Santana, L.V., Brandão, A. S. and M. Sarcinelli-Filho
% "Outdoor Waypoint Navigation with the AR.Drone Quadrotor" 
% International Conference on Unmanned Aircraft Systems (ICUAS),2015

% 
% psi = drone.pPos.X(6);  % yaw angle
% 
% % Direct Kinematic Model
% K = [cos(psi), -sin(psi), 0, 0;
%      sin(psi), cos(psi),  0, 0;
%      0,        0,         1, 0;
%      0,        0,         0, 1];
%  
% % Current position
% drone.pPos.X([1 2 3 6]) = drone.pPos.X([1 2 3 6]) + K*drone.pSC.U*drone.pPar.Ts;
%  
% % Angle limitation per quadrant
% for ii = 4:6
%     if abs(drone.pPos.X(ii)) > pi
%         if drone.pPos.X(ii) < 0
%             drone.pPos.X(ii) = drone.pPos.X(ii) + 2*pi;
%         else
%             drone.pPos.X(ii) = drone.pPos.X(ii) - 2*pi;
%         end
%     end
% end
% 
% % First-time derivative of the current position
% drone.pPos.dX([7 8 9 12]) = K*drone.pSC.U;