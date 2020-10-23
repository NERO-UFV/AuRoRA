function sInvKinematicModel(robot,dXr)

% Determine the robot velocity, based on the reference velocity
%        +-------------+
% dXr -> | InvKinematic|  ->  Ur
%        | Model       |
%        +-------------+
%
% Verify vector length
l = length(dXr);
% Inverse Kinematic Matrix (2D)
if l==2 
    
    Kinv = [cos(robot.pPos.X(6)),            sin(robot.pPos.X(6));
        -sin(robot.pPos.X(6))/robot.pPar.a,  cos(robot.pPos.X(6))/robot.pPar.a];
    
% Inverse Kinematic Matrix (3D)
elseif l==3
    
    Kinv = [cos(robot.pPos.X(6)),              sin(robot.pPos.X(6)),          0;
        -sin(robot.pPos.X(6))/robot.pPar.a,  cos(robot.pPos.X(6))/robot.pPar.a, 0;
                   0,                                 0,                    0;
                   0,                                 0,                    0];
               
% Ps.: Kinv(4x3) --> Ur(4,1) ==> same pattern as ArDrone's command signals  

else
    disp('Invalid vector length (please verify dXr).');
    Kinv =0;
end

% Reference control signal
robot.pSC.Ur = Kinv*dXr;