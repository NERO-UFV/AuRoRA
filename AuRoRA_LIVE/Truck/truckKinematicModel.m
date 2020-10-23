function truck = truckKinematicModel(truck)

% Determine the robot pose, based on the control signal
%      +-----------+      .
% U -> | Kinematic |  ->  X
%      | Model     |      
%      +-----------+      
%

K = [cos(truck.pPos.X(6)) -truck.pPar.a*sin(truck.pPos.X(6)+truck.pPar.alpha); ...
     sin(truck.pPos.X(6))  truck.pPar.a*cos(truck.pPos.X(6)+truck.pPar.alpha); ...
             0                             1                   ];

% Current position
truck.pPos.X([1 2 6]) = truck.pPos.X([1 2 6]) + K*truck.pSC.U(1:2)*truck.pPar.Ts;

% Angle limitation per quadrant
for ii = 4:6
    if abs(truck.pPos.X(ii)) > pi
        if truck.pPos.X(ii) < 0
            truck.pPos.X(ii) = truck.pPos.X(ii) + 2*pi;
        else
            truck.pPos.X(ii) = truck.pPos.X(ii) - 2*pi;
        end
    end
end

% Pose of the robot's center
truck.pPos.Xc([1 2 6]) = truck.pPos.X([1 2 6]) - ...
    [cos(truck.pPos.X(6)) -sin(truck.pPos.X(6)) 0; sin(truck.pPos.X(6)) cos(truck.pPos.X(6)) 0; 0 0 1]*...
    [truck.pPar.a*cos(truck.pPar.alpha); truck.pPar.a*sin(truck.pPar.alpha); 0];
