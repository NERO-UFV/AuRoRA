function sKinematicModel(p3dx)

% Determine the robot pose, based on the control signal
%      +-----------+      .
% U -> | Kinematic |  ->  X
%      | Model     |      
%      +-----------+      
%

K = [cos(p3dx.pPos.X(6)) -p3dx.pPar.a*sin(p3dx.pPos.X(6)+p3dx.pPar.alpha); ...
     sin(p3dx.pPos.X(6))  p3dx.pPar.a*cos(p3dx.pPos.X(6)+p3dx.pPar.alpha); ...
             0                             1                   ];

% Current position
p3dx.pPos.X([1 2 6]) = p3dx.pPos.X([1 2 6]) + K*p3dx.pSC.U(1:2)*p3dx.pPar.Ts;

% first-time derivative of the current position
p3dx.pPos.X([7 8 12]) = K*p3dx.pSC.U(1:2);

% Angle limitation per quadrant
for ii = 4:6
    if abs(p3dx.pPos.X(ii)) > pi
        if p3dx.pPos.X(ii) < 0
            p3dx.pPos.X(ii) = p3dx.pPos.X(ii) + 2*pi;
        else
            p3dx.pPos.X(ii) = p3dx.pPos.X(ii) - 2*pi;
        end
    end
end

% Pose of the robot's center
p3dx.pPos.Xc([1 2 6]) = p3dx.pPos.X([1 2 6]) - ...
    [cos(p3dx.pPos.X(6)) -sin(p3dx.pPos.X(6)) 0; sin(p3dx.pPos.X(6)) cos(p3dx.pPos.X(6)) 0; 0 0 1]*...
    [p3dx.pPar.a*cos(p3dx.pPar.alpha); p3dx.pPar.a*sin(p3dx.pPar.alpha); 0];
