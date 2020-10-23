
function mTrajectoryEllipse(obj,a,b,w,ta)
%MTRAJECTORYCIRCLE Summary of this function goes here
%   Detailed explanation goes here

    % Trajectory
    % Ellipse (0) and Circle (o)
    % Positions      
    obj.pPos.Qd(1)  = a*cos(w*ta);                   % posição x
    obj.pPos.Qd(2)  = b*sin(w*ta);                   % posição y
    %         obj.pPos.Qd(3)  = 0;                   % z position
    %         obj.pPos.Qd(4)  = 2;                   % rho position
    %         obj.pPos.Qd(5)  = deg2rad(0);          % alpha position
    %         obj.pPos.Qd(6)  = deg2rad(90);         % beta position

    
    % Velocities
    obj.pPos.dQd(1) = -a*w*sin(w*ta);                % velocidade em x
    obj.pPos.dQd(2) =  b*w*cos(w*ta);                % velocidade em y
    obj.pPos.dQd(3)  = 0;                            % z velocities
    obj.pPos.dQd(4)  = 0;                            % rho velocities
    obj.pPos.dQd(5)  = 0;                            % alpha velocities
    obj.pPos.dQd(6)  = 0;                            % beta velocities


end

