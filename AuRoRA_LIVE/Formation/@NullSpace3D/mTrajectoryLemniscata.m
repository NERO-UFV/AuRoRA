function mTrajectoryLemniscata(obj,a,b,w,ta)
%MTRAJECTORYLEMNISCATA Summary of this function goes here
%   Detailed explanation goes here   

    % Trajectory
    % Bernoulli Leminiscata
    % Positions    
    obj.pPos.Qd(1)  = a*sin(w*ta);         % x position
    obj.pPos.Qd(2)  = b*sin(2*w*ta);       % y position
    %         obj.pPos.Qd(3)  = 0;                   % z position
    %         obj.pPos.Qd(4)  = 2;                   % rho position
    %         obj.pPos.Qd(5)  = deg2rad(0);          % alpha position
    %         obj.pPos.Qd(6)  = deg2rad(90);         % beta position
    
    % Velocities
    obj.pPos.dQd(1)  = a*w*cos(w*ta);      % x velocities
    obj.pPos.dQd(2)  = b*2*w*cos(2*w*ta);  % y velocities
    obj.pPos.dQd(3)  = 0;                  % z velocities
    obj.pPos.dQd(4)  = 0;                  % rho velocities
    obj.pPos.dQd(5)  = 0;                  % alpha velocities
    obj.pPos.dQd(6)  = 0;                  % beta velocities

end

