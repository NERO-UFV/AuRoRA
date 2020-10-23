% Adept MobileRobots Robotics Interface for Applications (ARIA)
% Copyright (C) 2004, 2005 ActivMedia Robotics LLC
% Copyright (C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
% Copyright (C) 2011, 2012, 2013 Adept Technology
% 
%      This program is free software; you can redistribute it and/or modify
%      it under the terms of the GNU General Public License as published by
%      the Free Software Foundation; either version 2 of the License, or
%      (at your option) any later version.
% 
%      This program is distributed in the hope that it will be useful,
%      but WITHOUT ANY WARRANTY; without even the implied warranty of
%      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%      GNU General Public License for more details.
% 
%      You should have received a copy of the GNU General Public License
%      along with this program; if not, write to the Free Software
%      Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
% 
% If you wish to redistribute ARIA under different terms, contact 
% Adept MobileRobots for information about a commercial version of ARIA at 
% robots@mobilerobots.com or 
% Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960

% simple example that drives robot to a series of positions

points = {{3000, 0}, {3000, 3000}, {0, 3000}, {0, 0}}
velMult = 0.5
distThresh = 350 % mm
angleThresh = 0.5 % deg

%aria_init -rh 10.0.200.42 -ris
aria_init
arrobot_connect()
arrobot_resetpos()
curPoint = 1

while (true)
  % get current robot position from aria
  rx = arrobot_getx
  ry = arrobot_gety

  % Check if we've reached goal point
  p = points{curPoint}
  px = p{1}
  py = p{2}
  d = sqrt( (px - rx)^2 + (py - ry)^2 )
  disp(sprintf(' distance remaining: %f', d))
  if d <= distThresh
    disp 'reached point'
    arrobot_stop

    % choose next point in list
    curPoint = curPoint + 1
    if curPoint > length(points)
      curPoint = 1
    end
    p = points{curPoint} 
    px = p{1}
    py = p{2}

    % determine how far to rotate to face this next point
    rt = arrobot_getth
    dt =  (atan2(py - ry, px - rx) * (180/3.14159)) - rt
    dt = mod((dt + 180), 360) - 180 % restrict to (-180,180)

    
    % restrict to (-180,180)
%     if (dt >= 360)
%         dt = dt - 360.0 * (dt / 360)
%     end
%     if (dt < -360)
%         dt = dt + 360.0 * (dt / -360)
%     end
%     if (dt <= -180)
%         dt = + 180.0 + (dt + 180.0)
%     end
%     if (dt > 180)
%         dt = - 180.0 + (dt - 180.0)
%     end

    % request robot to turn
    disp(sprintf('Turning by %d deg.', dt))
    arrobot_setdeltaheading(dt)

    % wait until heading change is achieved within specified threshold
    target = rt + dt
    target = mod((target + 180), 360) - 180 % restrict to (-180,180)
    while abs(arrobot_getth - target) > angleThresh  % note, get newest robot theta from aria each time
      disp(sprintf(' angle remaining %f', abs(arrobot_getth - target)))
      pause(0.08)
    end
    disp 'Heading done.'
  else
    % request robot velocity proportionally to distance remaining
    arrobot_setvel(d*velMult)
  
    % Could also set delta heading or rotational velocity here to continuously
    % correct angle as well.
    
    % Could check for obstacles from sonar, bumpers and wheel stall here as
    % well.
    
  end
  pause(0.08)
end


