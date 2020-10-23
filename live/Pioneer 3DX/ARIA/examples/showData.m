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


% This example just displays data from the robot, does not cause it to move.
% You can use the joystick to move the robot, or call arrobot_ functions in
% the Matlab command window.  (Use arrobot_stop to stop the robot moving.)

% TODO: show examples of plotting data.


%aria_init -rh 10.0.200.42 -ris
aria_init
arrobot_connect

disp(sprintf('\nConnected to robot.\n\nRobot and ARIA information:\nRadius=%d, Length=%d, Width=%d, NumSonar=%d.', arrobot_radius, arrobot_length, arrobot_width, arrobot_getnumsonar))

arloginfo


while (true)

  disp(sprintf('x:%f y:%f th:%f vel:%f rotvel:%f enabled:%d stall:%d battv:%f', arrobot_getx, arrobot_gety, arrobot_getth, arrobot_getvel, arrobot_getrotvel, arrobot_motorsenabled, arrobot_isstalled, arrobot_getbatteryvoltage))

  pause(2)
 
end


