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

funcs = {
    'aria_init',  
    'aria_shutdown',
    'arrobot_command',
    'arrobot_connect', 
    'arrobot_disconnect', 
    'arloginfo', 
    'arrobot_setvel', 
    'arrobot_setrotvel', 
    'arrobot_getx', 
    'arrobot_gety', 
    'arrobot_getth', 
    'arrobot_stop',
    'arrobot_length',
    'arrobot_width',
    'arrobot_radius',
    'arrobot_getsonarrange',
    'arrobot_getnumsonar',
    'arrobot_getdigin',
    'arrobot_setlatvel',
    'arrobot_getbatteryvoltage',
    'arrobot_getvel',
    'arrobot_getrotvel',
    'arrobot_getlatvel',
    'arrobot_setdeltaheading',
    'arrobot_resetpos',
    'arrobot_isstalled',
    'arrobot_isleftstalled',
  	'arrobot_isrightstalled',
    'arrobot_setwheelvels',
    'arrobot_setlatvel',
    'arrobot_setvels',
    'arrobot_getpose',
    'arrobot_setpose',
    'arrobot_move',
    'arrobot_motorsenabled',
    'arrobot_enable_motors',
    'arrobot_disable_motors'
}

% Unload old mex functions etc.  Note, if you add a new mex function above that
% does not match these patterns, add it to this list or a pattern that
% matches it.
clear aria_* arloginfo arrobot_*

 
% Set filenames and options depending on current platform
def = '';
switch computer
    case 'PCWIN'
		ariainstdir = 'C:/Program Files/MobileRobots/Aria'
        ariaclink = '-L. -lariac_vc10_i386'
        arialink = '-L../lib -L../bin -lAriaVC10'
        def = '-DWIN32 -win32'
        ariadll = '../bin/AriaVC10.dll'
        username = getenv('USERNAME')
        hostname = getenv('COMPUTERNAME')
		disp 'You are on Windows 32 bit. Will attempt to use AriaVC10.dll and ariac_vc10_i386.lib, built in Release configuration for win32 platform with Visual C++ 2010.';
    case 'PCWIN64'
		ariainstdir = 'C:/Program Files/MobileRobots/Aria'
        ariaclink = '-L. -lariac_vc10_x64'
        def = '-DWIN32'
        arialink = '-L../lib64 -L../bin64 -lAriaVC10'
        ariadll = '../bin64/AriaVC10.dll'
        username = getenv('USERNAME')
        hostname = getenv('COMPUTERNAME')
		disp 'You are on Windows 64 bit. Will attempt to use AriaVC10.dll and ariac_vc10_x64.lib, built in Release configuration for x64 platform with Visual C++ 2010.';
    case 'GLNX86'
		ariainstdir = '/usr/local/Aria'
        ariaclink = '-L. -lariac'
        arialink = '-L../lib -lAria'
        ariadll = '../lib/libAria.so'
        username = getenv('USER')
        hostname = getenv('HOSTNAME')
		disp 'You are on Linux 32 bit. Will attempt to use libAria.so and libariac.so.';
    case 'GLNXA64'
		ariainstdir = '/usr/local/Aria'
        ariaclink = '-L. -lariac'
        arialink = '-L../lib -lAria'
        ariadll = '../lib/libAria.so'
        username = getenv('USER')
        hostname = getenv('HOSTNAME')
		disp 'You are on Linux 64 bit. Will attempt to use libAria.so and libariac.so.';
	case 'MACI64'
	    disp 'Sorry, not set up for Mac yet. You will need to get ARIA and ariac built on Mac, then edit makemex.m to set the appropriate mex compilation flags under the MACI64 computer type case instead of displaying this message..';
        return
    otherwise
        disp 'error, unrecognized system type. (what kind of computer and OS are you on?)';
        computer
		return
end

% Put a copy of the ARIA runtime library into the current directory
disp 'Copying ARIA DLL into current directory so Matlab can easily find it...';
eval(['copyfile ' ariadll ' .'])

% Compile all the mex functions listed above:
disp 'Compiling mex functions...'
for i = 1:length(funcs);
  disp(funcs{i});
  cmd = sprintf('mex -g %s -DMATLAB -I. %s %s mex-src/%s.c', def, ariaclink, arialink, funcs{i});
  cmd;
  eval(cmd);
end;

disp 'Writing build info to file buildinfo.txt'
fp = fopen('buildinfo.txt', 'w');
fprintf(fp, 'ARIA Matlab Interface built %s by %s@%s\nMatlab %s %s\nUsed ARIA DLL %s with link flags %s %s\n', date, username, hostname, version, computer, ariadll, ariaclink, arialink);
fclose(fp);

disp 'done'
