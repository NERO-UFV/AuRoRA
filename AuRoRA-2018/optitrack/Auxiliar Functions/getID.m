function ID = getID(OPT,obj,number)
% This is a function to identify the rigid body ID from optitrack.
% Intputs:
%   OPT    - Optitrack class
%   obj    - Object class(Pioneer, Ardrone,Load,Obstacle)
%   number - object number, when there are more than one objects from the
%   same class. For example, a second ArDrone will  be number = 2.
%   In this case, the function will look for the last character of the rigid
%   body. Then the rigid body first and last characteres on OptiTrack  must
%   be the Class first letter(P,A,O,L) and the number of the object,
%   respectively.
%
%  *If the user does not input a number, will be consider that exists
% only one object of that type for the search
%
% Output =  ID of the object

name = class(obj);           % get object name
rb   = OPT.RigidBody;        % read optitrack data
n    = length(rb);           % number of rigid bodies
ID   = [];                   % initialize ID

if nargin < 3
    % only one type of the object
    for ii = 1:n
        type = rb(ii).Name;
        if  strcmpi(type(1),name(1))   % case insensitive
            ID = ii;
            return
        else
            ID = [];           
        end
    end
    
else
    % more than one object of the same kind, compares the first and last letters of
    % the rigid body name
    number = num2str(number);    % convert number to char
    for ii = 1:n
        type = rb(ii).Name;      % gets optitrack rigid body's name
        if  strcmpi(type(1),name(1)) && type(end) == number
            ID = ii;
            return
        else
            ID = [];
        end
    end
    
end


% Case object type do not match with any Optitrack rigid body
if isempty(ID)
    disp('Rigid body type not known. Consider changing the rigid body name on OptiTrack');
end