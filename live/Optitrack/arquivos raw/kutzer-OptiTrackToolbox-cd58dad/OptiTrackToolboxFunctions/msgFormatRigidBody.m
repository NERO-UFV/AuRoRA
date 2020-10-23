function [sndFormat,rsvFormat] = msgFormatRigidBody
% MSGFORMATRIGIDBODY returns the message format used when sending an
% individual rigid body via UDP.
%   $1:Name:TimeStamp,isTracked,P(1),P(2),P(3),Q(1),Q(2),Q(3),Q(4)!'
%
%   M. Kutzer, 04Nov2016, USNA

% Updates

%% Create message format
%   $1:Name:TimeStamp,isTracked,P(1),P(2),P(3),Q(1),Q(2),Q(3),Q(4)!'
sndFormat = ['$',...                    % Message start character
             '%d:',...                  % Rigid Body Index
             '%s:',...                  % Rigid Body Name
             '%.3f,',...                % Frame Time Stamp
             '%d,',...                  % Rigid Body Tracked {0,1}
             '%.2f,%.2f,%.2f,',...      % Rigid Body Position (mm)
             '%.7f,%.7f,%.7f,%.7f',...  % Rigid Body Quaternion
             '!'];                      % Message end character
         
rsvFormat = ['$',...                    % Message start character
             '%d:',...                  % Rigid Body Index
             '%s:',...                  % Rigid Body Name
             '%f,',...                  % Frame Time Stamp
             '%d,',...                  % Rigid Body Tracked {0,1}
             '%f,%f,%f,',...            % Rigid Body Position (mm)
             '%f,%f,%f,%f',...          % Rigid Body Quaternion
             '!'];                      % Message end character
