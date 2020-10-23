function sendRigidBody(udpSs,RigidBody)
% SENDRIGIDBODY sends individual rigid body properties via a designated 
% UDP servers (one server per rigid body).
%   SENDRIGIDBODY(udpSs,RigidBody) sends the following rigid body 
%   properties over a UDP servers specified by "udpSs"
%
%   udpSs - cell array containing individual udpS elements tied to unique
%           ports (one for each rigid body).
%
%   RigidBody - rigid body cell array output from OptiTrack. Only the
%               following fields are sent:
%
%       RigidBody(i).Name
%       RigidBody(i).TimeStamp
%       RigidBody(i).isTracked
%       RigidBody(i).Position
%       RigidBody(i).Quaternion
%
%   M. Kutzer, 03Nov2016, USNA

% Updates
%   08Nov2016 - Added multiple port functionality

%% Check inputs
% TODO - improve error handling
narginchk(2,2);

% Check for matching number of UDP servers and rigid bodies
if numel(udpSs) ~= numel(RigidBody)
    if numel(udpSs) < numel(RigidBody)
        error('SendRigidBody:NotEnoughUDPS',...
            'The total number of UDP senders must meet or exceed the number of rigid bodies being sent.');
    else
        warning('There are more UDP senders specified than rigid bodies.');
    end
end

%% Check UDP Sender
for i = 1:numel(udpSs)
    udpS = udpSs{i};
    switch class(udpS)
        case 'dsp.UDPSender'
            % Valid UDPSender
        otherwise
            error('SendRigidBody:BadSender',...
                ['Specified UDP Sender must be valid.',...
                '\n\t-> Use "udpS = initSender(port);" with a valid port ',...
                '\n\t   number to create a valid UDP Sender.']);
    end
end

%% Send message
% Define message format
sndFormat = msgFormatRigidBody;

for i = 1:numel(RigidBody)
    % Create message
    if RigidBody(i).isTracked
        % Rigid body is tracked
        msg_Send = sprintf(sndFormat,...
            i,...
            strrep(RigidBody(i).Name,' ','_'),... % Replace white spaces with underscores (to simplify parsing)
            RigidBody(i).TimeStamp,...
            RigidBody(i).isTracked,...
            RigidBody(i).Position,...
            RigidBody(i).Quaternion);
    else
        % Rigid body is not tracked
        msg_Send = sprintf(sndFormat,...
            i,...
            strrep(RigidBody(i).Name,' ','_'),... % Replace white spaces with underscores (to simplify parsing)
            RigidBody(i).TimeStamp,...
            RigidBody(i).isTracked,...
            zeros(1,3),...                        % Default to zero position
            zeros(1,4));                          % Default to quaternion of all zeros
    end
    % Convert message
    dataSend = uint8(msg_Send);
    % Send message
    step(udpSs{i}, dataSend);
    % Display message
    %fprintf('%s\n',msg_Send);
end


