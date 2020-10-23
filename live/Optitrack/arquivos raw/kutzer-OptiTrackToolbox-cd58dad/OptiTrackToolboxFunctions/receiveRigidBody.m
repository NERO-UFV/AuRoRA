function RigidBody = receiveRigidBody(udpRs)
% RECEIVERIGIDBODY sends rigid body properties via a port designated by a
% UDP receiver.
%   RigidBody = RECEIVERIGIDBODY(udpR) receives the following rigid body
%   properties via a single port designated by "udpR"
%       RigidBody.Index
%       RigidBody.Name
%       RigidBody.TimeStamp
%       RigidBody.isTracked
%       RigidBody.Position
%       RigidBody.Quaternion
%       RigidBody.Rotation
%       RigidBody.HgTransform
%
%   RigidBody = RECEIVERIGIDBODY(udpRs) receives the following rigid body
%   properties via a cell of ports designated by "udpRs"
%       RigidBody(i).Index      - Rigid body index value (should match i)
%       RigidBody(i).Name
%       RigidBody(i).TimeStamp
%       RigidBody(i).isTracked
%       RigidBody(i).Position
%       RigidBody(i).Quaternion
%       RigidBody(i).Rotation
%       RigidBody(i).HgTransform
%
%   M. Kutzer, 03Nov2016, USNA

% Updates
%   07Nov2016 - Added multiple port input capability

%% Check inputs
% TODO - improve error handling
narginchk(1,1);

%% Check UDP Receiver
% Create cell array if a single UDP receiver is specified
if ~iscell(udpRs)
    udpRs = {udpRs};
end

% Check UDP receivers
for i = 1:numel(udpRs)
    udpR = udpRs{i};
    switch class(udpR)
        case 'dsp.UDPReceiver'
            % Valid UDPReceiver
        otherwise
            error('ReceiveRigidBody:BadReceiver',...
                ['Specified UDP Receiver must be valid.',...
                '\n\t-> Use "udpR = initReceiver(port);" with a valid port ',...
                '\n\t   number to create a valid UDP Receiver.']);
    end
end

%% Receive message
% Define message format
[~,rsvFormat] = msgFormatRigidBody;

for i = 1:numel(udpRs)
    udpR = udpRs{i};
    % Receive message
    dataReceived = step(udpR);
    % Convert message
    msgRsvd = char(dataReceived');
    
    % Display message
    %fprintf('Bytes Received: %d, Msg: ',numel(msgRsvd));
    %fprintf('%s\n',msgRsvd);
    
    % Parse message
    splitStr = regexp(msgRsvd,'\:','split');
    rsvFormats = regexp(rsvFormat,'\:','split');
    if numel(splitStr) == 3
        idx = sscanf(splitStr{1},rsvFormats{1},[1 1]);
        Name = splitStr{2};
        out = sscanf(splitStr{3},rsvFormats{3},[1,9]);
        TimeStamp  = out(1);
        isTracked  = out(2);
        if isTracked
            Position   = out(3:5);
            Quaternion = out(6:end);
        else
            Position   = NaN(1,3);
            Quaternion = NaN(1,4);
        end
        
        % Package message
        RigidBody(i).Index       = idx;
        RigidBody(i).Name        = Name;
        RigidBody(i).TimeStamp   = TimeStamp;
        RigidBody(i).isTracked   = isTracked;
        RigidBody(i).Position    = Position;
        RigidBody(i).Quaternion  = Quaternion;
        if isTracked
            RigidBody(i).Rotation    = quat2dcm(RigidBody(i).Quaternion);
            RigidBody(i).HgTransform = [RigidBody(i).Rotation,RigidBody(i).Position';0,0,0,1];
        else
            RigidBody(i).Rotation = NaN(3,3);
            RigidBody(i).HgTransform = NaN(4,4);
        end
    else
        RigidBody(i).Index       = [];
        RigidBody(i).Name        = [];
        RigidBody(i).TimeStamp   = [];
        RigidBody(i).isTracked   = [];
        RigidBody(i).Position    = [];
        RigidBody(i).Quaternion  = [];
        RigidBody(i).Rotation    = [];
        RigidBody(i).HgTransform = [];
    end
end

