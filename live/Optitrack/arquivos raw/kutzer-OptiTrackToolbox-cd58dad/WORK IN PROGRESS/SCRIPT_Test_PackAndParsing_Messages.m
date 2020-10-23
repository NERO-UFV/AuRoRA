%% SCRIPT_Test_PackAndParsing_Messages
% Define message format
[sndFormat,rsvFormat] = msgFormatRigidBody;

tic;
for frame = 1:500
    % Create simulated rigid body
    t = toc;
    for i = 1:7
        RigidBody(i).Name = sprintf('Rigid Body %d',i);
        RigidBody(i).TimeStamp = t;
        RigidBody(i).isTracked = true;
        RigidBody(i).Position = i*rand(1,3);
        RigidBody(i).Quaternion = rand(1,4);
    end
    
    % Pack messages for sending
    for i = 1:7
        if RigidBody(i).isTracked
            % Tracked rigid body
            msgSend = sprintf(sndFormat,...
                i,...
                strrep(RigidBody(i).Name,' ','_'),... % Replace white spaces with underscores (to simplify parsing)
                RigidBody(i).TimeStamp,...
                RigidBody(i).isTracked,...
                RigidBody(i).Position,...
                RigidBody(i).Quaternion);
        else
            % Not tracked rigid body
            msgSend = sprintf(sndFormat,...
                i,...
                strrep(RigidBody(i).Name,' ','_'),... % Replace white spaces with underscores (to simplify parsing)
                RigidBody(i).TimeStamp,...
                RigidBody(i).isTracked,...
                zeros(1,3),...
                zeros(1,4));
        end
        
        % Receive message
        msgRsvd = msgSend;
        
        % Display original message
        fprintf('Msg: %s\n',msgRsvd);
        
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
        end
        
        % Display parsed message
        fprintf(['Prs: ',sndFormat,'\n'],...
            idx,...
            Name,...
            TimeStamp,...
            isTracked,...
            Position,...
            Quaternion);
        pause(0.01);
        %pause
    end
    
end