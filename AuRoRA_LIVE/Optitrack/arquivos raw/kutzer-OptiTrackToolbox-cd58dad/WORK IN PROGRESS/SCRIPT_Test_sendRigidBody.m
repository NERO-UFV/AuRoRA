%% SCRIPT_Test_sendRigidBody
% Delete existing udpS
if exist('udpSs')
    for i = 1:numel(udpSs)
        udpS = udpSs{i};
        delete(udpS);
    end
end
clear udpSs

% Designate port starting value
port0 = 31000;
% Define number of rigid bodies
n = 7;
% Initialize senders
for i = 1:n
    udpSs{i} = initSender(port0+i);
end

%% Create default rigid bodies for sending
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
    % Send rigid body
    sendRigidBody(udpSs,RigidBody);
    % Display update to command window
    fprintf('Frame %d Sent, t = %f\n',[frame,t]);
    pause(0.01);
end