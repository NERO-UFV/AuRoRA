%% SCRIPT_Test_receiveRigidBody
% Delete existing udpR
if exist('udpRs')
    for i = 1:numel(udpRs)
        udpR = udpRs{i};
        delete(udpR);
    end
end
clear udpRs

% Designate port (must match sender)
port0 = 3100;
% Define number of rigid bodies
n = 7;
% Initialize receivers
for i = 1:7
    udpRs{i} = initReceiver(port+i);
end

%%
while true
    fprintf('------------------- RECEIVING DATA --------------------\n');
    for i = 1:7
        RigidBody = receiveRigidBody(udpRs{i});
    end
    fprintf('-------------------- DATA RECEIVED --------------------\n');
    pause(0.01);
end
