%% SCRIPT_ReceiveRigidBodies

%% Define number of rigid bodies
n = 2;

%% Delete old receivers
if exist('udpSs','var')
    for i = 1:numel(udpSs)
        delete(udpSs{i});
    end
end

%% Setup receiver
port0 = 31000;
for i = 1:n
    % Define and open port
    port = port0 + i;
    udpRs{i} = initReceiver(port);
    % Display port information for each rigid body
    fprintf('---- Rigid Body %d ----\n',i);
    fprintf('\t     Port Number: %d\n',port);
end

%% Create and setup figure and axes (downward looking FOV)
fig = figure('Name','Visualize OptiTrack Example');
axs = axes('Parent',fig,'DataAspectRatioMode','manual',...
	'DataAspectRatio',[1 1 1],'NextPlot','add','View',[180,0]);

% Label axes
xlabel(axs,'x (mm)');
ylabel(axs,'y (mm)');
zlabel(axs,'z (mm)');

% Update limits to match tracking volume
%   Note: These will change based on setup and definition of the ground
%   plane in tracking software.
xx = [-5400, 6200]; % (mm)
yy = [    0, 3200]; % (mm), Floor to ceiling
zz = [-3500, 3500]; % (mm)
set(axs,'xlim',xx,'ylim',yy,'zlim',zz);

%% Create rigid body visualization
for i = 1:n
    hg(i) = triad('Parent',axs,'Scale',200,'LineWidth',2);
end

%% Receive data
while ishandle(fig)
    % Get current rigid body information
    RigidBody = receiveRigidBody(udpRs);
    pause(0.001);
    % Display rigid body
    for i = 1:numel(RigidBody)
        if ~isempty(RigidBody(i).Index)
            if RigidBody(i).isTracked
                set(hg(i),'Matrix',RigidBody(i).HgTransform);
            end
        end
    end
end