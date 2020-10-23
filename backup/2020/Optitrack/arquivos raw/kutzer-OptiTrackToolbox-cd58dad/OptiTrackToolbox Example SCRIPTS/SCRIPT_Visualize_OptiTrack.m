% SCRIPT_Visualize_OptiTrack
%   Visualize tracked rigid body motion throughout the workspace of the
%   camera system. 
%
%   NOTE: This code assumes Data Streaming -> Up Axis -> Y Up in the Motive
%   Software.
%
%   M. Kutzer 20Jan2016, USNA

%% Clear workspace, close all figures, clear command window
clear all
close all
clc

%% Create OptiTrack object and initialize
obj = OptiTrack;
obj.Initialize;

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

%% Plot rigid bodies
%   Note: All tracked rigid bodies must be visible when plotting rigid 
%   bodies.
hg = plotRigidBody(axs,obj.RigidBody);

%% Create position "tails"
for i = 1:numel(hg)
    H = get(hg(i),'Matrix');
    plt(i) = plot3(H(1,4),H(2,4),H(3,4),'.','Color',rand(1,3),...
        'MarkerSize',3);
end

%% Visualize rigid body movements
while true
    % Exit loop when figure is closed
    if ~ishandle(fig)
        break
    end
    % Get current rigid body information
    rb = obj.RigidBody;
    % Update each rigid body
    for i = 1:numel(rb)
        if rb(i).isTracked
            % Update rigid body pose if tracked
            H = rb(i).HgTransform;
            set(hg(i),'Matrix',H,'Visible','On');
            % Update tail if tracked
            x = get(plt(i),'XData');
            y = get(plt(i),'YData');
            z = get(plt(i),'ZData');
            % TODO - consider limiting tail length
            set(plt(i),...
                'XData',[x,H(1,4)],...
                'YData',[y,H(2,4)],...
                'ZData',[z,H(3,4)]);
        else
            % Make visualization invisible if rigid body is not tracked
            set(hg(i),'Visible','Off');
        end
    end
    drawnow
end