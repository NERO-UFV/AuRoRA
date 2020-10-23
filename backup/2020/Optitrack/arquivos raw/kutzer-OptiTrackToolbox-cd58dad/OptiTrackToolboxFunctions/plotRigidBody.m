function hg = plotRigidBody(varargin)
% PLOTRIGIDBODY create a visualization of an OptiTrack rigid body.
%   hg = PLOTRIGIDBODY(RigidBody) returns a handle (or handles) to an  
%   hgtransform object with children visualzing the markers and reference
%   frame of an OptiTrack rigid body. 
%
%   Note: "RigidBody" must be a 1xN structured array property of the 
%   OptiTrack class.
%
%   PLOTRIGIDBODY(RigidBody, RigidBodySettings) returns a handle (or  
%   handles) to an hgtransform object with children visualzing the markers 
%   and reference frame of an OptiTrack rigid body. RigidBodySettings is
%   used to specify marker color and tag each graphics object assoiciated 
%   with the rigid body.
%
%   Note: "RigidBodySettings" must be a 1xN structured array property of  
%   the OptiTrack class.
%
%   PLOTRIGIDBODY(axs,___) specify the parent axes of the visualization.
%
%   Example:
%       % Create and initialize OptiTrack object
%       OTobj = OptiTrack;
%       OTobj.Initialize;
%       % Create figure and axes (downward looking FOV)
%       fig = figure('Name','OptiTrack Object Example');
%       axs = axes('Parent',fig,'DataAspectRatioMode','manual',...
%           'DataAspectRatio',[1 1 1],'NextPlot','add','View',[180,0]);
%       % Plot rigid bodies
%       hg = plotRigidBody(axs,OTobj.RigidBody);
%       % Visualize rigid body movements
%       while true
%           % Exit loop when figure is closed
%           if ~ishandle(fig)
%               break
%           end
%           % Get current rigid body information
%           rb = OTobj.RigidBody;
%           % Update each rigid body 
%           for i = 1:numel(rb)
%               if rb(i).isTracked
%                   % Update rigid body pose if it is tracked
%                   set(hg(i),'Matrix',rb(i).HgTransform,'Visible','On');
%               else
%                   % Make visualization invisible if rigid body is not
%                   % tracked
%                   set(hg(i),'Visible','Off');
%               end
%           end
%           drawnow
%       end
%
%   See also OptiTrack triad hgtransform
%   
%   M. Kutzer 20Jan2016, USNA

% Upates:
%   10Mar2016 - Documentation update and added example
%   10Mar2016 - Empty-set error checking for objects that are not tracked

%% Parse inputs
narginchk(1,3);
% Assign output axes (or hgtransform)
if ishandle(varargin{1})
    axsOut = varargin{1};
    idx = 2;
else
    axsOut = gca;
    % Check class of first input
    switch lower( class(varargin{1}) )
        case 'struct'
            idx = 1; % treat first object as "RigidBody"
        otherwise
            % TODO - improve warning
            warning('Invalid graphics object specified. Ignoring first input.');
            idx = 2;
    end
end
% Assign rigid body
if nargin >= idx
    rigidBody = varargin{idx};
    idx = idx+1;
else
    error('Rigid body information must be specified (obj.RigidBody).');
end
% Assign rigid body settings
if nargin >= idx
   rbSettings = varargin{idx};
else
    % Set default values for rigid body settings.
    % NOTE: This is identical to the end of the obj.Initialize code in the 
    %   OptiTrack classdef. 
    for i = 1:numel(rigidBody)
        rbSettings(i).DisplayName = rigidBody(i).Name;
        rbSettings(i).Color = 'b';
        rbSettings(i).MarkerDesignPosition = ...
            rigidBody(i).MarkerPosition;
        rbSettings(i).HgOffset = eye(4);
    end
end

%% Create plot
% Create temporary invisible figure
%   Note: This is a quick workaround for determining the proper scale of
%   the reference frame for the rigid body.
fig = figure('Visible','Off');
axs = axes('Parent',fig,'NextPlot','add','DataAspectRatio',[1 1 1]);
view(axs,3);
% Define unit sphere surface coordinates
[X,Y,Z] = sphere(20);
for i = 1:numel(rigidBody)
    % Get marker diameter
    mSze = rigidBody(i).MarkerSize;
    % Apply rigid body settings
    tag = rbSettings(i).DisplayName;
    color = rbSettings(i).Color;
    
    % Get Marker position
    mPos = rigidBody(i).MarkerPosition;
    % Move marker position to body-fixed frame
    H = rigidBody(i).HgTransform;
    mPos(4,:) = 1;
    
    if isempty(H)
        error('OptiTrack:NotVisible',...
            ['Rigid Body "%s" is currently not visible.\n',...
            'Please move rigid body into field of view to continue.'],tag);
    else
        mPos = minv(H)*mPos;
    end
    
    % Plot markers
    n = numel(mSze);
    for j = 1:n
        ptch(j) = patch(surf2patch(...
            (mSze(j)/2)*X + repmat(mPos(1,j),size(X)),...
            (mSze(j)/2)*Y + repmat(mPos(2,j),size(Y)),...
            (mSze(j)/2)*Z + repmat(mPos(3,j),size(Z))),...
            'FaceColor',color,'EdgeColor','None','Parent',axs);
        set(ptch(j),'Tag',sprintf('%s, Marker %d',tag,j));
    end
    
    % Determine reference frame scale
    lims = [xlim(axs); ylim(axs); zlim(axs)];
    scle = max( lims(:,2) - lims(:,1) );
    
    % Create triad
    hg(i) = triad('Parent',axsOut,'Scale',1.2*scle,'LineWidth',2,...
        'Matrix',H,'Tag',tag);
    % Set markers to correct frame
    set(ptch,'Parent',hg(i));
end
% Delete temporary figure
delete(fig);
