classdef OptiTrack < matlab.mixin.SetGet % Handle
    % OptiTrack handle class for establishing communication with OptiTrack
    % Motive/Arena or similiar software
    %
    %   obj = OptiTrack creates an OptiTrack object to establish
    %   communication with OptiTrack Motive and/or Arena software.
    %
    % OptiTrack Methods
    %   Initialize  - Initialize the OptiTrack object.
    %   get         - Update and query properties of the OptiTrack object.
    %   delete      - Uninitialize and remove the OptiTrack object.
    %
    % OptiTrack Properties
    %   Client      - NatNet client (NatNetSDK specified)
    %   Frame       - Motion capture data frame (NatNetSDK specified)
    %   FrameRate   - Frame rate (frames per second)
    %   RigidBody   - 1xN structured array with rigid body information
    %       Name            - Rigid body name (e.g. 'Rigid Body 1')
    %       FrameIndex      - Frame index value (Motive/Arena assigned)
    %       TimeStamp       - Frame time stamp (seconds)
    %       FrameLatency    - TBD (seconds)
    %       isTracked       - 'True' if the object is currently being
    %                         tracked, 'False' otherwise
    %       Position        - Rigid body [x; y; z] position (millimeters)
    %       Quaternion      - Rigid body orientation represented using a 
    %                         quaternion [rb.qw, rb.qx, rb.qy, rb.qz]
    %       Rotation        - Rigid body orientation represented using a 
    %                         3x3 rotation matrix (also referred to as a
    %                         Directed Cosine Matrix or element of the
    %                         Special Orthogonal Group SO(3))
    %       HgTransform     - Rigid body position and orientation
    %                         represented using a 4x4 homogeneous rigid  
    %                         body transformation (also referred to as an
    %                         element of the Special Euclidean Group
    %                         SE(3))
    %       MarkerPosition  - Marker positions relative to the global 
    %                         coordinate frame (millimeters)
    %           MarkerPosition(:,i)     - [x; y; z] position of ith marker
    %       MarkerSize      - Marker diameters (millimeters)
    %           MarkerSize(:,i)         - diameter of the ith marker
    %
    %   Status              - Current status of OptiTrack object
    %   RigidBodySettings   - User specified rigid body settings
    %       DisplayName             - User defined display name for rigid
    %                                 body. This defaults to the rigid body
    %                                 name.
    %       Color                   - User defined rigid body color.
    %                                 Standard graphics objects colors are
    %                                 acceptable (e.g. 'b' or normalized
    %                                 3-element array). This defaults to
    %                                 blue.
    %       MarkerPosition          - Redundant parameter (see RigidBody)
    %       MarkerDesignPosition    - User specified marker positions used
    %                                 to determine an offset relating the
    %                                 body-fixed frame assigned in tracker
    %                                 to a design frame. Markers must be
    %                                 assigned in the same order used to
    %                                 define the rigid body in tracker.
    %                                 This defaults to the Motive-assigned
    %                                 marker positions.
    %       HgOffset                - Calculated using MarkerPosition and 
    %                                 MarkerDesignPosition to relat the
    %                                 body-fixed frame assigned in tracker
    %                                 to the design frame.
    %
    % Example:  
    %
    %       % Create, initialize, and visualize
    %       OTobj = OptiTrack;
    %       OTobj.Initialize;
    %       rigidBody = OTobj.RigidBody;
    %       for i = 1:numel(rigidBody)
    %           triad('Matrix',rigidBody.HgTransform);
    %       end
    %
    % See also PLOTRIGIDBODY
    %
    %   M. Kutzer & L. DeVries 14Jan2016, USNA
    
    % Updates
    %   17Feb2016 - Included calling function path for NatNet support
    
    % --------------------------------------------------------------------
    % General properties
    % --------------------------------------------------------------------
    properties(GetAccess='public', SetAccess='private')
        Client      % OptiTrack client created using NatNet
        Frame       % Current OptiTrack frame
        FrameRate   % Current OptiTrack frame rate
        RigidBody   % Rigid body information
        Status      % Client connection status
    end % end properties
    
    properties(GetAccess='public', SetAccess='public')
        RigidBodySettings   % User specified rigid body settings
    end % end properties
    
    % --------------------------------------------------------------------
    % Constructor/Destructor
    % --------------------------------------------------------------------
    methods(Access='public')
        function obj = OptiTrack
            % Create OptiTrack object.
            obj.Status = 'Disconnected';
        end
        
        function delete(obj)
            % delete function destructor
            switch lower(obj.Status)
                case 'ready'
                    fprintf('Uninitializing OptiTrack object...');
                    obj.Client.Uninitialize;
                    obj.Client = [];
                    obj.Frame = [];
                    obj.Status = 'Deleted';
                    fprintf('[COMPLETE]\n');
            end
        end
    end % end methods
    
    % --------------------------------------------------------------------
    % Initialization
    % --------------------------------------------------------------------
    methods(Access='public')
        function Initialize(obj,varargin)
            % Initialize initializes an OptiTrack client assuming the
            % NatNet server is set to local loop-back (127.0.0.1) and there
            % is a multicast connection.
            %
            % Note this is the case if the current instance of MATLAB and
            % Motive/Arena are running on the same machine.
            %
            % Initialize(obj,IP) initializes an OptiTrack client for a
            % designated Host IP address with a multicast connection.
            %
            % Initialize(obj,IP,ConnectionType) initializes an OptiTrack
            % client for a designated Host IP address, and a specified
            % connection type {'Multicast', 'Unicast'}.
            
            % Check inputs
            %narginchk(1,3);
            cType = 0;  % Default connection type to multicast
            if nargin >= 2
                % Designated host IP
                IP = varargin{1};
            else
                % Local loop-back
                IP = '127.0.0.1';
            end
            if nargin >= 3
                % Define connection type
                switch lower(varargin{2})
                    case 'multicast'
                        cType = 0;
                    case 'unicast'
                        cType = 1;
                    otherwise
                        error('OptiTrack:Init:BadConnectionType',...
                            'Connection property "%s" not recognized.',varargin{2});
                end
            end
            
            % Check IP
            % TODO - check for valid IP address
            if ~ischar(IP)
                error('OptiTrack:Init:BadIP',...
                    'The host IP must be specified as a character/string input (e.g. ''192.168.1.1'').');
            end
            
            % Check operating system to set dllPath
            % Get function path
            funcPath = mfilename('fullpath');
            [tboxPath,~,~] = fileparts(funcPath);
            % TODO - consider non-Windows
            OS = computer;
            switch lower(OS)
                case 'pcwin'
                    % 32-bit Windows
                    dllPath = fullfile(tboxPath,'OptiTrackToolboxSupport','NatNetSDK','lib','NatNetML.dll');
                case 'pcwin64'
                    % 64-bit Windows
                    dllPath = fullfile(tboxPath,'OptiTrackToolboxSupport','NatNetSDK','lib','x64','NatNetML.dll');
                otherwise
                    error('OptiTrack:Init:BadOS','Non-Windows OS detected: %s.',OS);
            end
            
            % Check if dllPath is valid
            if exist(dllPath,'file') ~= 2
                error('OptiTrack:Init:NoSDK',...
                    ['Required NatNetSDK files were not found.\n',...
                    ' -> Download and install the NatNetSDK to the current\n',...
                    '    directory from the following URL:\n\n',...
                    '    https://www.naturalpoint.com/optitrack/products/natnet-sdk/']);
            end
            
            % Add DLL path to .NET assembly
            NET.addAssembly(dllPath);
            
            % Initialize NatNet client
            client = NatNetML.NatNetClientML(cType);
            
            % Set the IP
            errFlag = client.Initialize(IP,IP);
            if errFlag
                client.Uninitialize;
                error('OptiTrack:Init:Failed','Failed to initialize the NatNet client.');
            end
            
            % Update class properties
            obj.Client = client;
            obj.Status = 'Ready';
            % Update rigid body settings
            rigidBody = obj.RigidBody;
            for i = 1:numel(rigidBody)
                rigidBodySettings(i).DisplayName = rigidBody(i).Name;
                rigidBodySettings(i).Color = 'b';
            end
            % Return empty set if no rigid bodies exist
            if ~exist('rigidBodySettings','var')
                rigidBodySettings(1).DisplayName = [];
                rigidBodySettings(1).Color = [];
            end
            % Update rigid body settings
            obj.RigidBodySettings = rigidBodySettings;
            
        end
    end % end methods
    
    % --------------------------------------------------------------------
    % Getters/Setters
    % --------------------------------------------------------------------
    methods
        function client = get.Client(obj)
            % Get client from OptiTrack object
            client = obj.Client;
        end
        
        function frame = get.Frame(obj)
            % Get current frame from OptiTrack client
            frame = obj.Client.GetLastFrameOfData();
            obj.Frame = frame;
        end
        
        function frameRate = get.FrameRate(obj)
            % Get current frame rate from OptiTrack client
            [byteArray, retCode] = obj.Client.SendMessageAndWait('FrameRate');
            if(retCode == 0)
                byteArray = uint8(byteArray);
                frameRate = typecast(byteArray,'single');
            else
                frameRate = [];
            end
            obj.FrameRate = frameRate;
        end
        
        function rigidBody = get.RigidBody(obj)
            % Get rigid body
            frame = obj.Frame;
            rigidBody = frame2rigidBody(frame);
            obj.RigidBody = rigidBody;
        end
        
        function status = get.Status(obj)
            % Get current client status
            status = obj.Status;
        end
        
        function rigidBodySettings = get.RigidBodySettings(obj)
            % Get rigid body settings
            rigidBodySettings = obj.RigidBodySettings;
        end
  
    end % end methods
    
    %% ********************************************************************
    % AuRoRa Functions here  

%% ********************************************************************  
 
   
end % end classdef

function rigidBody = frame2rigidBody(frame)
% Parse rigid body information from NatNetSDK frame
n = frame.nRigidBodies;
rigidBody = [];
for i = 1:n
    rb = frame.RigidBodies(i);
    % -> Get general rigid body information
    % Rigid body name
    rigidBody(i).Name = char(frame.MarkerSets(i).MarkerSetName);
    % Frame index (uint)
    rigidBody(i).FrameIndex = frame.iFrame;
    % Time stamp (seconds)
    rigidBody(i).TimeStamp = frame.fTimestamp;
    % Frame latency (seconds)
    rigidBody(i).FrameLatency = frame.fLatency;
    % Tracking status (binary)
    rigidBody(i).isTracked = logical(rb.Tracked);
    % Check if rigid body is tracked
    if rb.Tracked
        % Update info for tracked bodies
        % -> Native OptiTrack Info
        % Rigid body origin *relative* to the global reference frame
        rigidBody(i).Position = double([rb.x; rb.y; rb.z])*1000;
        % Quaternion representing the rigid body orientation *relative* to
        %   the global frame.
        % NOTE: The NatNetSDK natively returns components of the quaternion
        %   representing global frame orientation *relative* to the body 
        %   frame (i.e. the inverse of what is provided here). This change
        %   was made for consistency.
        rigidBody(i).Quaternion = ...
            quatinv( double([rb.qw, rb.qx, rb.qy, rb.qz]) );
        % -> Calculate rotation
        % Rotation matrix representing the orientation of the rigid body 
        %   *relative* to the global frame.
        rigidBody(i).Rotation = quat2dcm(rigidBody(i).Quaternion);
        % -> Create hgtransform
        rigidBody(i).HgTransform = ...
            [rigidBody(i).Rotation,rigidBody(i).Position;0,0,0,1];
        m = rb.nMarkers;
        % -> Marker info
        rigidBody(i).MarkerPosition = zeros(3,m);
        rigidBody(i).MarkerSize = zeros(1,m);
        for j = 1:m
            mrk = rb.Markers(j);
            % Marker positions relative to the global frame (mm)
            rigidBody(i).MarkerPosition(:,j) = ...
                double([mrk.x; mrk.y; mrk.z])*1000;
            % Marker diameter (mm)
            rigidBody(i).MarkerSize(1,j) = double(mrk.size)*1000;
        end
    else
        % Return empty set if rigid body is not tracked
        rigidBody(i).Position = [];
        rigidBody(i).Quaternion = [];
        rigidBody(i).Rotation = [];
        rigidBody(i).HgTransform = [];
        rigidBody(i).MarkerPosition = [];
        rigidBody(i).MarkerSize = [];
    end
end
end % end function
