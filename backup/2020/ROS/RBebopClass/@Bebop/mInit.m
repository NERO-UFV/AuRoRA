% Initialize variables
function mInit(obj,ID,iNamespace)
            
    % Classic Drone Variables
    if nargin < 1
        ID = 1;
        iNamespace = 'bebop';
    end

    obj.pID = ID; 
        
    iControlVariables(obj);
    iParameters(obj);
%     mCADload(obj);                

    
    % ROS inicialize Variables
    % Inicialize Variables        
    obj.pWorkSpaceName = '/bebop_ws'; 
    obj.pNamespace     = iNamespace;
    
    % Inicialize Messages Txt Types
    obj.pTxtTopic.Odom      = strcat('/',obj.pNamespace,'/odom');        % '/bebop/odom';
    obj.pTxtTopic.OdomLocal = strcat('/',obj.pNamespace,'/new_odom');    % '/bebop/new_odom';
    obj.pTxtTopic.OdomOpt   = strcat('/',obj.pNamespace,'/opt_odom');    % '/bebop/new_odom';
    obj.pTxtTopic.Land      = strcat('/',obj.pNamespace,'/land');        % '/bebop/new_odom';
    obj.pTxtTopic.Echo      = 'echo';     
    
    % Inicialize Messages    
    %msStopVel = rosmessage(obj.pStdMsgGeoType);
    obj.pStdMsgEmpty   = rosmessage('std_msgs/Empty');   
    obj.pOdom          = rosmessage('nav_msgs/Odometry');       
    obj.pVel           = rosmessage('geometry_msgs/Twist');
%     obj.Marker         = rosmessage('visualization_msgs/MarkerArray');
%     obj.Marker         = rosmessage('std_msgs/Empty');

    
    % Inicialize Publishers
    obj.pubBrTakeoff   = rospublisher(strcat('/',obj.pNamespace,'/takeoff'));
    obj.pubBrLand      = rospublisher(strcat('/',obj.pNamespace,'/land'));
    obj.pubCmdVel      = rospublisher(strcat('/',obj.pNamespace,'/cmd_vel')); 
            
    % Inicialize Subscribers
    obj.subOdom        = rossubscriber(obj.pTxtTopic.Odom ,'nav_msgs/Odometry');
    obj.subOdomLocal   = rossubscriber(obj.pTxtTopic.OdomLocal,'nav_msgs/Odometry');
    obj.subOdomOpt     = rossubscriber(obj.pTxtTopic.OdomOpt,'nav_msgs/Odometry');
    obj.subLand        = rossubscriber(obj.pTxtTopic.Land,'std_msgs/Empty',@obj.Callback_EmgStop);
    obj.subMarker      = rossubscriber(strcat('/',obj.pNamespace,'/whycon_lab'),'geometry_msgs/Pose');
    %obj.pFlag.EmergencyStop
end

