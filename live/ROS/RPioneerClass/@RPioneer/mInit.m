% Initialize variables
function mInit(obj,ID,iNamespace,flagTrailer)
            
    % Classic Drone Variables
    if nargin < 3
        ID = 1;
        iNamespace = 'P';        
    end

    obj.pID = ID; 
    
    obj.pFlag.Connected = 1;
    obj.pFlag.isTracked = 1;
    obj.pFlag.Trailer = flagTrailer;

    iControlVariables(obj);
    iParameters(obj);

%     mCADload(obj);            
    
    
    % ROS inicialize Variables
    % Inicialize Variables        
    obj.pWorkSpaceName = 'RosAria'; 
    obj.pNamespace     = iNamespace;      
    
    
    % Inicialize Messages Txt Types            
    obj.pTxtTopic.OdomOpt   = strcat('/',obj.pNamespace,'/opt_odom');    % '/RosAria/new_odom';    
    obj.pTxtTopic.Echo      = 'echo';  
    
    % Inicialize Messages    
    %msStopVel = rosmessage(obj.pStdMsgGeoType);

    obj.pOdom        = rosmessage('nav_msgs/Odometry');       
    obj.pVel         = rosmessage('geometry_msgs/Twist');
    obj.pPot1         = rosmessage('std_msgs/Float64');
    obj.pPot2         = rosmessage('std_msgs/Float64');
    
    % Inicialize Publishers    
    obj.pubCmdVel    = rospublisher(strcat('/',obj.pNamespace,'/cmd_vel')); 
    
    % Inicialize Subscribers
    obj.subOdomOpt   = rossubscriber(obj.pTxtTopic.OdomOpt,'nav_msgs/Odometry');
    obj.subOdom      = rossubscriber(strcat('/',obj.pNamespace,'/pose'),'nav_msgs/Odometry');
    if flagTrailer == 2
        obj.subPot1       = rossubscriber(strcat('/trailer_angle_1'),'std_msgs/Float64');
        obj.subPot2       = rossubscriber(strcat('/trailer_angle_2'),'std_msgs/Float64');
    elseif flagTrailer == 1
        obj.subPot1       = rossubscriber(strcat('/trailer_angle_1'),'std_msgs/Float64');
    end
end

