function PublisherROS(obj,Node,Topic)
    number = str2double(Topic(6));
    
    if strcmp(Topic(7:end),'/chatter')
        obj.Pubchatter{number} = robotics.ros.Publisher(Node,Topic,'std_msgs/String');
        obj.Msgchatter{number} = rosmessage(obj.Pubchatter{number});
    end
    
    
    if strcmp(Topic(7:end),'/pose')
        obj.Pubpose{number} = robotics.ros.Publisher(Node,Topic,'geometry_msgs/Point');
        obj.Msgpose{number} = rosmessage(obj.Pubpose{number});
    end
    
    
    if strcmp(Topic(7:end),'/vel')
        obj.Pubvel{number} = robotics.ros.Publisher(Node,Topic,'geometry_msgs/Twist');
        obj.Msgvel{number} = rosmessage(obj.Pubvel{number});
    end
    
    if strcmp(Topic(7:end),'/laser')
        obj.Publaser{number} = robotics.ros.Publisher(Node,Topic,'sensor_msgs/LaserScan');
        obj.Msglaser{number} = rosmessage(obj.Publaser{number});
    end
    
    
end