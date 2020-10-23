function SubscriberROS(obj,Node,Topic)
    number = str2double(Topic(6));
    
    if strcmp(Topic(7:end),'/chatter')
        obj.Subchatter{number} = robotics.ros.Subscriber(Node,Topic,'std_msgs/String');
        obj.SubMsgchatter{number} = obj.Subchatter{number}.LatestMessage;
    end
    
    
    if strcmp(Topic(7:end),'/pose')
        obj.Subpose{number} = robotics.ros.Subscriber(Node,Topic,'geometry_msgs/Point');
        obj.SubMsgpose{number} = obj.Subpose{number}.LatestMessage;
    end
    
    
    if strcmp(Topic(7:end),'/vel')
        obj.Subvel{number} = robotics.ros.Subscriber(Node,Topic,'geometry_msgs/Twist');
        obj.SubMsgvel{number} = obj.Subvel{number}.LatestMessage;
    end
    
    if strcmp(Topic(7:end),'/laser')
        obj.Sublaser{number} = robotics.ros.Subscriber(Node,Topic,'sensor_msgs/LaserScan');
        obj.SubMsglaser(number) = obj.Sublaser{number}.LatestMessage;
    end
    
    
end