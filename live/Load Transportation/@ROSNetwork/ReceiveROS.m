function ReceivedMsg = ReceiveROS(obj,Topic)
    number = str2double(Topic(6));
    
    if strcmp(Topic(7:end),'/chatter')
        obj.SubMsgchatter{number} = obj.Subchatter{number}.LatestMessage;
        ReceivedMsg = obj.SubMsgchatter{number}.Data;
    end
    
    if strcmp(Topic(7:end),'/pose')
        obj.SubMsgpose{number} = obj.Subpose{number}.LatestMessage;
        ReceivedMsg = [obj.SubMsgpose{number}.X obj.SubMsgpose{number}.Y obj.SubMsgpose{number}.Z];
    end
    
    if strcmp(Topic(7:end),'/vel')
        obj.SubMsgvel{number} = obj.Subvel{number}.LatestMessage;
        ReceivedMsg = [obj.SubMsgvel{number}.Linear.X obj.SubMsgvel{number}.Angular.Z];
    end
    
    if strcmp(Topic(7:end),'/laser')
        obj.SubMsglaser{number} = obj.Sublaser{number}.LatestMessage;
        ReceivedMsg = obj.SubMsglaser{number}.Ranges;
    end
    
end