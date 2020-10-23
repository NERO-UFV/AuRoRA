function SendROS(obj,Topic,Msg)
    number = str2double(Topic(6));
    
    if strcmp(Topic(7:end),'/chatter')
        obj.Msgchatter{number}.Data = Msg;
        send(obj.Pubchatter{number},obj.Msgchatter{number});
    end
    
    if strcmp(Topic(7:end),'/pose')
        obj.Msgpose{number}.X = Msg(1);
        obj.Msgpose{number}.Y = Msg(2);
        obj.Msgpose{number}.Z = Msg(3);
        send(obj.Pubpose{number},obj.Msgpose{number});
    end
    
    
    if strcmp(Topic(7:end),'/vel')
        obj.Msgvel{number}.Linear.X = Msg(1);
        obj.Msgvel{number}.Linear.Y = 0;
        obj.Msgvel{number}.Linear.Z = 0;
        obj.Msgvel{number}.Angular.X = 0;
        obj.Msgvel{number}.Angular.Y = 0;
        obj.Msgvel{number}.Angular.Z = Msg(2);
        send(obj.Pubvel{number},obj.Msgvel{number});
    end
   
    if strcmp(Topic(7:end),'/laser')
        obj.Msglaser{number}.Ranges = Msg;
        send(obj.Publaser{number},obj.Msglaser{number});
    end

end