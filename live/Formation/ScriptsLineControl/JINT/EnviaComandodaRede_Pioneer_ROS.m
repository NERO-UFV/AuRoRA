
rosinit('192.168.0.144', 'NodeHost','192.168.0.130','NodeName','/DELL_SERVER');
masterHost = '192.168.0.144';

P1 = robotics.ros.Node('/P1', masterHost);

P1_X = robotics.ros.Subscriber(P1,'/P1/X','geometry_msgs/Twist');
P1_U = robotics.ros.Subscriber(P1,'/P1/U','geometry_msgs/Twist');
P1_Ud = robotics.ros.Publisher(P1,'/P1/Ud','geometry_msgs/Twist');

t = tic;
tcontrol = tic;
while toc(t) < 10
    if toc(tcontrol) > 0.030
        tcontrol = tic;
        msg = rosmessage(P1_Ud);
        msg.Linear.X = 0;
        msg.Angular.Z = .5;
        send(P1_Ud,msg);
        
        try
            msg_U = P1_U.LatestMessage;
            A = msg_U.Linear.X;
            AA = msg_U.Angular.Z;           
        catch
        end
        disp(A);
        disp(AA);

    end
end
msg = rosmessage(P1_Ud);
msg.Linear.X = 0;
msg.Angular.Z = 0;
send(P1_Ud,msg);




msg = rosmessage(P1_Ud);
msg.Linear.X = 10;
% msgLinear.Y = 0;
% msg.Linear.Z = 0;
% msg.Angular.X = 0;
% msg.Angular.Y = 0;
msg.Angular.Z = 20;
send(P1_Ud,msg);

% Create a publisher and subscriber for the '/pose' topic
twistPub = robotics.ros.Publisher(node_1,'/P1/pose','geometry_msgs/Twist');
twistPubmsg = rosmessage(twistPub);
twistSub = robotics.ros.Subscriber(node_2,'/pose');