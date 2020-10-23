
%% Start ROS Network
r = ROSNetwork;

%% Creat ROS Master
% InitRos('MasterName')
% r.InitROS('/master')

%% Creat ROS Node
% InitRos('NodeName','MasterIPaddress')
r.InitROS('/robot2','192.168.0.158')

%% Publisher or Subscriber 
%r.SubscriberROS(r.node,'robot1/chatter');
%r.SubscriberROS(r.node,'robot2/chatter');
%r.SubscriberROS(r.node,'robot1/laser');

r.SubscriberROS(r.node,'robot1/vel');
P.pSC.Ud = r.ReceiveROS('robot1/vel')';

%r.ReceiveROS('robot1/laser')

%r.ReceiveROS('robot1/chatter')

%r.ReceiveROS('robot2/chatter')

%r.PublisherROS(r.node,'robot1/pose');

%Msg = [0 0 0];
%r.SendROS('robot1/pose',Msg);
