
%% Start ROS Network
r = ROSNetwork;

%% Creat ROS Master
% InitROS('MasterName')
r.InitROS('/master')

%% Creat ROS Node
% InitROS('NodeName','MasterIPaddress')
r.InitROS('/robot1','192.168.0.158')

%% Publisher or Subscriber 
% PublisherROS(Node,Topic);
% Topics: robot1/chatter robot2/pose robot3/vel robot4/laser

% r.PublisherROS(r.node,'robot1/laser');
r.PublisherROS(r.node,'robot1/vel');

% r.PublisherROS(r.node,'robot1/chatter');

% Msg = 'Pioneer ';
% r.SendROS('robot1/chatter',Msg);

Msg = [0.4 0.3];
r.SendROS('robot1/vel',Msg);

% Msg = [0 0];
% r.SendROS('robot1/vel',Msg);

% Msg = [1 2 3 4 5 6 76 7 8 9 9];
% r.SendROS('robot1/laser',Msg);


% r.SubscriberROS(r.node,'robot1/pose');
% r.ReceiveROS('robot1/pose')
rosshutdown;
clear all; close all; clc;
