function mInitROS(obj,NodeName,ip)
% Start ROS Master or Node on Matlab 
% Detailed explanation goes here
    switch nargin
        
        case 2
            %rosinit(obj.master);
            obj.master = robotics.ros.Core;
            rosinit(obj.master.MasterURI,'NodeName',NodeName)
            
            address = java.net.InetAddress.getLocalHost;
            IPaddress = char(address.getHostAddress);
            A = sscanf(IPaddress,'%d.%d.%d.%d;');
            fprintf('Master IP address is: %s \n',IPaddress)
            
        case 3
            obj.node = robotics.ros.Node(NodeName,ip);
            fprintf('Node %s is running and connected to the Master. \n',NodeName)
    end
        
end

