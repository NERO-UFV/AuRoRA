function mConectarNodos(obj)
%MLERDADOSSONAR Summary of this function goes here
%   Detailed explanation goes here
    %sim = true;
    try
        if ~obj.mProcuraNodo('/RosAria')
            runNode(obj.dispositivo,'rosaria');
        end
        if ~obj.mProcuraNodo('/nodePose')
            obj.pose.node   = robotics.ros.Node('nodePose', obj.master);
            obj.pose.sub    = robotics.ros.Subscriber(obj.pose.node,'/RosAria/pose');
        end
        if ~obj.mProcuraNodo('/nodeVel')
            obj.vel.node    = robotics.ros.Node('nodeVel', obj.master);
            obj.vel.pub     = robotics.ros.Publisher(obj.vel.node,'/RosAria/cmd_vel','geometry_msgs/Twist');
        end
        if ~obj.mProcuraNodo('/nodeSonar')
            obj.sonar.node  = robotics.ros.Node('nodeSonar',obj.master);
            obj.sonar.sub   = robotics.ros.Subscriber(obj.sonar.node,'/RosAria/sonar','sensor_msgs/PointCloud');
        end
        obj.scan = false;
        if obj.mProcuraNodo('/depthimage_to_laserscan')
            if ~obj.mProcuraNodo('/nodeScan')
                obj.laser.node  = robotics.ros.Node('nodeScan',obj.master);
            end
            obj.laser.sub   = robotics.ros.Subscriber(obj.laser.node,'/kinect_scan','sensor_msgs/LaserScan');
            obj.scan = true;
        elseif obj.mProcuraNodo('/scan')
            if ~obj.mProcuraNodo('/nodeScan')
                obj.laser.node  = robotics.ros.Node('nodeScan',obj.master);
            end
            obj.laser.sub   = robotics.ros.Subscriber(obj.laser.node,'/scan','sensor_msgs/LaserScan');
            obj.scan = true;
        end
        if obj.mProcuraNodo('/rtabmap/rtabmap')
            if ~obj.mProcuraNodo('/nodeMapa')
                obj.mapa.node  = robotics.ros.Node('nodeMapa',obj.master);
            end
            obj.mapa.sub = robotics.ros.Subscriber(obj.mapa.node,'/rtabmap/grid_map','nav_msgs/OccupancyGrid');
            obj.mapa.grid2d = [];
            obj.mapa.grid2d.og = readBinaryOccupancyGrid(receive(obj.mapa.sub));
            obj.mapa.grid2d.mod = [];
        end        
    catch err
        disp(err.message);
    end
end

