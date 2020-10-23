function obj = rGetMarker(obj)


    
  
    obj.Marker = obj.subMarker.LatestMessage;
    
    if isempty(obj.subMarker.LatestMessage) == 0

    % Convert quaternion to Euler angles in radians
    quat = [obj.Marker.Orientation.W
            obj.Marker.Orientation.X
            obj.Marker.Orientation.Y
            obj.Marker.Orientation.Z];

        
    eulXYZ = quat2eul(quat','XYZ');
    
    obj.pPos.X(1)  = -obj.Marker.Position.Z;   
    obj.pPos.X(2)  = obj.Marker.Position.X;   
    obj.pPos.X(3)  = obj.Marker.Position.Y;   
    obj.pPos.X(4)  = eulXYZ(1); % Roll   
    obj.pPos.X(5)  = eulXYZ(2); % Pitch
    obj.pPos.X(6)  = eulXYZ(3); % Yaw
    
    end

   

end
