function ForceDrObTot = sPotentialField_New(obstacle,drone,Par)
% drone = R; obj = A;
% ForceDrObT = [0 0 0];
dmin = Par.Dist(1);
dstart = Par.Dist(2);

fmin = Par.Forc(1);
fmax = Par.Forc(2);

%  idx = 3; obstacle(idx) = O(idx); drone = A; Par.Forc = [0.00 .005]; Par.Dist = [0.75 1.5];
% idx = 1;
ForceDrOb = [0 0]';
ForceDrObTot = [0 0]';
%%
%% Distância entre Drone e objeto do ambientes
for idx = 1:size(obstacle,2)
    
    DistDrOb =  [obstacle(idx).pPos.X(1)-drone.pPos.X(1)...
                 obstacle(idx).pPos.X(2)-drone.pPos.X(2)...
                 pdist([obstacle(idx).pPos.X(1:2)';drone.pPos.X(1:2)'])];
    DistXdOb = [obstacle(idx).pPos.X(1)-drone.pPos.Xd(1)...
        obstacle(idx).pPos.X(2)-drone.pPos.Xd(2)];
    
    phi   = abs(atan2(DistDrOb(2),DistDrOb(1)));
    theta = 0;
    if DistDrOb(3) < dstart
        if DistXdOb(1) < 0 && phi < 120*pi/180
            theta = atan2(drone.pPos.Xd(2)-drone.pPos.X(2),drone.pPos.Xd(1)-drone.pPos.X(1));
            ForceDrOb(1,idx) =  -fmax*.2;
            ForceDrOb(2,idx) =  1.0*cos(phi)*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
        end
%         if DistXdOb(1) < 0 && phi > 90*pi/180
%             theta = atan2(drone.pPos.Xd(2)-drone.pPos.X(2),drone.pPos.Xd(1)-drone.pPos.X(1));
%             ForceDrOb(1,idx) =  0;
%             ForceDrOb(2,idx) =  -cos(phi)*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
%         end
        if DistXdOb(1) > 0 && phi > 120*pi/180
            theta = atan2(drone.pPos.X(2)-drone.pPos.Xd(2),drone.pPos.X(1)-drone.pPos.Xd(1));
            ForceDrOb(1,idx) =  -fmax*.2;
            ForceDrOb(2,idx) =  1.0*cos(phi)*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
        end
%         if DistXdOb(1) > 0 && phi < 90*pi/180
%             theta = atan2(drone.pPos.X(2)-drone.pPos.Xd(2),drone.pPos.X(1)-drone.pPos.Xd(1));
%             ForceDrOb(1,idx) =  0;
%             ForceDrOb(2,idx) =  -cos(phi)*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
%         end
    end
    obstacle(idx).pPos.Xa = obstacle(idx).pPos.X;
    %
end
 ForceDrObTot = sum(ForceDrOb,2);
 
 if ForceDrObTot(1) > fmax
     ForceDrObTot(1) = 1.5*fsmax;
 end
  if ForceDrObTot(2) > fmax
     ForceDrObTot(2) = 1.5*fmax;
 end
 
 
% disp([DistDrOb(1:3) sign(DistXdOb(1)) phi*180/pi theta*180/pi psi*180/pi ForceDrObTot' ])





% if DistDrOb(3) < dstart
%     if DistXdOb(1) < 0 && phi < 90*pi/180
%         theta = atan2(drone.pPos.Xd(2)-drone.pPos.X(2),drone.pPos.Xd(1)-drone.pPos.X(1));
%         ForceDrOb(1) =  -0.004;
%         ForceDrOb(2) =  cos(theta)*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
%     end
%     if DistXdOb(1) > 0  && phi > 90*pi/180
%         theta = atan2(drone.pPos.X(2)-drone.pPos.Xd(2),drone.pPos.X(1)-drone.pPos.Xd(1));
%         ForceDrOb(1) =  0.004;
%         ForceDrOb(2) =  cos(theta)*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
%     end
%     ForceDrObTot = ForceDrOb;
%     obstacle(idx).pPos.Xa = obstacle(idx).pPos.X;
% end
%     disp([DistDrOb(1:3) sign(DistXdOb(1))  phi*180/pi theta*180/pi psi*180/pi ForceDrOb ])










end % Function

