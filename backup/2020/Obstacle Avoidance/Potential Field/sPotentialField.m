function ForceDrObTot = sPotentialField(obstacle,drone,Par)
% drone = R; obj = A;
% ForceDrObT = [0 0 0];
dmin = Par.Dist(1);
dstart = Par.Dist(2);

fmin = Par.Forc(1);
fmax = Par.Forc(2);

%  idx = 3; obstacle(idx) = O(idx); drone = A; Par.Forc = [0.00 .005]; Par.Dist = [0.75 1.5];
idx = 1;
ForceDrOb(1:2) = [0 0];
ForceDrObTot(1:2) = [0 0];
%%
% for idx = size(obstacle,2)
%% Distância entre Drone e objeto do ambientes
DistDrOb(1:3) =  [obstacle(idx).pPos.X(1)-drone.pPos.X(1)...
                  obstacle(idx).pPos.X(2)-drone.pPos.X(2)...
                  pdist([obstacle(idx).pPos.X(1:2)';drone.pPos.X(1:2)'])]; 
    
    v  = (pdist([obstacle(idx).pPos.X(1:2)';drone.pPos.X(1:2)']) - pdist([obstacle(idx).pPos.Xa(1:2)';drone.pPos.Xa(1:2)']))/drone.pPar.Ts;
    dx = (pdist([obstacle(idx).pPos.X(1)';drone.pPos.X(1)']) - pdist([obstacle(idx).pPos.Xa(1)';drone.pPos.Xa(1)']))/drone.pPar.Ts;
    vx = drone.pPos.X(7);
    dy = (pdist([obstacle(idx).pPos.X(2)';drone.pPos.X(2)']) - pdist([obstacle(idx).pPos.Xa(2)';drone.pPos.Xa(2)']))/drone.pPar.Ts;
    vy = drone.pPos.X(8);
    phi = atan2(DistDrOb(2),DistDrOb(1));
    theta = atan2(drone.pPos.Xd(2)-drone.pPos.X(2),drone.pPos.Xd(1)-drone.pPos.X(1));
    psi = atan2(DistDrOb(2)-drone.pPos.X(2),DistDrOb(1)-drone.pPos.X(1));
%     disp(psi)
%     if abs(v) < 0.01
%         v = 0;
%     end
%     if abs(dx) < 0.01
%         dx = 0;
%     end
%     if abs(dy) < 0.01
%         dy = 0;
%     end
%      disp([DistDrOb theta])
%%
if DistDrOb(3) < dstart
    
     ForceDrOb(2) =  abs(cos(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
    if DistDrOb(1) < dstart
         ForceDrOb(1) =  abs(sin(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin))-10\sign(vx)*abs(cos(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
         ForceDrOb(1) =  -sign(DistDrOb(1))*abs(sin(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin))-(-3\sign(vx)*abs(cos(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin))+2\sign(vx)*abs(cos(psi))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin)));
         ForceDrOb(1) =  -sign(DistDrOb(1))*abs(sin(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));

    end
    if DistDrOb(2) < dstart
%         ForceDrOb(2) =  abs(cos(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin))-10\sign(vy)*abs(sin(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
%         ForceDrOb(2) =  -sign(DistDrOb(2))*abs(cos(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin))-2\sign(vy)*abs(sin(theta))*fmax*tanh((dstart-DistDrOb(3))/(dstart-dmin));
    end

elseif DistDrOb(3) < dmin && v > 0
%     ForceDrOb(1) = -sign(vx)*sin(phi)*fmax*tanh(dmin/DistDrOb(3))/5;
%     ForceDrOb(2) = -sign(vx)*cos(phi)*fmax*tanh(dmin/DistDrOb(3))/5;
end
    ForceDrObTot = ForceDrOb;
    obstacle(idx).pPos.Xa = obstacle(idx).pPos.X;

end % Function


    
%     if v <= 0
%         disp('asd')
%         if vx < 0 && DistDrOb(3) < dmin
%             ForceDrOb(1) = sin(atan2(vy,vx))*DistDrOb(1)/DistDrOb(3)*(fmax/2-fmin)/(dstart-dmin)*(dstart-DistDrOb(3))+fmin;
%         else
%             ForceDrOb(1) = 0;
%         end
%         if vy < 0 && DistDrOb(3) < dmin
%             ForceDrOb(2) = cos(atan2(vy,vx))*DistDrOb(2)/DistDrOb(3)*(fmax-fmin)/(dstart-dmin)*(dstart-DistDrOb(3))+fmin;
%         else
%             ForceDrOb(2) = 0;
%         end
%     else
%         ForceDrOb(1:2) = [0 0];
%         disp('bb')
%     end

%%


% if v < 0 && DistDrOb(3) < dstart
%     ForceDrOb(2) = fmax;
%     disp('Força Máxima em Y')
% elseif   v < 0 && DistDrOb(3) > dstart
%     ForceDrOb(2) = DistDrOb(2)/DistDrOb(3)*(fmax-fmin)/(dstart-dmin)*(dstart-DistDrOb(3))+fmin;
%     disp('forca em Y')
% else
%     ForceDrOb(2) = 0;
% end


%% Calcula a força de repulsão em x
% if v < 0 && DistDrOb(3) < dstart
%     ForceDrOb(1) = fmax;
%     disp('Força Máxima em Y')
% elseif   v < 0 && DistDrOb(3) > dmin
%     ForceDrOb(1) = DistDrOb(1)/DistDrOb(3)*(fmax/2-fmin)/(dstart-dmin)*(dstart-DistDrOb(3))+fmin;
%     disp('forca em X')
% else
%     ForceDrOb(1) = 0;
% end
%% Calcula a força de repulsão em y
% 
% if v < 0 && DistDrOb(3) < dstart
%     ForceDrOb(2) = fmax;
%     disp('Força Máxima em Y')
% elseif   v < 0 && DistDrOb(3) > dstart
%     ForceDrOb(2) = DistDrOb(2)/DistDrOb(3)*(fmax-fmin)/(dstart-dmin)*(dstart-DistDrOb(3))+fmin;
%     disp('forca em Y')
% else
%     ForceDrOb(2) = 0;
% end
%                   
%                   
%                   
% %% Calcula a força de repulsão em x
%     if DistDrOb(3) > dstart
%         ForceDrOb(1) = fmin;
%     elseif DistDrOb(3) > abs(dmin)
%         if DistDrOb(1) > 0
% %           obj.pForc.RepField(1) = (obj.pForc.distDrone(1)/obj.pForc.distDrone(3)*(obj.pForc.RepMax-obj.pForc.RepMin)*(obj.pForc.distmax-obj.pForc.distDrone(3))/(obj.pForc.distmax-obj.pForc.distmin));
%         ForceDrOb(1) = DistDrOb(1)/DistDrOb(3)*(fmax/2-fmin)/...
%                                 (dstart-dmin)*(dstart-DistDrOb(3))+fmin;
%         end
%     else
%         ForceDrOb(1) = fmax;
%         disp('Força Máxima em X')
%         disp(ForceDrOb)
%     end
%     
% %% Calcula a força de repulsão em y
%     if DistDrOb(3) > dstart
%         ForceDrOb(2) = fmin;
%     elseif DistDrOb(3) > dmin
%         if dstart > 0
% %         obj.pForc.RepField(1) = obj.pForc.RepMax*exp(-obj.pForc.distDrone(2)/obj.pForc.distDrone(3));
%         ForceDrOb(2) = DistDrOb(2)/DistDrOb(3)*(fmax-fmin)/(dstart-dmin)*(dstart-DistDrOb(3))+fmin;
%         end
%     else
%         ForceDrOb(2) = fmax;
%         disp('Força Máxima em Y')
%         disp(ForceDrOb)
%     end
%     
%     ForceDrOb(2) = 0;
    
%     disp(obj.pForc.RepFieldT)
%     disp(obj.pForc.distDrone)
