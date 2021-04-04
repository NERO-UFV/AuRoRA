%% This function gets information about Position and Velocity
function vGetSensorData(vrep,p3dx,Index)
    %% Pioneer Position
    [erro,coordinateXYZ]= vrep.RemApi.simxGetObjectPosition(vrep.clientID,vrep.Pioneer(1,Index),-1,vrep.RemApi.simx_opmode_buffer);

    p3dx.pPos.Xc([1 2]) = coordinateXYZ([1 2])';
    
    %% Pioneer Orientation (alpha, beta e gama)
    [erro,orientation] = vrep.RemApi.simxGetObjectOrientation(vrep.clientID,vrep.Pioneer(1,Index),-1,vrep.RemApi.simx_opmode_buffer);
     
    % Linear Transform to find the Control Point
    p3dx.pPos.X([1 2])= p3dx.pPos.Xc([1 2]) ...
                         +[p3dx.pPar.a*cos(orientation(3));...
                           p3dx.pPar.a*sin(orientation(3));]; 

    

    %% Pioneer Linear and Angular Velocity
    [erro,LinearVelocity, AngularVelocity]= vrep.RemApi.simxGetObjectVelocity(vrep.clientID,vrep.Pioneer(1,Index),vrep.RemApi.simx_opmode_buffer);

    theta = atan2(LinearVelocity(2),LinearVelocity(1));
    psi = orientation(3);
    
    p3dx.pPos.Xc(6) = psi;
    p3dx.pPos.X(6) = psi;

    if theta*psi <0
        p3dx.pPos.X(7) = -sqrt(LinearVelocity(1)^2+LinearVelocity(2)^2 +LinearVelocity(3)^2);
    else
        p3dx.pPos.X(7) = sqrt(LinearVelocity(1)^2+LinearVelocity(2)^2 +LinearVelocity(3)^2);
    end
    
    p3dx.pPos.X(12) = AngularVelocity(3);
end