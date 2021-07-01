%% Get Data from Laser Scanner
function [Map]= vGetLaserData(vrep,p3dx,Index)
    %% Get Data from Laser Scanner on V-REP
    [erro, data] = vrep.RemApi.simxGetStringSignal(vrep.clientID,'communication',vrep.RemApi.simx_opmode_buffer);
    [array] = vrep.RemApi.simxUnpackFloats(data);
    
    %% Allocating space to save the Map
    coordX = zeros(1,length(array)/3);
    coordY = zeros(1,length(array)/3);
    polar= zeros(1,length(array)/3);
    
    for i=0:length(array)
        try
            coordX(i+1) = array(3*i+2);
            coordY(i+1) = array(3*i+3);
            polar(i+1) = sqrt(coordX(i+1)^2+coordY(i+1)^2);
        catch 
%             disp('Erro armazenando dados do Laser!')
%             disp(erro)
        end
    end
    
    %% Get Orientation and Position
    vrep.vGetSensorData(p3dx,Index);
    theta= atan2(p3dx.pPos.X(2)-p3dx.pPos.Xc(2),p3dx.pPos.X(1)-p3dx.pPos.Xc(1));
    
    %% Adjusting the Map
    X0 = cos(theta)*coordX -sin(theta)*coordY+p3dx.pPos.X(1);
    Y0 = sin(theta)*coordX + cos(theta)*coordY+p3dx.pPos.X(2);

    Map = [X0' Y0' polar'];
end