function mShutdown(obj)
%MLERDADOSSONAR Summary of this function goes here
%   Detailed explanation goes here
    try
        obj.mDesconectar;
        rosshutdown;
        stopCore(obj.dispositivo);
        %clear 
    catch err
        fprintf('Erro ao se desconectar no servidor ROS\n');
        disp(err.message);
    end
end

