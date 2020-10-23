function mDesconectar(obj)
%MLERDADOSSONAR Summary of this function goes here
%   Detailed explanation goes here
    try
        if size(fieldnames(obj.pose),1) > 0
            obj.pose = rmfield(obj.pose,fieldnames(obj.pose));
        end
        if size(fieldnames(obj.vel),1) > 0
            obj.vel = rmfield(obj.vel,fieldnames(obj.vel));
        end
        if obj.sensor == 'sonar'
            if size(fieldnames(obj.sonar),1) > 0
                obj.sonar = rmfield(obj.sonar,fieldnames(obj.sonar));
            end
        else
            if size(fieldnames(obj.laser),1) > 0
                obj.laser = rmfield(obj.laser,fieldnames(obj.laser));
            end
        end
        if isNodeRunning(obj.dispositivo,'rosaria')
            stopNode(obj.dispositivo,'rosaria');
        end
    catch err
        fprintf('Erro ao se desconectar do Nó Aria do ROS\n');
        disp(err.message);
    end
end

