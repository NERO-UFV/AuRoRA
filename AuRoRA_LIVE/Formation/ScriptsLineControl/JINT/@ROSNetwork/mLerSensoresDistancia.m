function [ dados ] = mLerSensoresDistancia(obj)
%MLERDADOSSONAR Summary of this function goes here
%   Detailed explanation goes here
    dados = []; 
    %dados.scan = [];
    %dados.kinect = [];
    medidas(1,:) = [-90 -50 -30 -10 10 30 50 90];
    medidas(1,:) = medidas(1,:) * pi/180;
    medidas(2,:) = [0 0 0 0 0 0 0 0];
    try
        sonarData = receive(obj.sonar.sub);        
        for i=1:8
            %dados.pontos(i) = sonarData.Points(i);
            medidas(2,i) = sqrt(sonarData.Points(9-i).X^2 + sonarData.Points(9-i).Y^2);  
        end   
        if obj.temLaser            
            laserData = receive(obj.laser.sub);
            c = 1;
            aux = [];
            for i=1:size(laserData.Ranges,1)
                if ~isnan(laserData.Ranges(i,1))
                    aux(1,c) = (i-1)*laserData.AngleIncrement + laserData.AngleMin;
                    aux(2,c) = laserData.Ranges(i,1);%/1000;
                    c = c+1;
                end
            end
            if size(aux,2) > 0
                dados(1,:) = [medidas(1,1:3) aux(1,:) medidas(1,6:8)]; 
                dados(2,:) = [medidas(2,1:3) aux(2,:) medidas(2,6:8)];  
                %dados.kinect = aux;
            else
                dados = medidas;
            end
        end
        %dados = sonarData;
    catch
        %fprintf('oi2\n');        
    end
end

