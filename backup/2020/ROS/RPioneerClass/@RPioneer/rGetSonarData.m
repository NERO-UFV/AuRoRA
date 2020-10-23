function dist = rGetSonarData(obj)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
sonar = receive(obj.subSonar);
sonar = double([sonar.Points.X; sonar.Points.Y]');
    % --------------------------------------------------------------------
    % MobileSim or Real P3DX
    SonarData(1,:) = [90 50 30 10 -10 -30 -50 -90]*pi/180;   
    for ii = 0:7
        SonarData(2,ii+1) = norm(sonar(ii+1,:));
    end   
end

