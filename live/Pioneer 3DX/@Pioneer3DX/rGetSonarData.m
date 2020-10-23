function SonarData = rGetSonarData(p3dx)

if p3dx.pFlag.Connected    
    % --------------------------------------------------------------------
    % MobileSim or Real P3DX
    SonarData(1,:) = [90 50 30 10 -10 -30 -50 -90]*pi/180;   
    for ii = 0:7
        SonarData(2,ii+1) = arrobot_getsonarrange(ii)/1000;
    end    
end