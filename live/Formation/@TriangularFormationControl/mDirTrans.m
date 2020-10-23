function mDirTrans(obj,Robo)
    xf = (Robo{obj.pID}.pPos.X(1)+Robo{obj.pID+1}.pPos.X(1)+Robo{obj.pID+2}.pPos.X(1))/3;   
    yf = (Robo{obj.pID}.pPos.X(2)+Robo{obj.pID+1}.pPos.X(2)+Robo{obj.pID+2}.pPos.X(2))/3;   
    pf = sqrt((Robo{obj.pID}.pPos.X(1)-Robo{obj.pID+1}.pPos.X(1))^2+(Robo{obj.pID}.pPos.X(2)-Robo{obj.pID+1}.pPos.X(2))^2);
    qf = sqrt((Robo{obj.pID}.pPos.X(1)-Robo{obj.pID+2}.pPos.X(1))^2+(Robo{obj.pID}.pPos.X(2)-Robo{obj.pID+2}.pPos.X(2))^2);
    
    rf = sqrt((Robo{obj.pID+1}.pPos.X(1)-Robo{obj.pID+2}.pPos.X(1))^2+(Robo{obj.pID+1}.pPos.X(2)-Robo{obj.pID+2}.pPos.X(2))^2);
    beta = acos((pf^2+qf^2-rf^2)/(2*pf*qf));
    psi = atan2((Robo{obj.pID}.pPos.X(2)-yf),(Robo{obj.pID}.pPos.X(1)-xf));
    
    obj.pPos.Q(1:6) = [xf yf psi pf qf beta]';
end