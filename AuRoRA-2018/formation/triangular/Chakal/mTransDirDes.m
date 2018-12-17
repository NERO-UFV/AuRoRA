function mTransDirDes(obj,Robo)

    xf = (Robo(1,1)+Robo(1,2)+Robo(1,3))/3;
    
    yf = (Robo(2,1)+Robo(2,2)+Robo(2,3))/3;
    
    pf = sqrt((Robo(1,1)-Robo(1,2))^2+(Robo(2,1)-Robo(2,2))^2);
    
    qf = sqrt((Robo(1,1)-Robo(1,3))^2+(Robo(2,1)-Robo(2,3))^2);
    
    rf = sqrt((Robo(1,2)-Robo(1,3))^2+(Robo(2,2)-Robo(2,3))^2);
    beta = acos((pf^2+qf^2-rf^2)/(2*pf*qf));
    
    psi = atan2((Robo(2,1)-yf),(Robo(1,1)-xf));
    
    obj.pPos.Xd(1:6) = [xf yf psi pf qf beta]';


end