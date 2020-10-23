function [parametros, G1, G2, G3, G4 ,prc, PSI] = CalculaParametros(Accel, V, psi, Ud)
 % Parametros Estimando
            ddx = Accel(1);
            ddy = Accel(2);
            ddz = Accel(3);
            ddpsi = Accel(4);
            
            dx = V(1);
            dy = V(2);
            dz = V(3);
            dpsi = V(4);
            
            G1 = [cos(psi)*ddx+sin(psi)*ddy     cos(psi)*dx+sin(psi)*dy];                       
            G2 = [-sin(psi)*ddx+cos(psi)*ddy    -sin(psi)*dx+cos(psi)*dy];            
            G3 = [ddz   dz];            
            G4 = [ddpsi     dpsi];           
            
            PSI = [cos(psi)*ddx+sin(psi)*ddy    +cos(psi)*dx+sin(psi)*dy     0                           0                           0       0    0       0;
                   0                            0                           -sin(psi)*ddx+cos(psi)*ddy  -sin(psi)*dx+cos(psi)*dy     0       0    0       0;
                   0                            0                            0                           0                           ddz     dz   0       0;
                   0                            0                            0                           0                           0       0    ddpsi   dpsi];
               
            %DM.parametros{DM.tmrCount} = inv(PSI'*PSI)*PSI'*Ud;            
 %           if(det(PSI'*PSI) == 0)
%                parametros = zeros(8,1);
            %else
            
            %end            
            prc  = pinv(PSI)*Ud;
            %parametros = pinv(PSI)*Ud;
            %theta1=  inv(G1'*G1)*G1'*Ud(1);   
            %theta2=  inv(G2'*G2)*G2'*Ud(2);   
            %theta3=  inv(G3'*G3)*G3'*Ud(3);   
            %theta4=  inv(G4'*G4)*G4'*Ud(4);   
            
            theta1=  pinv(G1)*Ud(1);   
            theta2=  pinv(G2)*Ud(2);   
            theta3=  pinv(G3)*Ud(3);   
            theta4=  pinv(G4)*Ud(4);   
            
            parametros = vertcat(theta1, theta2, theta3, theta4);            
            %G = vertcat(G1,G2,G3,G4);
            %parametros
            %G
            %pause(2)
end