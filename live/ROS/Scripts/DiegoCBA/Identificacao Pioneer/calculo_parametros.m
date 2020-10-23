function [parametros, G1, G2] = calculo_parametros(Accel, V, Ud)
 % Parametros Estimando           
            ddu = Accel(1);
            ddw = Accel(2);
            
            du = V(1);
            dw = V(2);
           
            G1 = [ddu -dw^2 du];
            G2 = [ddw du*dw dw];
            
            theta_est1 =  pinv(G1)*Ud(1);
            theta_est2 =  pinv(G2)*Ud(2);
            
            parametros = vertcat(theta_est1,theta_est2);            

end