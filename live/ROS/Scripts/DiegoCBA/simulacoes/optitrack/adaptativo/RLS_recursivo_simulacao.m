function [Theta_est,Y1,Y2,T1,T2] = RLS_recursivo_simulacao(parametros, G1, G2, V, k,Theta1,Theta2,Yfp1,Yfp2,Tfp1,Tfp2)

   u_v = V(1);
   u_w = V(2);  
         
Theta1.atualiza(u_v,G1);    
Theta2.atualiza(u_w,G2);
     
   Yfp1 = vertcat(Yfp1, u_v);
   Yfp2 = vertcat(Yfp2, u_w);
   Tfp1 = vertcat(Tfp1, G1);    
   Tfp2 = vertcat(Tfp2, G2);       

theta1 = inv(Tfp1'*Tfp1)*Tfp1'*Yfp1;
theta_1 = theta1(1);
theta_3 = theta1(2);
theta_4 = theta1(3);
%
theta2 = inv(Tfp2'*Tfp2)*Tfp2'*Yfp2;
theta_2 = theta2(1);
theta_5 = theta2(2);
theta_6 = theta2(3);

Theta_est = [theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];

   Y1 = Yfp1; 
   Y2 = Yfp2; 
   T1 = Tfp1;      
   T2 = Tfp2;  

end