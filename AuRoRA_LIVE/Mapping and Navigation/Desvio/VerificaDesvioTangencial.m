function [ Xv,gamma ] = VerificaDesvioTangencial( gamma,fe,dobs,dmin,beta,X,Xd)
%VERIFICADESVIOTANGENCIAL Summary of this function goes here
%   Detailed explanation goes here
    Xv = Xd;    
    %if pos ~= ult_pos
    %    gamma = 0;
    %end
    %ult_pos = pos;
    if dmin < dobs
        psi = X(6);
        theta = atan2(Xd(2) - X(2),Xd(1) -X(1));            
        alpha = theta - psi;            
        if beta < 0 % Obst�culo � direita
            gammac = -pi/2 + alpha - beta;
        else
            gammac =  pi/2 + alpha - beta;
        end            
        gamma = gamma*(1-fe) + fe*gammac; 
        %gamma = gammac;
        % Dist�ncia entre o rob� e o destino
        d = norm(Xd(1:2) - X(1:2)); 
        if d < 0.5
            d = 1;
        end
        Xv(1:2) = X(1:2) + [cos(theta-gamma); sin(theta-gamma)]*(2*tanh(d));  
    else        
        gamma = gamma*(1-fe);
    end
    
end

