function [P,tmax,Pilot] = pSafetyControl(P,J,Pilot,tmax)
%pSafetyControl Ativa controle de segurança pelo joystick.
%   Sobrescreve resultados do controlador usado originalmente, substiutindo
%   pelo controlador cinemático estendido. 

    J.mRead;
    gamma = 0.01;
if(J.pSNES==0)
    k1 = 0.07;
else
    k1 = 0.15;
%     BREAKER = J.pDigital(7:8);
end
    if (J.pDigital(end-1))
        disp('Code interrupted!')
        tmax = 0;
        Pilot = 0;
    end

    if(abs(J.pAnalog(1))||abs(J.pAnalog(2)))
        if (Pilot == 1)
            disp('AutoPilot is OFF!')
            Pilot = 2;
        end    
    end

    if (J.pDigital(6)||Pilot==1)
        if (Pilot ~= 1)
            disp('AutoPilot is ON!')
            Pilot = 1;
        elseif(J.pDigital(5))
            disp('SelfPilot is ON!')
            Pilot = 2;
            P.pPos.Xd(7:8) = 0;
        end

        P.pPos.Xd(7:8) = 0;
        P.pPos.Xd(1:2) = P.pPos.Xd(1:2) +[J.pDigital(2),J.pDigital(1)]'.*k1 ...
                                        -[J.pDigital(4),J.pDigital(3)]'.*k1; 
    elseif (J.pDigital(5)||Pilot==2)
        if (Pilot ~= 2)
            disp('SelfPilot is ON!')
            Pilot = 2;
            P.pPos.Xd(7:12) = 0;
            P.pPos.Xd([1,2,6]) = P.pPos.X([1,2,6]);
        elseif(J.pDigital(6))
             disp('AutoPilot is ON!')
            Pilot = 1;
        end
        
        P.pPos.Xd(7:8) = P.pPos.Xd(7:8) +[J.pAnalog(1),-J.pAnalog(2)]'.*k1;

%         disp(P.pPos.Xd)
        
        if (norm(P.pPos.Xd(7:8))>0 && gamma>0)
            P.pPos.Xd(7:8) = P.pPos.Xd(7:8) -sign(P.pPos.Xd(7:8)).*gamma;
        end

        P.pPos.Xd(1:2) = P.pPos.X(1:2);
        else
            %Controller is on!
    end

    % Pegando os dados do robo
    P = fKinematicControllerExtended(P);
end

