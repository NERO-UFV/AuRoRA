function [P,tmax,AutoPilotON] = fSafetyControl(P,J,AutoPilotON,tmax)
%pSafetyControl Ativa controle de segurança pelo joystick.
%   Sobrescreve resultados do controlador usado originalmente, substiutindo
%   pelo controlador cinemático estendido. 
    if nargin==2
        AutoPilotON = 0;
    end
       
    J.mRead;
    gamma = 0.01;
    
    if (J.pDigital(end-1))
        tmax = 0;
        AutoPilotON = 0;
    end

    if(abs(J.pAnalog(1))||abs(J.pAnalog(2)))
        if (AutoPilotON == 1)
            disp('AutoPilot is OFF!')
            AutoPilotON = 0;
        end    
    end

    if (J.pDigital(end)||AutoPilotON)
        if (AutoPilotON == 0)
            disp('AutoPilot is ON!')
            AutoPilotON = 1;
        end
        P.pPos.Xd(7:8) = 0;
        P.pPos.Xd(1:2) = P.pPos.Xd(1:2) +[J.pDigital(2),J.pDigital(1)]'.*0.07 ...
                                        -[J.pDigital(4),J.pDigital(3)]'.*0.07; 
    else 
        P.pPos.Xd(7:8) = P.pPos.Xd(7:8) +[J.pAnalog(1),-J.pAnalog(2)]'.*0.07;

        if (norm(P.pPos.Xd(7:8))>0 && gamma>0)
            P.pPos.Xd(7:8) = P.pPos.Xd(7:8) -sign(P.pPos.Xd(7:8)).*gamma;
%             P.pPos.Xd(7:8) = P.pPos.Xd(7:8) -sign(P.pPos.Xd(7:8)).*gamma*toc(ts);
        end

        P.pPos.Xd(1:2) = P.pPos.X(1:2);
    end
    
    % Pegando os dados do robo
    P = fKinematicControllerExtended(P);
end

