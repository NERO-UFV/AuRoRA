        P.pPos.Xac = 0;
        P2.pPos.Xac = 0;

        P_real_X_hist
        P2_real_X_hist
        
        for i = 1:length(P2_real_X_hist) 

        if abs(P.pPos.Xc(6)-P.pPos.Xac) > pi
           P.pPos.Xc(6) = P.pPos.Xc(6) - 2*pi*sign(P.pPos.Xc(6));
           P.pPos.Xac = P.pPos.Xc(6);
        end
        
        if abs(P2.pPos.Xc(6)-P2.pPos.Xac) > pi
           P2.pPos.Xc(6) = P2.pPos.Xc(6) - 2*pi*sign(P2.pPos.Xc(6));
           P2.pPos.Xac = P2.pPos.Xc(6);
        end
        
        hold on
        plot(i,P2.pPos.Xc(6),'-')
        plot(i,P.pPos.Xc(6),'-')
        hold off
        
        drawnow
        
        end
        
       