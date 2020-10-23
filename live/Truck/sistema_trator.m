%% Rascunho do projeto Pioneer + carretinha. Retirado do Artigo: "Robust Adaptive Controller for a Tractor–Trailer"
% Thiago Ridogério, 13/03/2020

%% Limpeza de variavéis 
    clc
    close all
    clearvars
 
%% Planejador Offline
    %Carregando as caracteristicas do pioneer e do reboque (classe)
     P = Pioneer3DX;
     R = Pioneer3DX;
     encaixe= zeros[12,1];
     
    %Definindo os parametros iniciais do robô
     P.pPar.a= 0;
     P.pPar.alpha=0;
     pgains=[1,1,1];
     
     %Definindo a trajetória do robô:
        % Circuferência:
%            V_max= .5;
            rx=1;
            ry=rx;
            w=0;
        
        Xd = [rx*cos(w*0);
              ry*sin(w*0);
              0];
          
        dXd = [-w*rx*sin(w*0);
               w*ry*cos(w*0);
              0];
    
        P.pPos.Xd([1:3 6]) = [Xd; atan2(dXd(2),dXd(1))];
        
        % Definindo a posição inicial do pioneer
         P.rSetPose(P.pPos.Xd([1:3 6]));


          
           

    