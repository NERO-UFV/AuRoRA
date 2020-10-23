%% CONTROLE DE VELOCIDADE

%% GOOD PRACTICES 
    clc;
    close all;
    clearvars;
%% CODE:
P.pSC.Ud=[0.001;0.2]; % P.pSC.Ud(1) é a velocidade linear e P.pSC.Ud(2) é a velocidade angular

if P.pSC.Ud(1)==0
    disp('Valor da velocidade linear inválido')
elseif P.pSC.Ud(2) > 0.1 && P.pSC.Ud(1)<0.075
    P.pSC.Ud(2)=[0.1];
    disp(P.pSC.Ud)
    
elseif P.pSC.Ud(2) > 0.3 && P.pSC.Ud(1)<0.15
    P.pSC.Ud(2)=[0.3];
    disp(P.pSC.Ud)
    
elseif P.pSC.Ud(2) > 0.4 && P.pSC.Ud(1)<0.225
    P.pSC.Ud(2)=[0.4];
    disp(P.pSC.Ud)
    
elseif P.pSC.Ud(2) > 0.6 && P.pSC.Ud(1)<0.3
    P.pSC.Ud(2)=[0.6];
    disp(P.pSC.Ud)
    
elseif P.pSC.Ud(2) > 0.7 && P.pSC.Ud(1)<0.375
    P.pSC.Ud(2)=[0.7];
    disp(P.pSC.Ud)
    
elseif P.pSC.Ud(2) > 0.9 && P.pSC.Ud(1)<0.45
    P.pSC.Ud(2)=[0.9];   
    disp(P.pSC.Ud)
    
elseif P.pSC.Ud(2)==0 && P.pSC.Ud(1)==0 
    disp('Velocidade nula. Entre pelo menos com uma velocidade linear valida.')
    
end
