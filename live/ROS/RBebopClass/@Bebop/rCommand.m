%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:

function rCommand(obj)

%     B.pSC.Ud(1) % Frente/Tras [-1,1] (+) Avança, Move frente para baixo
%     B.pSC.Ud(2) % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
%     B.pSC.Ud(3) % Velocidade Vertical [-1,1] (+) Eleva o drone

%     # Regra da Mão Direita
%     B.pSC.Ud(4) % Angulo do drone [-1,1] (+) rotaciona para  em torno do Eixo 
%     B.pSC.Ud(5) % Angulo do drone [-1,1] (+) rotaciona para  em torno do Eixo 
%     B.pSC.Ud(6) % Angulo do drone [-1,1] (+) rotaciona para Positivo em torno do Eixo Z 

    % Linear Variable
    obj.pVel.Linear.X = obj.pSC.Ud(1);
    obj.pVel.Linear.Y = obj.pSC.Ud(2);
    obj.pVel.Linear.Z = obj.pSC.Ud(3);
    
    % Angular Variable
    obj.pVel.Angular.X = obj.pSC.Ud(4);
    obj.pVel.Angular.Y = obj.pSC.Ud(5);
    obj.pVel.Angular.Z = obj.pSC.Ud(6);   

    send(obj.pubCmdVel,obj.pVel);


end

