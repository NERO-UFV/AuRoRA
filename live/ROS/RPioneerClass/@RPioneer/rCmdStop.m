%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:

function rCmdStop(obj)

%   Detailed explanation goes here
%     obj.pSC.Ud(1) % Frente/Tras [-1,1] (+) Avança, Move frente para baixo
%     obj.pSC.Ud(2) % Esquerda/Direita [-1,1] (+) Move rover para Esquerda                        
%     obj.pSC.Ud(3) % Velocidade Vertical [-1,1] (+) Eleva o rover

%     # Regra da Mão Direita
%     obj.pSC.Ud(4) % Angulo do rover [-1,1] (+) rotaciona para  em torno do Eixo 
%     obj.pSC.Ud(5) % Angulo do rover [-1,1] (+) rotaciona para  em torno do Eixo 
%     obj.pSC.Ud(6) % Angulo do rover [-1,1] (+) rotaciona para Positivo em torno do Eixo Z 

    % Linear Variable
    obj.pVel.Linear.X = 0.0;
    obj.pVel.Linear.Y = 0.0;
    obj.pVel.Linear.Z = 0.0;
    
    % Angular Variable
    obj.pVel.Angular.X = 0.0;
    obj.pVel.Angular.Y = 0.0;
    obj.pVel.Angular.Z = 0.0;

    send(obj.pubCmdVel,obj.pVel);

end

