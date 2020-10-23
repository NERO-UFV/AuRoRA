%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:



function rGetPotentiometerData(obj)


if obj.pFlag.Connected
    if obj.pFlag.Trailer == 2
        obj.pPot1 = receive(obj.subPot1);
        obj.pPot2 = receive(obj.subPot2);
    elseif obj.pFlag.Trailer == 1
        obj.pPot1 = receive(obj.subPot1);
    end
else
    % Simulation    -----------------------------------------------------------------
    obj.pPos.X = obj.pPos.X;
end
end