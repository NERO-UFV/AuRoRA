%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:


function output = rConnect(obj)

try
    if (obj.RI.pCom == false)
        obj.RI.rConnect;
        output = true;
    end
catch
        obj.RI.rDisconnect;
        output = false;       
end


end

