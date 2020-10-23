%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:


function output = rDisconnect(obj)

    try
        if (obj.RI.pCom == true)
            obj.RI.rDisconnect;
            output = true;
        end
    catch
            output = false;       
    end

end

