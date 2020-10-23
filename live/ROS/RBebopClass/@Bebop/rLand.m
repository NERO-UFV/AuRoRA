%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:

function rLand(obj)
 
 send(obj.pubBrLand,obj.pStdMsgEmpty);
 obj.pFlag.EmergencyStop = 1;

end

