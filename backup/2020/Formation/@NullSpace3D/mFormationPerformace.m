%    ***************************************************************
%    Course    : Master Science                              
%    Developer : Mauro Sergio Mafra Moreira                     
%    Date      : 06/02/2019 
%    Revision  : R00                               
%                                                                 
%    History   : R00 - Inicial Emission
%
%    ***************************************************************

% Description:

function mFormationPerformace(obj,sampleTime,time,index)

  % Position Errors
  xTil = obj.pPos.Qtil(1);
  yTil = obj.pPos.Qtil(2);
  zTil = obj.pPos.Qtil(3);

  % Formation Errors
  rTil = obj.pPos.Qtil(4);  
  bTil = obj.pPos.Qtil(5);  
  aTil = obj.pPos.Qtil(6);  
  
  % Calculate Individual Scores
  obj.IAEv = obj.IAEv + sampleTime.*(abs(obj.pPos.Qtil));        
  obj.ISEv = obj.ISEv + sampleTime.*(obj.pPos.Qtil.^2);    

  % Calculate Total Scores
  obj.IAEt = sum(obj.IAEv);
  obj.ISE  = sum(obj.ISEv)/length(obj.ISEv);
  
  % Indices Valentim conferir
  obj.IAE  = obj.IAE  + sampleTime*norm(obj.pPos.Qtil);
  obj.ITAE = obj.ITAE + time*sampleTime*norm(obj.pPos.Qtil);
   
  % Normalized Index 1-Posição  2-Formação
  obj.nFErr(1,index) = abs(obj.WFEc(1)*(abs(xTil) + abs(yTil) + abs(zTil)));
  obj.nFErr(2,index) = abs(obj.WFEc(2)*(abs(rTil))); 

   
end

