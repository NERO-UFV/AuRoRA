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

function mDisplayFormationPerformace(obj,iTitle)
%MDISPLAYFORMATIONPERFORMACE Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    iTitle = " Position and Formation";  
    % disp('Gains not given. Using standard ones.');
end


% Performace Incices
disp(" ");
disp('-------------------------')
disp('Integral do Erro Absoluto')
disp('-------------------------')
disp('IAE')
disp(strcat(" IAExp: ",toStringJSON(obj.IAEv(1))));
disp(strcat(" IAEyp: ",toStringJSON(obj.IAEv(2))));
disp(strcat(" IAEzp: ",toStringJSON(obj.IAEv(3))));
disp(strcat(" IAErf: ",toStringJSON(obj.IAEv(4))));
disp(strcat(" IAEbf: ",toStringJSON(obj.IAEv(5))));
disp(strcat(" IAEaf: ",toStringJSON(obj.IAEv(6))));
disp(strcat(" IAEf : ",toStringJSON(obj.IAE)));
disp(" ");
disp(strcat(" IAE_Formation: ",toStringJSON(obj.IAEv(1)+obj.IAEv(2)+obj.IAEv(3))));
disp(strcat(" IAE_Position:  ",toStringJSON(obj.IAEv(4)+obj.IAEv(5)+obj.IAEv(6))));
disp(" ");
% disp(strcat(" ISEf : ",toStringJSON(obj.ISE)));
% disp(" ");
disp('IAE Total')
disp(obj.IAEt');
disp(" ");

disp(" ");
disp('----------------------------');
disp('Integral do Erro ao Quadrado');
disp('----------------------------');
disp('ISE');
disp(" ");
disp(strcat(" ISExp: ",toStringJSON(obj.ISEv(1))));
disp(strcat(" ISEyp: ",toStringJSON(obj.ISEv(2))));
disp(strcat(" ISEzp: ",toStringJSON(obj.ISEv(3))));
disp(strcat(" ISErf: ",toStringJSON(obj.ISEv(4))));
disp(strcat(" ISEbf: ",toStringJSON(obj.ISEv(5))));
disp(strcat(" ISEaf: ",toStringJSON(obj.ISEv(6))));
disp(" ");
disp(strcat(" ISE_Formation: ",toStringJSON(obj.ISEv(1)+obj.ISEv(2)+obj.ISEv(3))));
disp(strcat(" ISE_Position:  ",toStringJSON(obj.ISEv(4)+obj.ISEv(5)+obj.ISEv(6))));
disp(" ");
% disp(strcat(" ISE Medio : ",toStringJSON(obj.ISE)));
% disp(" ");
disp('ISE Médio')
disp(obj.ISE);
disp(" ");



figure()
xAux = 1:1:length(obj.nFErr(2,:));
hold on
    plot(xAux, obj.nFErr(1,:),'r',xAux, obj.nFErr(2,:),'b--');        
    title(strcat('Normalized Error - ',iTitle));
    legend('Position','Formation');
    axis([0 450 0 1.2])
    set(gcf, 'name', iTitle);
    grid on;
hold off


figure()
xAux = 1:1:length(obj.nFErr(2,:));
hold on
    plot(xAux, obj.nFErr(1,:),'r',xAux, obj.nFErr(2,:),'b--');        
    title(strcat('Normalized Error - ',iTitle));
    legend('Position','Formation');    
    set(gcf, 'name', iTitle);
    grid on;
hold off


end

