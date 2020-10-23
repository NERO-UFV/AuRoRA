function [] = rSaveData(filename,Hist, IAE, ITAE, IASC, idAtraso)
%****** Função para salvar os dados de cada experimento
save(filename, 'Hist', 'IAE', 'ITAE', 'IASC', 'idAtraso');

% A = ['Hist' datestr(now,30) '.mat'];
% save(A,'Hist');

% whos('-file','test.mat')

end
