function Nomes = attNomes(Nomes,Elemento)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
pos = find(Elemento==Nomes');
Nomes = Nomes';
Nomes(pos) = 0;
for i=pos:(size(Nomes,1)*size(Nomes,2))
    if Nomes(i) ~= 0
        Nomes(i) = Nomes(i) - 1;
    end
end
Nomes = Nomes';
end

