function dist = distanciaManhattan(S,Sg)
%distanciaManhattan calcula a distancia manhattan
dist = abs(S(2) - Sg(2)) + abs(S(3) - Sg(3));
dist = sqrt((S(2) - Sg(2))^2 + (S(3) - Sg(3))^2);
try
    G = evalin('base','Dist');
    dist = round(dist/G);
end
end