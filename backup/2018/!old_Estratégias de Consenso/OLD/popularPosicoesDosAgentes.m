function X = popularPosicoesDosAgentes(X, Conexoes)


for ii = 1:length(Conexoes)
    [ID, POS, SC, flag] = receberDado(Conexoes(ii));
        if flag
            X(:,ID) = POS(1:3);
        end
    
end
