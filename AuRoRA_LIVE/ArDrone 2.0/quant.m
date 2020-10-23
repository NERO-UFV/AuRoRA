
% This function generates quantization matrix
% given rows (i), columns (jj) and quantization coeficient q

function Q = quant(ii,jj,nq)

% nq = 15;

for i = 1:ii
    for j = 1:jj

Q(i,j) = (1+(1+i+j)*nq);


    end
end