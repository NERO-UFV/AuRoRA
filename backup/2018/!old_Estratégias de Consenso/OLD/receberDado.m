function [ID, POS, SC, flag] = receberDado(Conexao)
flag = 0;
ID = NaN;
POS = NaN(12,1);
SC = NaN(2,1);
while get(Conexao, 'BytesAvailable') && flag ~= 1
    msg = fscanf(Conexao);

if length(msg) > 4 && strcmp(msg(1:4),'$POS')
    flag = -1;
    [A, count ]= sscanf(msg,'$POSX%u:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,SC:%f,%f;');
    if count == 15
        ID = A(1);
        POS = A(2:13);
        SC = A(14:15);
        flag = 1;
        flushinput(Conexao);
    end
end

end
