function resp = IEEE754decript(mensagem)

sinal = mensagem(1);
expom = double(uint8(bi2de(mensagem(2:9), 'left-msb')));
mantissa = double(int32(bi2de(mensagem(10:32), 'left-msb')));

if sinal == 1
    resp = (-1)*(1+mantissa/8388607)*2^(expom-127);
else
    resp = (1+mantissa/8388607)*2^(expom-127);
end

end