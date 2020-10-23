function CriarConexoes(obj)
n = length(Madj);

for ii = 1:n
    
        Conexoes(ii) = udp('127.0.0.1', 'localPort', 25000+ii);
   
end

fopen(Conexoes);

end