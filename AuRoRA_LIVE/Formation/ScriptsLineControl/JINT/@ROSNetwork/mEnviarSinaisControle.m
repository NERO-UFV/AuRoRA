function mEnviarSinaisControle(obj)
    %if obj.verificaConexao        
        velmsg = rosmessage(obj.vel.pub);
        velmsg.Linear.X = obj.pSC.Ur(1);    %(obj.pSC.Ur(1)) * cos(obj.pPos.X(6));
        velmsg.Linear.Y = 0;                %(obj.pSC.Ur(1)) * sin(obj.pPos.X(6));
        velmsg.Linear.Z = 0;
        velmsg.Angular.X = 0;
        velmsg.Angular.Y = 0;
        velmsg.Angular.Z = obj.pSC.Ur(2);
        send(obj.vel.pub,velmsg);   
    %else
    %    fprintf('Alerta mEnviarSinaisControle: sem conexão com o RosAria\n');
    %end
end