function mLerDadosOdometria(obj)


%if obj.verificaConexao
    try
        poseData = receive(obj.pose.sub);
        q = [poseData.Pose.Pose.Orientation.W poseData.Pose.Pose.Orientation.X poseData.Pose.Pose.Orientation.Y poseData.Pose.Pose.Orientation.Z];
        obj.pPos.Xc(1) = poseData.Pose.Pose.Position.X;
        obj.pPos.Xc(2) = poseData.Pose.Pose.Position.Y;    
        axang          = quat2angle(q);
        obj.pPos.Xc(6) = axang(1);    

        obj.pSC.U(1) = sqrt(poseData.Twist.Twist.Linear.X^2 + poseData.Twist.Twist.Linear.Y^2);
        obj.pSC.U(2) = poseData.Twist.Twist.Angular.Z;

        K2 = [ cos(obj.pPos.Xc(6)), -obj.pPar.a*sin(obj.pPos.Xc(6)); ...
        sin(obj.pPos.Xc(6)), +obj.pPar.a*cos(obj.pPos.Xc(6))];

        K1 = [ cos(obj.pPos.Xc(6)), 0; ...
               sin(obj.pPos.Xc(6)), 0];

        obj.pPos.Xc(7:8) = K1 * obj.pSC.U;
        obj.pPos.Xc(12) = obj.pSC.U(2);    
        % Posição de controle do robôs
        obj.pPos.Xs([1 2 3]) = obj.pPos.Xc([1 2 3]) + [obj.pPar.a*cos(obj.pPos.Xc(6)); obj.pPar.a*sin(obj.pPos.Xc(6)); 0];
        obj.pPos.Xs([4 5 6]) = obj.pPos.Xc([4 5 6]); 
        obj.pPos.Xs(7:8) = K2 * obj.pSC.U;
        obj.pPos.Xs(12) = obj.pSC.U(2);
    catch
        %cfprintf('oi\n');
    end
    
%else    
    %fprintf('Alerta: sem conexão com RosAria \n');
    %obj.pPos.Xs = obj.pPos.X;    
    % Posição do centro do robôs
    %obj.pPos.Xc([1 2 6]) = obj.pPos.X([1 2 6]) - [obj.pPar.a*cos(obj.pPos.Xs(6)); obj.pPar.a*sin(obj.pPos.Xs(6)); 0];   
    
%end
