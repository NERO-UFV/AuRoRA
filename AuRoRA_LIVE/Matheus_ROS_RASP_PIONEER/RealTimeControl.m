map = sub.LatestMessage;
Map1 = [cos(P.pPos.X(6)) -sin(P.pPos.X(6)); sin(P.pPos.X(6)) cos(P.pPos.X(6))]*map.readCartesian' +P.pPos.X([1 2]); 
L = length(Map1(1,:));
Map1 = [Map1' map.Ranges(1:L)];

n = 1:2:length(Map1(:,1));
Map1(n,:) = []; 

%% Find the objects vertices through the Map
[Min,Vert,Max]= H.ObjectSearch(Map1);
V_box_robot = Map1(Min,1:2)-P.pPos.X([1 2])';
Dist_box_robot2 = [];

for j=1:length(V_box_robot(:,1))
    Dist_box_robot2(j) = norm(V_box_robot(j,:));
end

[~,I]= min(Dist_box_robot2);
try
    Mpoint = (Map1(Min(I),1:2)+Map1(Max(I),1:2))/2;
                
    
    Erro_trajectore= norm(Curve(:,path.k)-Mpoint')
    Dist_Box_Goal = norm(Mpoint-Goal(1:2));
    
    
    
    if Erro_trajectore>0.15 

        while norm(Mpoint'-P.pPos.X([1 2]))<0.6
            rb = OPT.RigidBody;
            P = getOptData(rb,P);
            
            P.pSC.Ud = [-0.2;0];
            cmd_vel.Linear.X = P.pSC.Ud(1);
            cmd_vel.Angular.Z =P.pSC.Ud(2);
            send(pub,cmd_vel)
        end
        
        
        P.pSC.Ud =[0;0];
        cmd_vel.Linear.X = P.pSC.Ud(1);
        cmd_vel.Angular.Z = P.pSC.Ud(2);
        send(pub,cmd_vel)
        t = tic;
        % Get Position and Velocity from VREP
        rb = OPT.RigidBody;
        P = getOptData(rb,P);
        
        
        try
            [Xv,Yv] = H.ProjectionFace(Map1,Min(I),Vert(I),Max(I));
            clear Min Vert Max;
            [BoxAdjust] = H.Orthogonal(Map1,Xv,Yv,Goal);
            clear Xv Yv;
        end
        
        [~,I] = min(sqrt((BoxAdjust(5) - NavigationPath(1,:)).^2 + (BoxAdjust(6) - NavigationPath(2,:)).^2));
        NavigationPath(:,1:I) = [];
        NavigationPath = [P.pPos.X([1 2]) BoxAdjust([1 2])' BoxAdjust([3 4])' BoxAdjust([5 6])' NavigationPath];
        Curve1 = H.BezierCurve(NavigationPath(:,1:3));
        Curve2 = H.BezierCurve(NavigationPath(:,3:end));
        Curve = [];
        Curve = [Curve1 Curve2];
        hold on
        plot(Curve(1,:),Curve(2,:))
        
        i=1;
        
       
    end
end

% figure(1)
% hold on
% axis([-6,6,-6,6])
% h = H.PlotMap(Map1,Min(I),[],Max(I));
% 
% hold on
% Curve = H.BezierCurve(NavigationPath)
% plot(Curve(1,:),Curve(2,:))
% plot(Mpoint(1),Mpoint(2),'o')
% 
% plot(Vrotate1(1),Vrotate1(2),'o')
% plot(Vrotate2(1),Vrotate2(2),'o')

