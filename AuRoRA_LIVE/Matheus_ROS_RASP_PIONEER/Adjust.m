% Get Position and Velocity from Optitrack
rb = OPT.RigidBody;
P = getOptData(rb,P);

map = sub.LatestMessage;
Map1 = [cos(P.pPos.X(6)) -sin(P.pPos.X(6)); sin(P.pPos.X(6)) cos(P.pPos.X(6))]*map.readCartesian' +P.pPos.X([1 2]); 
Map1 = [Map1' map.Ranges];

n = 1:2:length(Map1(:,1));
Map1(n,:) = []; 

%% Find the objects vertices through the Map
[Min,Vert,Max]= H.ObjectSearch(Map1);
V_box_robot = Map1(Min,1:2)-P.pPos.X([1 2])';

for j=1:length(V_box_robot(:,1))
    Dist_box_robot(j) = norm(V_box_robot(j,:));
end

[~,I]= min(Dist_box_robot);

try
    [Xv,Yv] = H.ProjectionFace(Map1,Min(I),Vert(I),Max(I));
    clear Min Vert Max;
    [BoxAdjust] = H.Orthogonal(Map1,Xv,Yv,Goal);
    clear Xv Yv;
end
Erro = norm(NavigationPath(:,2)-[BoxAdjust(5); BoxAdjust(6)])
if Erro>0.3 && Erro<2
     
    NavigationPath(:,1) = [BoxAdjust(3); BoxAdjust(4)];
    NavigationPath(:,2) = [BoxAdjust(5); BoxAdjust(6)];
    NavigationPath = [P.pPos.X(1) BoxAdjust(1) NavigationPath(1,:);...
                        P.pPos.X(2) BoxAdjust(2) NavigationPath(2,:)];
                    
    Curve1 = H.BezierCurve(NavigationPath);
    b = plot(Curve1(1,:),Curve1(2,:),'k');
        
    [arclen,~] = H.ArcLength(Curve1(1,:),Curve1(2,:));
    tmax = arclen/0.2;
    t = tic;
end

