%%% Calcular a inclinação das faces
function [Xv,Yv]=ProjectionFace(hpo,Map,Min,Vert,Max)
    Xv=[]; Yv=[];
    %Projection of Faces
    for i=1:length(Max)
        DifMaxVert = norm(Map(Max(i),1:2)-Map(Vert(i),1:2));
        DifMinVert = norm(Map(Min(i),1:2)-Map(Vert(i),1:2));
        if DifMaxVert>DifMinVert
            % First Face
%             i=1;
            Vector = Map(Vert(i),1:2)-Map(Max(i),1:2);
%             plot(Map(Vert(i),1),Map(Vert(i),2),'o')
%             plot(Map(Max(i),1),Map(Max(i),2),'o')
            
            %% Rotate 
            Vrotate = [cos(-pi/2) sin(-pi/2);...
                    -sin(-pi/2) cos(-pi/2)]*Vector'+Map(Max(i),1:2)';

%             plot(Vrotate(1),Vrotate(2),'o')

            x1 = linspace(Map(Max(i),1),Vrotate(1),6);
            y1 = linspace(Map(Max(i),2),Vrotate(2),6);
            hold on
            plot(x1,y1,'.b');

            
            %Second Face

%             x2 = Vector(1)+Vrotate(1);
%             y2 = Vector(2)+Vrotate(2);
            
            x2 = linspace(Vrotate(1),Vector(1)+Vrotate(1),6);
            y2 = linspace(Vrotate(2),Vector(2)+Vrotate(2),6);
            hold on
            plot(x2,y2,'.b');
            
            
            Xv = [Xv; x1(end) Map(Min(i),1) Map(Vert(i),1) Map(Max(i),1)];
            Yv = [Yv; y1(end) Map(Min(i),2) Map(Vert(i),2) Map(Max(i),2)];
            
            
%             hold on
%             plot(x2,y2,'.b');
            
            
        else
%             i=3
            Vector = Map(Vert(i),1:2)-Map(Min(i),1:2);
%             plot(Map(Vert(i),1),Map(Vert(i),2),'o')
%             plot(Map(Min(i),1),Map(Min(i),2),'o')

            %% Rotate 
            Vrotate = [cos(pi/2) sin(pi/2);...
                -sin(pi/2) cos(pi/2)]*Vector'+Map(Min(i),1:2)';

%             plot(Vrotate(1),Vrotate(2),'o')

            x1 = linspace(Map(Min(i),1),Vrotate(1),6);
            y1 = linspace(Map(Min(i),2),Vrotate(2),6);
            hold on
            plot(x1,y1,'.b');


            %Second Face
            x2 = linspace(Vrotate(1),Vector(1)+Vrotate(1),6);
            y2 = linspace(Vrotate(2),Vector(2)+Vrotate(2),6);
            hold on
            plot(x2,y2,'.b');

%             
            Xv = [Xv; x1(end) Map(Min(i),1) Map(Vert(i),1) x2(end)];
            Yv = [Yv; y1(end) Map(Min(i),2) Map(Vert(i),2) y2(end)];
            
        end
    end
end