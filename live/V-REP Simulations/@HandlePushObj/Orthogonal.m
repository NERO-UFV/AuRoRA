function [ObjList] = Orthogonal(hpo,Map,Xv,Yv,Goal)
    ObjList = [];
    %% Midpoint of the Boxes Face
    Xm = zeros(size(Xv));
    Ym = zeros(size(Yv));
    
    for j=1:size(Xv,1)
        for i=1:size(Xv,2)
            Xm(j,i) = (Xv(j,i)+Xv(j,i+1))/2;
            Ym(j,i) = (Yv(j,i)+Yv(j,i+1))/2;

            if i+1 == size(Xv,2)
                Xm(j,i+1) = (Xv(j,1)+Xv(j,i+1))/2;
                Ym(j,i+1) = (Yv(j,1)+Yv(j,i+1))/2;
                break;
            end
        end
    end
    hold on
    

    %% Plot the Goal point
    plot(Goal(1), Goal(2),'o', ...
            'MarkerSize', 5,'MarkerEdgeColor','k','MarkerFaceColor' , 'r')
    
    %% Find the best face to approach
    for j=1:size(Xm,1)
        % Direction Vectors from Midpoint to Goal
        VectorX = Goal(1)-Xm(j,:);
        VectorY = Goal(2)-Ym(j,:);

        % Normal Vector Boxes Face
        VectorFx(1) = Xm(j,3)-Xm(j,1);
        VectorFx(2) = Xm(j,4)-Xm(j,2);
        VectorFx(3) = Xm(j,1)-Xm(j,3);
        VectorFx(4) = Xm(j,2)-Xm(j,4);

        VectorFy(1) = Ym(j,3)-Ym(j,1);
        VectorFy(2) = Ym(j,4)-Ym(j,2);
        VectorFy(3) = Ym(j,1)-Ym(j,3);
        VectorFy(4) = Ym(j,2)-Ym(j,4);

        %Find the Resultant of Direction+Normal
        for i=1:size(VectorX,2)
            
            Result(i,:) = [VectorX(i) VectorY(i)]+[VectorFx(i) VectorFy(i)];
            Module(i) = sqrt(Result(i,1)^2+Result(i,2)^2);
        end
        
        [~,index] = max(Module);
        % Find the orthogonal auxiliar point
        alpha = VectorFy(index)/VectorFx(index);
        d = 1.2;
        Xb(1) = d/sqrt(alpha^2+1)+Xm(j,index);
        Xb(2) = -d/sqrt(alpha^2+1)+Xm(j,index);
        
        Yb(1) = alpha*(Xb(1)-Xm(j,index))+Ym(j,index);
        Yb(2) = alpha*(Xb(2)-Xm(j,index))+Ym(j,index);
        
        % Box center
        Xc(j) = (Xm(j,3)+Xm(j,1))/2;
        Yc(j) = (Ym(j,3)+Ym(j,1))/2;
        
        % Find the correct position

        for i=1:2
            Direct(i,:) = [Xm(j,index) Ym(j,index)]-[Xb(i) Yb(i)];
            Resultant(i,:) = [VectorFx(index) VectorFy(index)]+[Direct(i,1) Direct(i,2)];
            L(i) = sqrt(Resultant(i,1)^2+Resultant(i,2)^2);
        end
        [~,k] = max(L);

         plot(Xc(j),Yc(j),'*');
%         plot([Xb(k) Xm(j,index)],[Yb(k) Ym(j,index)]);
        ObjList = [ObjList; Xb(k) Yb(k) Xm(j,index) Ym(j,index) Xb(Xb~=Xb(k)) Yb(Yb~=Yb(k)) Xc(j) Yc(j)];
    end
end

