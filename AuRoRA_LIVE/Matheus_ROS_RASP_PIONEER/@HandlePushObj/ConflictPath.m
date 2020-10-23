function [Avoidance] = ConflictPath(hpo,Curve,Obj)
%     Curve = Curve1;
    % Checking for conflicts zones on the path
    conflict = zeros(2,length(Curve(1,:)));
    Avoidance=[];
    for i=1:size(Obj,2)
        % Calculate the distance between Curve and objects
        Zone(i,:) = sqrt((Curve(1,:)-Obj(1,i)).^2+(Curve(2,:)-Obj(2,i)).^2);
        
        for j=1:length(Zone(i,:))
            if Zone(i,j)<0.25
                conflict(1,j) = j;
                conflict(2,j) = i;
            end
        end        
    end
    
    conflict(:,1:15) = [0];
    conflict(:,end-5:end) = [0];
    for i=1:length(conflict(1,:))
        if conflict(1,i)>0
            plot(Curve(1,conflict(1,i)),Curve(2,conflict(1,i)),'o')
        end
    end
    
    for i=1:length(conflict(1,:))
        if conflict(1,i)>0
            vector1 = [Curve(1,conflict(1,i))-Obj(1,conflict(2,i)) Curve(2,conflict(1,i))-Obj(2,conflict(2,i))];
            vector2 = [Curve(1,conflict(1,i))-Curve(1,conflict(1,i)-1) Curve(2,conflict(1,i))-Curve(2,conflict(1,i)-1)];
            Delta1 = vector1(2)/vector1(1);
            Delta2 = vector2(2)/vector2(1);
                
            if Delta2-0.1 < Delta1 && Delta1 < Delta2+0.1
                if atan2(vector1(2),vector1(1))*atan2(vector2(2),vector2(1))<0
                    k=-1;
                else
                    k=1;
                end
                Scale = 1.5/Zone(conflict(2,i),conflict(1,i));
                Aux = [Obj(1,conflict(2,i)) Obj(2,conflict(2,i))]'+Scale*[cos(k*pi/2) -sin(k*pi/2); sin(k*pi/2)  cos(k*pi/2)]*vector1';
                Avoidance = [Avoidance Aux];
            else
                Scale = 0.8/Zone(conflict(2,i),conflict(1,i));
                Aux = [Obj(1,conflict(2,i)) Obj(2,conflict(2,i))]'+Scale*vector1';
                Avoidance = [Avoidance Aux];
            end
        end
    end
            

end