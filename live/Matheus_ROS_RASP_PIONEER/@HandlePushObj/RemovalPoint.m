function [Pcontrol] = RemovalPoint(hpo,Sequence)

    Pcontrol = Sequence(:,1);
    j=1;
    for i=1:length(Sequence(1,:))
        if norm(Sequence(:,i)-Pcontrol(:,j))>1
            Pcontrol = [Pcontrol Sequence(:,i)];
            j=j+1;
        end
    end

end