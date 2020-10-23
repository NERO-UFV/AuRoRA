function [Minimum, Vertice, Maximum] = ObjectSearch(hpo,Map)
   
    %% Calculate differences between consecutive distances
    Dif= zeros(1,length(Map(:,1)));
    for i=2:length(Map(:,1))
        Dif(i) = Map(i,3)-Map(i-1,3);
    end

    %% Find the Maximum and Minimum
    Minimum= []; Maximum = [];
    
    for i=1:length(Dif)
        if Dif(i)<-0.5
            for j=i:length(Dif)
                if Dif(j)>0.5
                    L = norm(Map(i,1:2)-Map(j-1,1:2));
                    if L>0.10 && L<0.7
                        Minimum = [Minimum i];
                        Maximum = [Maximum j-1];
                        break
                    else
                        break
                    end
                end
            end
        end
    end
%     for i=1:length(Dif)
%         if Dif(i)>0.5
%             Maximum = [Maximum i];
%         end
%     end
    
%     if Minimum(1)>Maximum(1)
%         Maximum(1) = [];
%     end
%% ==================================================   
    Vertice = [];
    Maximum = Maximum-1;
    if isempty(Minimum)==0 && isempty(Maximum)==0
        for i=1:length(Maximum)
            for j=Minimum(i):Maximum(i)
%                 [~,aux] = min(Map(Minimum(i):Maximum(i),3));
               tf = hpo.Collinear([Map(Minimum(i):j,1) Map(Minimum(i):j,2)],1e-1);
               if tf==0
                   Vertice = [Vertice j-1];
                   break
               elseif j==Maximum(i)
                   Vertice = [Vertice j];
               end
            end
%             Vertice = [Vertice j];
            
        end    
    end
%     if isempty(Vertice)==0
%         Vertice = Vertice+ Minimum(1:length(Maximum))-1;
%     end
% ==================================================       
    
end
