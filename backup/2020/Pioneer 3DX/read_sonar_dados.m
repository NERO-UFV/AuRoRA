load sonar_dados

for t = 1:596-1
    X = Sdata(t,:);
    Y = Sdata(t+1,:);
    
    med = mean([X;Y]');
    
    hold on
    plot(med(1),med(2),'r*')
    hold on
    plot(X,Y,'b--');
    pause(0.2)
end