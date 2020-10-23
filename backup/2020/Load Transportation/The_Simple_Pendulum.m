clear;
%% The simple Pendulum
% q = [x y z phi tht];

C.m = .1;
C.g = 9.8;
C.l = 1;

C.I = C.m*C.l^2;
T = .05;
u = .0;
b = .05;

% q = [x y z p t]; 
ddq = [0 0 0 0 0]; 
 dq = [0 0 0 0 0];
  q = [0 0 0 1 1];
%%
h = figure(1);
set(h,'units','pix','pos',[300 150 1200 800],'PaperPositionMode','auto')
axis image
xlim = [-1.5 1.5];
ylim = [-2 0];
% zlim = [ -2 0];
axis([xlim ylim])
grid on
hold on
xlabel('x [m]','interpreter','latex')
ylabel('z [m]','interpreter','latex')
zlabel('z [m]','interpreter','latex')

% view(45,22)
%%
t.laco = tic;
t.atual = tic;
while toc(t.atual) < 25
    if toc(t.laco)> T
        t.laco = tic;
         
        ddq(4) = 1/C.I*(u -b(1)*dq(4) - C.m*C.g*C.l*sin(q(4)));
         dq(4) = dq(4) + ddq(4)*T;
          q(4) =  q(4) +  dq(4)*T;
         
        q(1) = C.l*sin(q(4));
        q(2) = C.l*sin(q(5));
        q(3) = -C.l*cos(q(4));
        
        
        try delete(ds); end
        ds(1) = plot(q(1),q(3),'.','Color','k','MarkerSize',50);
        hold on
        ds(2) = plot([0 q(1)],[0 q(3)],'Color','k');
        drawnow
        display('.');
    end
end
disp('end');
%         ddq(4) = 1/C.I*(u -b(1)*dq(4) - C.m*C.g*C.l*sin(q(4)));
%          dq(4) = dq(4) + ddq(4)*T;
%           q(4) =  q(4) +  dq(4)*T;