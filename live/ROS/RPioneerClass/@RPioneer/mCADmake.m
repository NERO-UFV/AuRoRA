function mCADmake(p3dx)

% Physical Robot P3DX Parameters 

N = 5;

%% Define robot vertices
% Superior surface
x = [-0.075 -0.025 0.26*cos(linspace(-pi/3,pi/3,N)) -0.025 -0.075 0.27*cos(linspace(3/4*pi,5/4*pi,N))];
y = [0.27*sin(5*pi/4) 0.26*sin(-pi/3) 0.26*sin(linspace(-pi/3,pi/3,N)) 0.26*sin(pi/3) 0.27*sin(3*pi/4)  0.27*sin(linspace(3/4*pi,5/4*pi,N))];
z = (0.05+0.215*ones(1,size(y,2)));
p3dx.pCAD.vertices{1} = [x; y; z];

% Sonar suport
x =  [-0.075 -0.025 0.22*cos(linspace(-pi/3,pi/3,N)) -0.025 -0.075 0.23*cos(linspace(3/4*pi,5/4*pi,N))];
y =  [0.23*sin(5*pi/4) 0.22*sin(-pi/3) 0.22*sin(linspace(-pi/3,pi/3,N)) 0.22*sin(pi/3) 0.23*sin(3*pi/4)  0.23*sin(linspace(3/4*pi,5/4*pi,N))];
zt = (0.05 + 0.213*ones(1,size(y,2)));
zb = (0.05 + 0.155*ones(1,size(y,2)));
p3dx.pCAD.vertices{2} = [x;y;zt];
p3dx.pCAD.vertices{3} = [x;y;zb];

% Body
x = [-0.13 0.1 0.175 0.175 0.1 -0.13 -0.195 -0.195];
y = [0.15 0.15 0.1 -0.1 -0.15 -0.15 -0.1 0.1];
zt = (0.05 + 0.154*ones(1,size(y,2)));
zb = (0.05 + 0.05*ones(1,size(y,2)));
p3dx.pCAD.vertices{4} = [x;y;zt];
p3dx.pCAD.vertices{5} = [x;y;zb];

% Wheels
p3dx.pCAD.N_lados = 15;
for k=1:4
    p3dx.pCAD.rodas{k}(1,:) = 0.09*cos(linspace(-pi,pi,p3dx.pCAD.N_lados));
    p3dx.pCAD.rodas{k}(3,:) = (0.09 + .09*sin(linspace(-pi,pi,p3dx.pCAD.N_lados)));
end
p3dx.pCAD.rodas{1}(2,:) = -0.21*ones(1,p3dx.pCAD.N_lados);
p3dx.pCAD.rodas{2}(2,:) = -0.155*ones(1,p3dx.pCAD.N_lados);
p3dx.pCAD.rodas{3}(2,:) = 0.155*ones(1,p3dx.pCAD.N_lados);
p3dx.pCAD.rodas{4}(2,:) = 0.21*ones(1,p3dx.pCAD.N_lados);


% % Castor Wheel
% x = -0.195 + 0.057*cos(linspace(-pi,pi,p3dx.pCAD.N_lados));
% y = [-0.02 0.02];
% z = 0.03+ 0.057*sin(linspace(-pi,pi,p3dx.pCAD.N_lados));
% p3dx.roda_apoio = [x;y;z]';

%% Fill Robot

% Superior sonar suportSuporte /Inferior sonar suport(TB)
N1 = size(p3dx.pCAD.vertices{2},2);

for k=1:(N1-1)
    x = [p3dx.pCAD.vertices{2}(1,[k k+1]) p3dx.pCAD.vertices{3}(1,[k+1 k])];
    y = [p3dx.pCAD.vertices{2}(2,[k k+1]) p3dx.pCAD.vertices{3}(2,[k+1 k])];
    z = (0.05 + [0.213*ones(1,2) 0.155*ones(1,2)]);
    p3dx.pCAD.corpo{k} = [x;y;z];
end
p3dx.pCAD.corpo{N1} = [p3dx.pCAD.vertices{2}(1,[N1 1]) p3dx.pCAD.vertices{3}(1,[1 N1])
    p3dx.pCAD.vertices{2}(2,[N1 1]) p3dx.pCAD.vertices{3}(2,[1 N1])
    0.05+0.213*ones(1,2)       0.05+0.155*ones(1,2)];

% Superior Body/ Inferior Body (BB)
N2 = size(p3dx.pCAD.vertices{4},2);
cont = N1;
for k=1:(N2-1)
    cont = cont+1;
    x = [p3dx.pCAD.vertices{4}(1,[k k+1]) p3dx.pCAD.vertices{5}(1,[k+1 k])];
    y = [p3dx.pCAD.vertices{4}(2,[k k+1]) p3dx.pCAD.vertices{5}(2,[k+1 k])];
    z = (0.05 + [ 0.154*ones(1,2)    0.05*ones(1,2)]);
    p3dx.pCAD.corpo{cont} = [x;y;z];
end
p3dx.pCAD.corpo{cont+1} = [p3dx.pCAD.vertices{4}(1,[N2 1]) p3dx.pCAD.vertices{5}(1,[1 N2])
    p3dx.pCAD.vertices{4}(2,[N2 1]) p3dx.pCAD.vertices{5}(2,[1 N2])
    0.05+0.154*ones(1,2)       0.05+0.05*ones(1,2)];

% Wheels (WS)

cont = size(p3dx.pCAD.rodas,2);

for k=1:(p3dx.pCAD.N_lados-1)
    cont = cont+1;
    x = [p3dx.pCAD.rodas{1}(1,[k k+1]) p3dx.pCAD.rodas{2}(1,[k+1 k])];
    y = [p3dx.pCAD.rodas{1}(2,[k k+1]) p3dx.pCAD.rodas{2}(2,[k+1 k])];
    z = [p3dx.pCAD.rodas{1}(3,[k k+1]) p3dx.pCAD.rodas{2}(3,[k+1 k])];
    p3dx.pCAD.rodas{cont} = [x;y;z];
end
p3dx.pCAD.rodas{cont+1} = [p3dx.pCAD.rodas{1}(1,[p3dx.pCAD.N_lados 1])  p3dx.pCAD.rodas{2}(1,[1 p3dx.pCAD.N_lados])
    p3dx.pCAD.rodas{1}(2,[p3dx.pCAD.N_lados 1])  p3dx.pCAD.rodas{2}(2,[1 p3dx.pCAD.N_lados])
    p3dx.pCAD.rodas{1}(3,[p3dx.pCAD.N_lados 1])  p3dx.pCAD.rodas{2}(3,[1 p3dx.pCAD.N_lados])];
cont = cont+1;

for k=1:(p3dx.pCAD.N_lados-1)
    cont = cont+1;
    x = [p3dx.pCAD.rodas{3}(1,[k k+1]) p3dx.pCAD.rodas{4}(1,[k+1 k])];
    y = [p3dx.pCAD.rodas{3}(2,[k k+1]) p3dx.pCAD.rodas{4}(2,[k+1 k])];
    z = [p3dx.pCAD.rodas{3}(3,[k k+1]) p3dx.pCAD.rodas{4}(3,[k+1 k])];
    p3dx.pCAD.rodas{cont} = [x;y;z];
end
p3dx.pCAD.rodas{cont+1} = [p3dx.pCAD.rodas{3}(1,[p3dx.pCAD.N_lados 1])  p3dx.pCAD.rodas{4}(1,[1 p3dx.pCAD.N_lados])
    p3dx.pCAD.rodas{3}(2,[p3dx.pCAD.N_lados 1])  p3dx.pCAD.rodas{4}(2,[1 p3dx.pCAD.N_lados])
    p3dx.pCAD.rodas{3}(3,[p3dx.pCAD.N_lados 1])  p3dx.pCAD.rodas{4}(3,[1 p3dx.pCAD.N_lados])];

% % Castor Wheel (SWS)
% for k=1:(p3dx.pCAD.N_wh-1)
% p3dx.x_sws{k} = [p3dx.pCAD.x_swh(1,[k k+1]) p3dx.pCAD.x_swh(1,[k+1 k])];
% p3dx.y_sws{k} = [ p3dx.pCAD.y_swh(1)*ones(1,2) p3dx.pCAD.y_swh(2)*ones(1,2)];
% p3dx.z_sws{k} = [p3dx.pCAD.z_swh(1,[k k+1]) p3dx.pCAD.z_swh(1,[k+1 k])];
% 
% end
% p3dx.x_sws{p3dx.pCAD.N_wh} = [p3dx.pCAD.x_swh(1,[p3dx.pCAD.N_wh 1]) p3dx.pCAD.x_swh(1,[1 p3dx.pCAD.N_wh])];
% p3dx.y_sws{p3dx.pCAD.N_wh} = [ p3dx.pCAD.y_swh(1)*ones(1,2) p3dx.pCAD.y_swh(2)*ones(1,2)];
% p3dx.z_sws{p3dx.pCAD.N_wh} = [p3dx.pCAD.z_swh(1,[p3dx.pCAD.N_wh 1]) p3dx.pCAD.z_swh(1,[1 p3dx.pCAD.N_wh])];


%% Assemble 2D robot
p3dx.pCAD.corpo2D = [0 0.3 0; -0.15 0 0.15; zeros(1,3)];
p3dx.pCAD.rodad2D = [0.07 0.07 -0.07 -0.07; -0.15 -0.18 -0.18 -0.15; zeros(1,4)];
p3dx.pCAD.rodae2D = [0.07 0.07 -0.07 -0.07;  0.15  0.18  0.18  0.15; zeros(1,4)];



end