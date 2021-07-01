function [H,globOH,x,y] = add_Catamara(X,Y,n,H,height)
%add_Catamara Summary of this function goes here
%   Detailed explanation goes here
 
width=15;
r=ceil(width/2);


%!!!
pos=[X,Y];
x=X;
y=Y;
% get(handles.axes3,'currentpoint');

Dropx=floor(pos(1,1));
Dropy=floor(pos(1,2));
max=n-r-1;
min=r+1;
Dropx(Dropx>max)=max;
Dropx(Dropx<min)=min;
Dropy(Dropy>max)=max;
Dropy(Dropy<min)=min;
[x,y] = ndgrid(-1:(2/(width)):1);


a=1;
b=0;
c=0;
d=0;
e=0;
f=0;
g=0;

if b      % Type 2:
    [x,y] = ndgrid(-1.5:(2/(width/1.5-1)):1);
    D = height*0.5*(exp(-18*x.^2-0.5*y.^2).*cos(4*x)+...
        exp(-3*((x+0.5).^2+0.5*y.^2)));
elseif c  % Type 3:
    [x,y] = ndgrid(-2.5:(5/(width)):2.5);
    D = height*1.4*y .* exp(-x.^2 - y.^2);
elseif d  % Matlab Logo:
    D = height*membrane(1,r);
elseif e  % Peaks:
    [x,y] = ndgrid(-3:(6/(width)):3);
    D=height/12*peaks(x,y);
elseif a  % Type 1:
    D = height*exp(-5*(x.^2+y.^2));
elseif f  % Half Sphere:
    [x,y] = ndgrid(-2.1:(4.2/(width)):2.1);
    D=height/4*real((4-x.^2-y.^2).^0.5);
elseif g  % Bethoven:
    D=height/5*Z;
    D(1,:)=0;
    D(:,1)=0;
    Dropx=2;
    Dropy=2;
    r=1;
else         % Hi
    set(handles.Speed,'String','0.54');
    set(handles.sliderSpeed,'value',0.54);
    set(handles.Damp,'String','1');
    set(handles.sliderDamp,'value',1);
    D=hello-1;
    Dropx=r+1;
    Dropy=r+1;
end

w = size(D,1);
i2 = (Dropx-r):w+(Dropx-r)-1;
j2 = (Dropy-r):w+(Dropy-r)-1;
H(i2,j2)=1;
H(i2,j2) = H(i2,j2) + D;
globOH(i2,j2) = H(i2,j2);

end

