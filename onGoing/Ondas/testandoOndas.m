%% Boas praticas
clearvars; clc;

%% Importando catamara
% Definindo propriedades do catamara
% barco = Catamara;
% barco.pPos.X([1 2]) = [10,50];
PesoVirtual = -50;
% barco.mCADload

%% Inicializando superficie

n=100; % limites (X,Y) em cm
H=zeros(n,n)-2; % gera area de 1 m2
mesh(H);

% Energia cinetica e potencial
Ekin=[]; Epot=[];
oldH=H;
newH=H;
i = 2:n-1;
j = 2:n-1;
h=surf(newH);

% Estilizando superficie
lighting phong;
material shiny;
lightangle(-45,30)
light('Position',[-10 20 10]);
axis([1 n 1 n -50 100]);

% Inicializando variaveis (add_Catamara)
globOH=0;

% Colocar Catamara no (x,y)=5,5
altura = PesoVirtual;
% [H,globOH] = add_Catamara(barco.pPos.X(1),barco.pPos.X(2),n,H,altura);
 % plotar catamara
[H,globOH] = add_Catamara(10,50,n,H,altura);

while 1
    newH=Wave(n,i,j,0.05,12,0.2,H,oldH,0,0,0);
            % n,i,j,dt,c,k,H,oldH,fix,cont,connect
    set(h,'zdata',newH);
    pause(0.05);
    oldH=H;
    H=newH;
end

 
 
 
 


 
 
 
