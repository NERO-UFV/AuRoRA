%% MATLAB
%Parametros
%Codigo origem : D:\Doutorado\Codigos\controladorPD-cvdrone-master - DIF DKF - LELIS\build\vs2010\Resultados
clc, clear, close all;
disp('  Teste com Beepop : Prof. Milton');
d = dir('outfile_ardrone_*.txt');
%d = dir('claudio_ardrone_*.txt');
str = {};
for k=1:size(d,1)
    nome = d(k).name;
    str = vertcat(str, ['' nome(1:end-4)  ' | ' num2str(round(d(k).bytes/1024)) ' Kb']);
end
%str = {d.name};
[s,v] = listdlg('PromptString','Selecione o arquivo:',...
    'SelectionMode','single',...
    'ListSize',[230 300],...
    'ListString',str);
data = load(d(s).name);

inicio=796;
fim=size(data,1);
filtro = 2145;

joy_vx = data(inicio:fim,1);
joy_vy = data(inicio:fim,2);
joy_vz = data(inicio:fim,3);
joy_vr = data(inicio:fim,4);
roll = data(inicio:fim,5);
pitch = data(inicio:fim,6);
yaw= data(inicio:fim,7);
vx= data(inicio:fim,8);
vy= data(inicio:fim,9);
vz= data(inicio:fim,10);
dt= data(inicio:fim,11);
z= data(inicio:fim,12);

acx= 9.81*data(inicio:fim,13);
acx = acx-9.81*mean(data(1:4,13));
acy= 9.81*data(inicio:fim,14);
acy = acy -9.81*mean(data(1:4,14));
acz= 9.81*data(inicio:fim,15);
acz = acz -9.81*mean(data(1:4,15));

acroll= 9.81*data(inicio:fim,16);
acpitch= 9.81*data(inicio:fim,17);
acpsi= 9.81*data(inicio:fim,18);

vpsi = (yaw(2:end) - yaw(1:end-1))./dt(2:end);
vpsi = vertcat(0,vpsi);
mean(dt)
%
for k=1:length(acx)
             F = [cos(yaw(k))   -sin(yaw(k))   0   0;
                 sin(yaw(k))   cos(yaw(k))    0   0;
                 0          0           1   0;
                 0          0           0   1];
    Vw = F*[vx(k);vy(k);vz(k);vpsi(k)];
    
    vxw(k) = Vw(1);
    vyw(k) = Vw(2);
    vzw(k) = Vw(3);
    vpsiw(k) = Vw(4);
    
    Acw = F*[acx(k); acy(k); acz(k); acpsi(k)];
    acxw(k) = Acw(1);
    acyw(k) = Acw(2);
    aczw(k) = Acw(3);
    acpsiw(k) = Acw(4);
end


id = find(abs(vpsiw) >1);
%while(id)
    for k=1:length(id)
       if(id(k) > 1)
           vpsiw(id(k)) = vpsiw(id(k)-1);
       end
    end
%%

x_initial = 0;
y_initial = 0;
z_initial = z(1);
psi_initial = yaw(1);

xrw(1) = x_initial;
    yrw(1) = y_initial;
    zrw(1) = z_initial;
    psir(1) = psi_initial;
    
accx(1) = acxw(1);
accy(1) = acyw(1);
accz(1) = aczw(1);
accpsi(1) = acpsiw(1);

for k=2:length(vxw)    
    xrw(k) = xrw(k-1) + vxw(k)*dt(k);
    yrw(k) = yrw(k-1) + vyw(k)*dt(k);
    zrw(k) = zrw(k-1) + vzw(k)*dt(k);
    psir(k) = psir(k-1) + vpsiw(k)*dt(k);
    
    accx(k) = (vxw(k) - vxw(k-1))/dt(k);
    accy(k) = (vyw(k) - vyw(k-1))/dt(k);
    accz(k) = (vzw(k) - vzw(k-1))/dt(k);
    accpsi(k) = (vpsiw(k) - vpsiw(k-1))/dt(k);
end

%accx = predice(accx, 20, 0.1);
accpsi = predice(accpsi, 20, 0.01);
 %%
 drone_acoplado = DroneMilton_Trajetoria;
 drone_acoplado.SetEstadoInicial(x_initial, y_initial, z_initial, psi_initial, vxw(1), vyw(1), vzw(1), vpsiw(1), acxw(1), acyw(1), aczw(1), acpsiw(1));
 
 %drone_acoplado.SetModelo(19.2784,1.0282,7.6097,0.83822,6.626,7.5627,1.8903,0.54161);

 %drone_acoplado.SetModelo(5, 0.3, 5.5, 0.1, 0.55, 0.7, 1.45, 0.7);
 drone_acoplado.SetModelo(12.6284,1.0887,4.0745, 0.69169, 6.6805,7.6284,1.9936,0.60495);
 %5, 0.3, 5.5, 0.1, 0.55, 0.7, 1.45, 0.7
 
 %des media drone_acoplado.SetModelo(19.2784,1.0282,7.6097,0.83822,6.626,7.5627,1.8903,0.54161);
 %aco media drone_acoplado.SetModelo(33.2893, 1.0282, 13.9867, 0.83822, 6.6258, 7.5627, 1.8929, 0.54161);
 %aco mediana drone_acoplado.SetModelo(19.117,0.83304,12.3528,0.84134,6.6783,7.6193,1.9937,0.60497);
%drone_acoplado.SetModelo(12.6284,1.0887,4.0745, 0.69169, 6.6805,7.6284,1.9936,0.60495);

for k=1:length(acx)
    drone_acoplado.EnviaAcoesManual;
    drone_acoplado.ExecutaAcoesControle(joy_vx(k),joy_vy(k),joy_vz(k),joy_vr(k));            
end

%%
figure,
subplot(3,1,1)
plot(accx), hold on, plot(drone_acoplado.robot_acelx, 'r--')
title('Accelx')
legend('Real','Modeled');
axis tight
subplot(3,1,2)
plot(vxw), hold on, plot(drone_acoplado.robot_vx, 'r--')
title('Vx')
axis tight
subplot(3,1,3)
plot(xrw), hold on, plot(drone_acoplado.robot_x, 'r--')
title('X')
axis tight
%%
figure,
subplot(3,1,1)
plot(accy), hold on, plot(drone_acoplado.robot_acely, 'r--')
title('Accely');
legend('Real','Modeled');
axis tight
subplot(3,1,2)
plot(vyw), hold on, plot(drone_acoplado.robot_vy, 'r--')
title('Vy')
axis tight
subplot(3,1,3)
plot(yrw), hold on, plot(drone_acoplado.robot_y, 'r--')
title('y')
axis tight
%%
figure,
subplot(3,1,1)
plot(aczw), hold on, plot(drone_acoplado.robot_acelz, 'r--')
title('Accelz')
axis tight
subplot(3,1,2)
plot(vzw), hold on, plot(drone_acoplado.robot_vz, 'r--')
title('Vz')
axis tight
subplot(3,1,3)
plot(zrw), hold on, plot(drone_acoplado.robot_z, 'r--')
title('z')
axis tight
%%
figure,
subplot(3,1,1)
plot(accpsi), hold on, plot(drone_acoplado.robot_acelpsi, 'r--')
title('Accel \psi')
axis tight
subplot(3,1,2)
plot(vpsiw), hold on, plot(drone_acoplado.robot_vpsi, 'r--')
title('V_{\psi}')
axis tight
subplot(3,1,3)
plot(yaw), hold on, plot(drone_acoplado.robot_psi, 'r--')
title('\psi')
axis tight