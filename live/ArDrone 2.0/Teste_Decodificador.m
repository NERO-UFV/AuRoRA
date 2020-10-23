clear; close; clc

salvo = load('a');
% salvo = load('D:\Downloads\salvo2.mat');
% salvo = load('D:\Downloads\salvo4.mat');

Dados = salvo.a;
fprintf(['Tamnho dos Dados é: ',num2str(size(Dados)),'\n']);
%%
for i = 1:size(Dados,2)
    Navdata.Demoraw{1,i} = dec2hex(Dados(1:4,i),2);
    Navdata.Demoraw{2,i} = dec2hex(Dados(9:12,i),2);   
    Navdata.Demoraw{3,i} = dec2hex(Dados(25:28,i),2);      % Bateria
    Navdata.Demoraw{4,i} = dec2hex(Dados(29:32,i),2);      % Pitch
    Navdata.Demoraw{5,i} = dec2hex(Dados(33:36,i),2);      % Roll
    Navdata.Demoraw{6,i} = dec2hex(Dados(37:40,i),2);      % Yaw
    Navdata.Demoraw{7,i} = dec2hex(Dados(41:44,i),2);      % Altitude
    Navdata.Demoraw{8,i} = dec2hex(Dados(45:48,i),2);      % Vx
    Navdata.Demoraw{9,i} = dec2hex(Dados(49:52,i),2);      % Vy
    Navdata.Demoraw{10,i} = dec2hex(Dados(527:530,i),2);     % Vz
    Navdata.Demoraw{11,i} = dec2hex(Dados(443:446,i),2);     % Vz
    % gaz_altitude  = 443:446
    % // Raw measurements
    % unsigned short tag;
    % unsigned short size;
    % unsigned short raw_accs[3];         // filtered accelerometers
    % short          raw_gyros[3];        // filtered gyrometers
    % short          raw_gyros_110[2];    // gyrometers  x/y 110 deg/s
    % unsigned int   vbat_raw;            // battery voltage raw (mV)
    % unsigned short us_debut_echo;
    % unsigned short us_fin_echo;
    % unsigned short us_association_echo;
    % unsigned short us_distance_echo;
    % unsigned short us_courbe_temps;
    % unsigned short us_courbe_valeur;
    % unsigned short us_courbe_ref;
    % unsigned short flag_echo_ini;
    % //unsigned short frame_number;
    % unsigned short nb_echo;
    % unsigned int   sum_echo;
    % int            alt_temp_raw;
    % short          gradient;
    for k = 1:20
        Navdata.RawMeasuresraw{k,i} = dec2hex(Dados(173+(k*2-2):174+(k*2-2),i),2);
    end
    
    % // Timestamp
    % unsigned short tag;
    % unsigned short size;
    % unsigned int   time;
    Navdata.TimeStampRaw{1,i} = dec2hex(Dados(165:166,i),2);
    Navdata.TimeStampRaw{2,i} = dec2hex(Dados(167:168,i),2);
    Navdata.TimeStampRaw{3,i} = dec2hex(Dados(169:172,i),2);
    
    % // Physical measurements
    % unsigned short tag;
    % unsigned short size;
    % float          accs_temp;
    % unsigned short gyro_temp;
    % float          phys_accs[3];
    % float          phys_gyros[3];
    % unsigned int   alim3V3;         // 3.3 volt alim       [LSB]
    % unsigned int   vrefEpson;       // ref volt Epson gyro [LSB]
    % unsigned int   vrefIDG;         // ref volt IDG gyro   [LSB]

    Navdata.PhysicalMeasurementsRaw{1,i} = dec2hex(Dados(229:232,i),2); % Acc Temp
    Navdata.PhysicalMeasurementsRaw{2,i} = dec2hex(Dados(233:234,i),2);
    %Filtered Acc
    Navdata.PhysicalMeasurementsRaw{3,i} = dec2hex(Dados(235:238,i),2);
    Navdata.PhysicalMeasurementsRaw{4,i} = dec2hex(Dados(239:242,i),2);
    Navdata.PhysicalMeasurementsRaw{5,i} = dec2hex(Dados(243:246,i),2);
    %Filtered Gyro
    Navdata.PhysicalMeasurementsRaw{6,i} = dec2hex(Dados(247:250,i),2);
    Navdata.PhysicalMeasurementsRaw{7,i} = dec2hex(Dados(251:254,i),2);
    Navdata.PhysicalMeasurementsRaw{8,i} = dec2hex(Dados(255:258,i),2);  
end
%%
%%
%% Decriptografar o NavDataDemo
for i = 1:size(Dados,2)
    %%
    Navdata.Demo{1}(i) = hex2dec([Navdata.Demoraw{3,i}(4,:) Navdata.Demoraw{3,i}(3,:) Navdata.Demoraw{3,i}(2,:) Navdata.Demoraw{3,i}(1,:)]); % Bateria
    Navdata.Demo{2}(i) = hex2dec([Navdata.Demoraw{7,i}(4,:) Navdata.Demoraw{7,i}(3,:) Navdata.Demoraw{7,i}(2,:) Navdata.Demoraw{7,i}(1,:)]); % Altitude
    
    Navdata.Demo{3}(i) = IEEE754decript(hexToBinaryVector([Navdata.Demoraw{4,i}(4,:) Navdata.Demoraw{4,i}(3,:) Navdata.Demoraw{4,i}(2,:) Navdata.Demoraw{4,i}(1,:)],32)); % Pitch
    Navdata.Demo{4}(i) = IEEE754decript(hexToBinaryVector([Navdata.Demoraw{5,i}(4,:) Navdata.Demoraw{5,i}(3,:) Navdata.Demoraw{5,i}(2,:) Navdata.Demoraw{5,i}(1,:)],32)); % Roll
    Navdata.Demo{5}(i) = IEEE754decript(hexToBinaryVector([Navdata.Demoraw{6,i}(4,:) Navdata.Demoraw{6,i}(3,:) Navdata.Demoraw{6,i}(2,:) Navdata.Demoraw{6,i}(1,:)],32)); % Yaw

    Navdata.Demo{6}(i) = IEEE754decript(hexToBinaryVector([Navdata.Demoraw{8,i}(4,:) Navdata.Demoraw{8,i}(3,:) Navdata.Demoraw{8,i}(2,:) Navdata.Demoraw{8,i}(1,:)],32)); % Vx
    Navdata.Demo{7}(i) = IEEE754decript(hexToBinaryVector([Navdata.Demoraw{9,i}(4,:) Navdata.Demoraw{9,i}(3,:) Navdata.Demoraw{9,i}(2,:) Navdata.Demoraw{9,i}(1,:)],32)); % Vy
    Navdata.Demo{10}(i) = IEEE754decript(hexToBinaryVector([Navdata.Demoraw{10,i}(4,:) Navdata.Demoraw{10,i}(3,:) Navdata.Demoraw{10,i}(2,:) Navdata.Demoraw{10,i}(1,:)],32)); % Vz
    Navdata.Demo{11}(i) = IEEE754decript(hexToBinaryVector([Navdata.Demoraw{11,i}(4,:) Navdata.Demoraw{11,i}(3,:) Navdata.Demoraw{11,i}(2,:) Navdata.Demoraw{11,i}(1,:)],32)); % Vz

end
%% Decriptografar o RawMeasuresraw
for i = 1:size(Dados,2)
    for k = 1:20
        Navdata.RawMeasures{k}(i) = hex2dec([Navdata.RawMeasuresraw{k,i}(2,:) Navdata.RawMeasuresraw{k,i}(1,:)]); 
    end
end
%%
%% Decriptografar o TimeStampRaw
for i = 1:size(Dados,2)
    Navdata.TimeStamp{1}(i) = hex2dec([Navdata.TimeStampRaw{1,i}(2,:) Navdata.TimeStampRaw{1,i}(1,:)]);
    Navdata.TimeStamp{2}(i) = hex2dec([Navdata.TimeStampRaw{2,i}(2,:) Navdata.TimeStampRaw{2,i}(1,:)]);
    Navdata.TimeStamp{3}(i) = ardrone_time_to_usec(hex2dec(...
                             [Navdata.TimeStampRaw{3,i}(4,:) Navdata.TimeStampRaw{3,i}(3,:)...
                              Navdata.TimeStampRaw{3,i}(2,:) Navdata.TimeStampRaw{3,i}(1,:)]));
end
%%
% Decriptografar o Filtered MeasurementsRaw
for i = 1:size(Dados,2)
    Navdata.PhysicalMeasurements{1}(i) = IEEE754decript(hexToBinaryVector([Navdata.PhysicalMeasurementsRaw{1,i}(4,:) Navdata.PhysicalMeasurementsRaw{1,i}(3,:) Navdata.PhysicalMeasurementsRaw{8,i}(1,:) Navdata.PhysicalMeasurementsRaw{1,i}(1,:)],32)); % Yaw
    Navdata.PhysicalMeasurements{2}(i) = hex2dec([Navdata.PhysicalMeasurementsRaw{2,i}(1,:) Navdata.PhysicalMeasurementsRaw{2,i}(2,:)]);
    Navdata.PhysicalMeasurements{3}(i) = IEEE754decript(hexToBinaryVector([Navdata.PhysicalMeasurementsRaw{3,i}(4,:) Navdata.PhysicalMeasurementsRaw{3,i}(3,:) Navdata.PhysicalMeasurementsRaw{3,i}(2,:) Navdata.PhysicalMeasurementsRaw{3,i}(1,:)],32));
    Navdata.PhysicalMeasurements{4}(i) = IEEE754decript(hexToBinaryVector([Navdata.PhysicalMeasurementsRaw{4,i}(4,:) Navdata.PhysicalMeasurementsRaw{4,i}(3,:) Navdata.PhysicalMeasurementsRaw{4,i}(2,:) Navdata.PhysicalMeasurementsRaw{4,i}(1,:)],32));
    Navdata.PhysicalMeasurements{5}(i) = IEEE754decript(hexToBinaryVector([Navdata.PhysicalMeasurementsRaw{5,i}(4,:) Navdata.PhysicalMeasurementsRaw{5,i}(3,:) Navdata.PhysicalMeasurementsRaw{5,i}(2,:) Navdata.PhysicalMeasurementsRaw{5,i}(1,:)],32));
    for k = 6:8
        Navdata.PhysicalMeasurements{k}(i) = IEEE754decript(hexToBinaryVector([Navdata.PhysicalMeasurementsRaw{k,i}(4,:) Navdata.PhysicalMeasurementsRaw{k,i}(3,:) Navdata.PhysicalMeasurementsRaw{k,i}(2,:) Navdata.PhysicalMeasurementsRaw{k,i}(1,:)],32)); % Yaw
    end
end

%%
close
h = figure(1);
set(h,'units','pix','pos',[10 150 1500 600],'PaperPositionMode','auto');
grid on
% land = 700;
% set(gca,'Xlim',[0 land+20])
% line([land land],get(gca,'YLim'),'Color','k')
hold off
hold on
% plot(Navdata.Demo{1});
% plot(Navdata.Demo{2}-700);
% plot(Navdata.Demo{3}/100); % Pitch
% plot(Navdata.Demo{4}/100); % Roll
% plot(Navdata.Demo{5}/150000); % Yaw
% % 
% plot(Navdata.Demo{6});  % Vx
% plot(Navdata.Demo{7});  % Vy
plot(Navdata.Demo{10}); % Vz
plot(Navdata.Demo{11}); % Vz
% plot(Navdata.RawMeasures{1}); % Id
% plot(Navdata.RawMeasures{2}); % Size

% plot(Navdata.RawMeasures{3}-2000);
% plot(Navdata.RawMeasures{4}-2000);
% plot(Navdata.RawMeasures{5}-1550);

% plot(Navdata.RawMeasures{6});
% plot(Navdata.RawMeasures{7});
% plot(Navdata.RawMeasures{8});

% plot(Navdata.RawMeasures{9});
% plot(Navdata.RawMeasures{10});
% plot(Navdata.RawMeasures{11}/1000); % Bateria

% plot(Navdata.TimeStamp{1});
% plot(Navdata.TimeStamp{2});
% plot(Navdata.TimeStamp{3});

% plot(Navdata.PhysicalMeasurements{1}); % Acc Temp
% plot(Navdata.PhysicalMeasurements{2}); % Gyro Temp

% plot(Navdata.PhysicalMeasurements{3}); % Accx
% plot(Navdata.PhysicalMeasurements{4}); % Accy
% plot(Navdata.PhysicalMeasurements{5}/1000); % Accz
% 
% plot(Navdata.PhysicalMeasurements{6}); % Gyrox
% plot(Navdata.PhysicalMeasurements{7}); % Gyroy
% plot(Navdata.PhysicalMeasurements{8}); % Gyroz

%%


function tempo = ardrone_time_to_usec(temporaw)
TSECDEC = 21; % 4295000000
TUSECMASK = uint32(bitshift(1,TSECDEC)-1); % '00000000000111111111111111111111'
% negTUSECMASK = bitcmp(TUSECMASK,'uint32'); % '11111111111000000000000000000000'
tempo = bitshift(uint32(temporaw),-TSECDEC)*1000000 + uint32(bitand(uint32(temporaw),TUSECMASK,'uint32'));
end













