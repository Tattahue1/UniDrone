clc
clear all
close all

Ts = 0.01;   % Tiempo de muestreo
NIT = 1600; % número de interacciones+
%Yr(1:800) = 0.4; Yr(801:NIT) = 0.1; %Sebas
%Yr(1:800) = 0.1; Yr(801:1600) = 0.3; %Jhai 
%Yr(1:800) = 0.3; Yr(801:1600) = 0.2; %Ramiro
Yr(1:800) = 0.1; Yr(801:1600) = 0.4; %Brandon
distancia(1:NIT) = 0;
%Yr(1:NIT) = 0.2; %+ Referencia
%do(1:500) = 0; do(501:599) = 0; do(600:NIT)=0;% perturbación salida Sebas
%di(1:1200) = 0; di(1201:1350) = 0.01; di(1351:NIT) = 0.01;%perturbacion entrada Sebas
%do(1:500) = 0; do(501:599) = 0.03; do(600:NIT)=0.03;% perturbación salida Jhai
%di(1:1200) = 0; di(1201:1350) = 0; di(1351:NIT) = 0;%perturbacion entrada Jhai
%do(1:500) = 0; do(501:599) = 0; do(600:NIT)=0;% perturbación salida Ramiro
%di(1:1200) = 0; di(1201:1350) = 0.02; di(1351:NIT) = 0.02;%perturbacion entrada Ramiro
do(1:500) = 0; do(501:599) = 0.01; do(600:NIT)=0.01;% perturbación salida Brandon 
di(1:1200) = 0; di(1201:1350) = 0; di(1351:NIT) = 0;%perturbacion entrada Brandon
Yp(1:10) = 0; Y(1:10)= 0 ; % salidas del sistema
e(1:10) = 0; % error
up(1:10) = 0; % errorcontrol
u(1:10) = 0; % control
%x= 1:NIT;
umax=0.7; umin=0.5; %limites saturador
%curve= animatedline('Color','k');
%set(gca,'Xlim',[1 NIT],'Ylim',[0 5]);
%grid on
puertoSerial = arduino('COM5', 'Uno', 'Libraries', {'Ultrasonic', 'Servo'});
sensor = ultrasonic(puertoSerial,'D3','D2');
brushless = 'D6';

ESC = servo(puertoSerial,brushless,'MaxPulseDuration',2e-3,'MinPulseDuration',1e-3) ;
writePosition(ESC,0)
pause(3);

%puertoSerial = arduino('COM5');+
%pinMode(puertoSerial,6,'output');

tic
for k = 2 : NIT
   distancia(k)=readDistance(sensor) -0.09;
    if(distancia(k) >= 0.50)
        distancia(k) = distancia(k-1);
    end
   Yp(k) = distancia(k);%aprox 0.6
   
   Y(k) = Yp(k) + do(k); 
   
   e(k) = Yr(k) - Y(k); 
   
   up(k) = up(k-1) + 1.4286621072*e(k) - 1.3762159408*e(k-1); %estesi
   %up(k) = up(k-1) + 0.428598632*e(k) -0.412864782*e(k-1);
   %up(k) = up(k-1) + 2.142993162*e(k) -2.064323912*e(k-1); %estetalvez*4
   %up(k) = up(k-1) + 0.107149658*e(k) -0.103216196*e(k-1); %conservativo*80
   
   u(k)=up(k)+ di(k);
   
   if (u(k)>=umax)
       u(k)=umax;
   elseif (u(k)<=umin)
           u(k)=umin;
   end
   writePosition(ESC,u(k));
   pause(Ts);   
end
tiempo=toc;
for(i=1:100)
    writePosition(ESC,0.5);
end
writePosition(ESC,0);
%%fprintf('El proceso ha tardado %f segundos', tiempo)
%t=(1:NIT); % interacciones
t=(0:Ts:(NIT-1)*Ts); % segundos 
figure(1);
subplot(211),plot(t,Yr,'--k',t,Y,'r','lineWidth',2),xlabel('tiempo(Segundos)'),ylabel('salida(dm)');
legend('referencia', 'salida');
grid on;
subplot(212),plot(t,u,'b'),xlabel('tiempo(Segundos)'),ylabel('control(señal PWM)');
legend('control');
grid on;
%GraficaFt(t,Y,tiempo,NIT);