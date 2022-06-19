clc
clear all
close all

Ts = 0.01;   % Tiempo de muestreo
NIT = 2400; % número de interacciones+
%Yr(1:2000) = 0.2; Yr(601:NIT) = 0.25;
distancia(1:NIT) = 0;
Yr(1:NIT) = 0.2; %+ Referencia
do(1:600) = 0; do(601:699) = 0; do(700:NIT)=0;% perturbación
Yp(1:10) = 0; Y(1:10)= 0 ; % salidas del sistema
e(1:10) = 0; % error
u(1:10) = 0; % control
%x= 1:NIT;
umax=0.7; umin=0.1; %limites saturador
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
    if(distancia(k) >= 0.35)
        distancia(k) = distancia(k-1);
    end
   % aux=((1650 - distancia(k)*300)-1000)/1000
    aux = 0.6;
    if(distancia(k) <= 0.2)
        aux = 0.62 + (distancia(k)-0.2)*0.2;
    end
    if(distancia(k) > 0.2)
        aux = 0.59 + (distancia(k)-0.2)*0.1;
    end
   Yp(k) = distancia(k);%aprox 0.6
   
   Y(k) = Yp(k) + do(k); 
   
   e(k) = Yr(k) - Y(k); 
   
  %u(k) = u(k-1) + 0.99*e(k) - 0.9*e(k-1);
  %[6.1995 -5.44192]/z-1 u(k)*(1) = 6.1995(k-1) + u(k-1) 
   u(k) = u(k-1) + 5.260963954349999*e(k) - 5.06783403965*e(k-1);
   if (u(k)>=umax)
       u(k)=umax;
   elseif (u(k)<=umin)
           u(k)=umin;
   end
   if(u(k) > 1)
      u(k) = 0.9;
   end
%   u(k-1)
%   e(k)
%   e(k-1)
%   Y(k)
%   Yp(k)
%   u(k)
   writePosition(ESC,u(k));%*0.1688713763)
%   enviar_dato(u(k),10,puertoSerial);
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
subplot(211),plot(t,Yr,'--k',t,Y,'r','lineWidth',2),xlabel('tiempo(Segundos)'),ylabel('salida(Volts)');
legend('referencia', 'salida');
grid on;
subplot(212),plot(t,u,'b'),xlabel('tiempo(Segundos)'),ylabel('control(señal PWM)');
legend('control');
grid on;
%GraficaFt(t,Y,tiempo,NIT);