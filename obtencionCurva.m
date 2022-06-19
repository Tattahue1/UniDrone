clc
clear all
close all

ComSerial = arduino('COM5', 'Uno', 'Libraries', {'Ultrasonic', 'Servo'});
sensor = ultrasonic(ComSerial,'D3','D2');
brushless = 'D6';

ESC = servo(ComSerial,brushless,'MaxPulseDuration',2e-3,'MinPulseDuration',1e-3) ;
writePosition(ESC,0)
pause(3);

NIT=1200;
ts=1/10;
distancia = zeros(1,NIT);
iteraciones = 1:NIT;
%Y=zeros(1,NIT);
%x= 1:NIT;
%Codigo para graficar
curve= animatedline('Color','k');
set(gca,'Xlim',[1 NIT],'Ylim',[0 0.8]);
grid on
disp('iniciando');
pause(1);
%Se envia el voltaje pero antes se hace la conversión necesaria
%voltaje=(voltaje/5)*255;
tic

for k=1:NIT
    distancia(k)=readDistance(sensor) -0.09;
    if(distancia(k) >= 0.3)
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
    writePosition(ESC,aux)
    addpoints(curve,iteraciones(k),distancia(k));
    drawnow limitrate
    pause(0.01)
end
t=toc
writePosition(ESC,0);
iteraciones=iteraciones*t;
    iteraciones=iteraciones/NIT;
    figure(1);
    plot(iteraciones,distancia)
    title('LECTURA ANALOGICA CON ARDUINO');
    xlabel('Tiempo(Segundos)');
    ylabel('Distancia(metros)');
    ylim([-0.1 0.5]);
    xlim([0 t+0.5]);
%fprinf('El proceso ha tardado %f segundos', tiempo)
%graficaFt (x, Y, tiempo, NIT);
%Si no se ejecuta esta instruccion los motores seguiran girando aun
%después de terminar el proceso