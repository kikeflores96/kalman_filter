function [] = plot_GPS(OUT_posENG,OUT_posGPS,OUT_vitENG,OUT_vitGPS)

% plot_GPS(OUT_posENG,OUT_posGPS,OUT_vitENG,OUT_vitGPS);
%--------------------------------------------------------------------------
% Tracés des positions (et vitesses) mesurées par GPS
%--------------------------------------------------------------------------

figure;

%--------------------------------------------------------------------------
% Position 
%--------------------------------------------------------------------------
if nargin==2
    
    k=1;
    subplot(3,1,k);hold on; grid on;
    plot(OUT_posGPS.time,OUT_posGPS.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_posENG.time,OUT_posENG.signals.values(:,k),'r--','Linewidth',1);
    legend('GPS','Vraie','Location','Best');
    title('POSITION x [m]');
    
    k=2;
    subplot(3,1,k);hold on; grid on;
    plot(OUT_posGPS.time,OUT_posGPS.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_posENG.time,OUT_posENG.signals.values(:,k),'r--','Linewidth',1);
    legend('GPS','Vraie','Location','Best');
    title('POSITION y [m]');
    
    k=3;
    subplot(3,1,k);hold on; grid on;
    INT_posENG=interp1(OUT_posENG.time,OUT_posENG.signals.values,OUT_posGPS.time,'pchip');
    plot(OUT_posGPS.time,OUT_posGPS.signals.values-INT_posENG,'Linewidth',1.5);
    legend('xGPS - xVRAI','yGPS - yVRAI','Location','Best');
    title('ERREUR de POSITION [m]');
    
end;

%--------------------------------------------------------------------------
% Vitesse et angle
%--------------------------------------------------------------------------
if nargin==4
    
    k=1;
    subplot(2,2,2*k-1);hold on; grid on;
    plot(OUT_posGPS.time,OUT_posGPS.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_posENG.time,OUT_posENG.signals.values(:,k),'r--','Linewidth',1);
    legend('GPS','Vraie','Location','Best');
    title('POSITION x [m]');
    subplot(2,2,2*k);hold on; grid on;
    plot(OUT_vitGPS.time,OUT_vitGPS.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitENG.time,OUT_vitENG.signals.values(:,k),'r--','Linewidth',1);
    legend('GPS','Vraie','Location','Best');
    title('VITESSE x [m/s]');
    
    k=2;
    subplot(2,2,2*k-1);hold on; grid on;
    plot(OUT_posGPS.time,OUT_posGPS.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_posENG.time,OUT_posENG.signals.values(:,k),'r--','Linewidth',1);
    legend('GPS','Vraie','Location','Best');
    title('POSITION y [m]');
    subplot(2,2,2*k);hold on; grid on;
    plot(OUT_vitGPS.time,OUT_vitGPS.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitENG.time,OUT_vitENG.signals.values(:,k),'r--','Linewidth',1);
    legend('GPS','Vraie','Location','Best');
    title('VITESSE y [m/s]');
    
    
end;