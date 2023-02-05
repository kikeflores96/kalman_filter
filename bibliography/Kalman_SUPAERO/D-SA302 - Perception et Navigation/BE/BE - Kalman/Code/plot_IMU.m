function [] = plot_IMU(OUT_omeENG,OUT_omeIMU,OUT_attENG,OUT_intomeIMU)

% plot_IMU(OUT_omeENG,OUT_omeIMU,OUT_attENG,OUT_intomeIMU);
%--------------------------------------------------------------------------
% Tracés des vitesses (et angles) mesurées par GYROMETRES
%--------------------------------------------------------------------------

figure;

%--------------------------------------------------------------------------
% Vitesse 
%--------------------------------------------------------------------------
if nargin==2
    
    k=1;
    subplot(2,1,k);hold on; grid on;
    plot(OUT_omeIMU.time,OUT_omeIMU.signals.values*180/pi,'b-','Linewidth',1);
    plot(OUT_omeENG.time,OUT_omeENG.signals.values*180/pi,'r--','Linewidth',1);
    legend('Gyromètre','Vraie','Location','Best');
    title('VITESSE de rotation [°/s]');
    
    k=2;
    subplot(2,1,k);hold on; grid on;
    INT_omeENG=interp1(OUT_omeENG.time,OUT_omeENG.signals.values,OUT_omeIMU.time,'pchip');
    plot(OUT_omeIMU.time,(OUT_omeIMU.signals.values-INT_omeENG)*180/pi,'Linewidth',1.5);
    legend('Gyromètre-Vraie','Location','Best');
    title('ERREUR de VITESSE de rotation [°/s]');

end;

%--------------------------------------------------------------------------
% Vitesse et angle
%--------------------------------------------------------------------------
if nargin==4
    
    k=1;
    subplot(2,1,k);hold on; grid on;
    plot(OUT_omeIMU.time,OUT_omeIMU.signals.values*180/pi,'b-','Linewidth',1);
    plot(OUT_omeENG.time,OUT_omeENG.signals.values*180/pi,'r--','Linewidth',1);
    legend('Gyromètre','Vraie','Location','Best');
    title('VITESSE de rotation [°/s]');
    
    k=2;
    subplot(2,1,k);hold on; grid on;
    plot(OUT_intomeIMU.time,OUT_intomeIMU.signals.values*180/pi,'b-','Linewidth',1);
    plot(OUT_attENG.time,OUT_attENG.signals.values*180/pi,'r--','Linewidth',1);
    legend('Intégration gyromètre','Vraie','Location','Best');
    title('ORIENTATION [°]');
    
end;