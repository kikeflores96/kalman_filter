function [] = plot_VEHICULE(OUT_posENG,OUT_attENG)

% plot_VEHICULE(OUT_posENG,OUT_attENG);
%--------------------------------------------------------------------------
% Tracés de l'état du VEHICULE
%--------------------------------------------------------------------------

figure;

subplot(3,3,[1 8]);hold on; grid on;
plot(OUT_posENG.signals.values(:,1),OUT_posENG.signals.values(:,2),'b-','Linewidth',1.0);
axis equal
xlabel('x [m]');ylabel('y [m]');
title('Trajectoire');

subplot(3,3,3);hold on; grid on;
plot(OUT_posENG.time,OUT_posENG.signals.values(:,1),'b-','Linewidth',1.0);
xlabel('t [s]');ylabel('x [m]');
title('Position');

subplot(3,3,6);hold on; grid on;
plot(OUT_posENG.time,OUT_posENG.signals.values(:,2),'b-','Linewidth',1.0);
xlabel('t [s]');ylabel('y [m]');

subplot(3,3,9);hold on; grid on;
plot(OUT_attENG.time,OUT_attENG.signals.values*180/pi,'b-','Linewidth',1.0);
xlabel('t [s]');ylabel('\theta [°]');
title('Cap');

end

