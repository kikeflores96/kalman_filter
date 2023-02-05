function [] = plot_ESTIMATEUR(OUT_posENG,OUT_posEST,OUT_attENG,OUT_attEST)

% plot_ESTIMATEUR(OUT_posENG,OUT_posEST,OUT_attENG,OUT_attEST);
%--------------------------------------------------------------------------
% Tracés de l'état estimé du VEHICULE
%--------------------------------------------------------------------------

figure;

subplot(3,3,[1 8]);hold on; grid on;
plot(OUT_posEST.signals.values(:,1),OUT_posEST.signals.values(:,2),'b-','Linewidth',1.0);
plot(OUT_posENG.signals.values(:,1),OUT_posENG.signals.values(:,2),'r--','Linewidth',1.0);
axis equal
legend('Estimée','Vraie','Location','Best');
xlabel('x [m]');ylabel('y [m]');
title('Trajectoire');

INT_posENG=interp1(OUT_posENG.time,OUT_posENG.signals.values,OUT_posEST.time,'pchip');
INT_attENG=interp1(OUT_attENG.time,OUT_attENG.signals.values,OUT_attEST.time,'pchip');

subplot(3,3,3);hold on; grid on;
plot(OUT_posEST.time,OUT_posEST.signals.values(:,1)-INT_posENG(:,1),'Linewidth',1.5);
xlabel('t [s]');ylabel('x [m]');
title('ERREUR de POSITION [m]');

subplot(3,3,6);hold on; grid on;
plot(OUT_posEST.time,OUT_posEST.signals.values(:,2)-INT_posENG(:,2),'Linewidth',1.5);
xlabel('t [s]');ylabel('y [m]');

subplot(3,3,9);hold on; grid on;
plot(OUT_attEST.time,OUT_attEST.signals.values-INT_attENG,'Linewidth',1.5);
xlabel('t [s]');ylabel('\theta [°]');
title('ERREUR de CAP');

end

