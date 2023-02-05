function [] = plot_VITESSE(OUT_consVIT,OUT_VIT,OUT_vitENG,OUT_vitROUE)

% plot_VITESSE(OUT_consVIT,OUT_VIT,OUT_vitENG,OUT_vitROUE);
%--------------------------------------------------------------------------
% Tracés des vitesses
%--------------------------------------------------------------------------

vitENG = OUT_vitENG.signals.values;
vitENG = sqrt(sum(vitENG.*vitENG,2));

figure;

hold on; grid on;
plot(OUT_consVIT.time,OUT_consVIT.signals.values,'b--','Linewidth',1.0);
plot(OUT_VIT.time,OUT_VIT.signals.values,'b-','Linewidth',1.5);
plot(OUT_vitENG.time,vitENG,'r--','Linewidth',1.5);
plot(OUT_vitROUE.time,OUT_vitROUE.signals.values,'--','Linewidth',1);
legend('Consigne','V','Véhicule','Roue ar. g.;','Roue ar. d.','Roue av. g.','Roue av. d.','Location','Best');
title('Vitesse des roues');

end

