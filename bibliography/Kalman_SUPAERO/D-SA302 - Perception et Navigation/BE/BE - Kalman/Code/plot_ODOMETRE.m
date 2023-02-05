function [] = plot_ODOMETRE(OUT_vitROUE,OUT_vitODO,OUT_depROUE,OUT_depODO)

% plot_ODOMETRE(OUT_vitROUE,OUT_vitODO,OUT_depROUE,OUT_depODO);
%--------------------------------------------------------------------------
% Tracés des vitesses (et déplacements) mesurées par ODOMETRES
%--------------------------------------------------------------------------

figure;

%--------------------------------------------------------------------------
% Mesure uniquement de la vitesse 
%--------------------------------------------------------------------------
if nargin==2
    
    % Roue Arrière Gauche
    k=1;
    subplot(4,1,k);hold on; grid on;
    plot(OUT_vitODO.time,OUT_vitODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitROUE.time,OUT_vitROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('VITESSE roue ar. g.');
    
    % Roue Arrière Droite
    k=2;
    subplot(4,1,k);hold on; grid on;
    plot(OUT_vitODO.time,OUT_vitODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitROUE.time,OUT_vitROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('VITESSE roue ar. d.');
    
    % Roue Avant Gauche
    k=3;
    subplot(4,1,k);hold on; grid on;
    plot(OUT_vitODO.time,OUT_vitODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitROUE.time,OUT_vitROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('VITESSE roue av. g.');
    
    % Roue Avant Droite
    k=4;
    subplot(4,1,k);hold on; grid on;
    plot(OUT_vitODO.time,OUT_vitODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitROUE.time,OUT_vitROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('VITESSE roue av. d.');

end;

%--------------------------------------------------------------------------
% Mesure de la vitesse et du déplacement
%--------------------------------------------------------------------------
if nargin==4
    
    % Roue Arrière Gauche
    k=1;
    subplot(4,2,2*k-1);hold on; grid on;
    plot(OUT_vitODO.time,OUT_vitODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitROUE.time,OUT_vitROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('VITESSE roue ar. g.');
    subplot(4,2,2*k);hold on; grid on;
    plot(OUT_depODO.time,OUT_depODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_depROUE.time,OUT_depROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('DEPLACEMENT roue ar. g.');
    
    % Roue Arrière Droite
    k=2;
    subplot(4,2,2*k-1);hold on; grid on;
    plot(OUT_vitODO.time,OUT_vitODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitROUE.time,OUT_vitROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('VITESSE roue ar. d.');
    subplot(4,2,2*k);hold on; grid on;
    plot(OUT_depODO.time,OUT_depODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_depROUE.time,OUT_depROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('DEPLACEMENT roue ar. d.');
    
    % Roue Avant Gauche
    k=3;
    subplot(4,2,2*k-1);hold on; grid on;
    plot(OUT_vitODO.time,OUT_vitODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitROUE.time,OUT_vitROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('VITESSE roue av. g.');
    subplot(4,2,2*k);hold on; grid on;
    plot(OUT_depODO.time,OUT_depODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_depROUE.time,OUT_depROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('DEPLACEMENT roue av. g.');
    
    % Roue Avant Droite
    k=4;
    subplot(4,2,2*k-1);hold on; grid on;
    plot(OUT_vitODO.time,OUT_vitODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_vitROUE.time,OUT_vitROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('VITESSE roue av. d.');
    subplot(4,2,2*k);hold on; grid on;
    plot(OUT_depODO.time,OUT_depODO.signals.values(:,k),'b-','Linewidth',1);
    plot(OUT_depROUE.time,OUT_depROUE.signals.values(:,k),'r--','Linewidth',1);
    legend('Odomètre','Vraie','Location','Best');
    title('DEPLACEMENT roue av. d.');
    
end;

end

