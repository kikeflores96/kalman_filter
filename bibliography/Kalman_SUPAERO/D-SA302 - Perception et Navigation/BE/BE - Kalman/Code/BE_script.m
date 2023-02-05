%==========================================================================
%==========================================================================
% BE Navigation 
%==========================================================================
%==========================================================================

clearvars
close all

%--------------------------------------------------------------------------
% Mod?le de SIMULATION
%--------------------------------------------------------------------------

% Mod?le du VEHICULE
[paramENG,initENG] = modele_VEHICULE('model_ENG_1');

% Trajectoire de l'ENGIN (s?quence des consignes)
[IN_consVIT,IN_consPHI] = modele_TRAJECTOIRE('model_TRA_1');

% Mod?le des ODOMETRES
[paramODO] = modele_ODOMETRE('model_ODO_1');

% Mod?le de l'IMU
[paramIMU] = modele_IMU('model_IMU_1');

% Mod?le de GPS
[paramGPS] = modele_GPS('model_GPS_1');


%--------------------------------------------------------------------------
% Estimateur
%--------------------------------------------------------------------------

[paramEST,initEST] = modele_ESTIMATEUR('model_EST_1.2',paramENG,paramODO,paramIMU,paramGPS);


%--------------------------------------------------------------------------
% Simulation
%--------------------------------------------------------------------------

% Param?tres de simulation
paramSIM.T	= 180;       % Dur?e

% Simulation
sim('schemaSIM');


%--------------------------------------------------------------------------
% Trac?s
%--------------------------------------------------------------------------

% Consignes et r?ponses des actionneurs
plot_COMMANDE(OUT_consVIT,OUT_VIT,OUT_consPHI,OUT_PHI);

% Trajectoire du VEHICULE
plot_VEHICULE(OUT_posENG,OUT_attENG);

% Les vitesses
plot_VITESSE(OUT_consVIT,OUT_VIT,OUT_vitENG,OUT_vitROUE);

% Les mesures
plot_ODOMETRE(OUT_vitROUE,OUT_vitODO);
plot_IMU(OUT_omeENG,OUT_omeIMU);
plot_GPS(OUT_posENG,OUT_posGPS);

% Les estimations
plot_ESTIMATEUR(OUT_posENG,OUT_posEST,OUT_attENG,OUT_attEST);
