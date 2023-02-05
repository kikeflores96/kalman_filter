function [paramIMU] = modele_IMU(nom)

% [paramIMU] = modele_IMU(nom);
%--------------------------------------------------------------------------
% Modèle des CAPTEURS INERTIELS
%--------------------------------------------------------------------------
%   nom :   Nom du modèle à utiliser
%
%	paramIMU :	structure contenant les paramètres du modèle
%--------------------------------------------------------------------------

if nargin==0
    nom = 'model_IMU_1';
end;

switch nom
    
    case 'model_IMU_1'
        %------------------------------------------------------------------
        % Mesure gyrométrique réaliste
        %------------------------------------------------------------------
        
        % Paramètres
        paramIMU.dt         = 0.01; % Pas d'échantillonnage [s]
        paramIMU.sig_ome    = 0.1;	% Ecart-type du bruit [°/s]
        paramIMU.biais_ome	= 5.0;	% Amplitude maximale du biais [°/s]
        paramIMU.deriv_ome	= 0.05;	% Dérive maximale du biais [°/s /s]
        
    case 'model_IMU_2'
        %------------------------------------------------------------------
        % Mesure gyrométrique sans biais
        %------------------------------------------------------------------
        
        % Paramètres
        paramIMU.dt         = 0.01; % Pas d'échantillonnage [s]
        paramIMU.sig_ome    = 0.1;	% Ecart-type du bruit [°/s]
        paramIMU.biais_ome	= 0.0;	% Amplitude maximale du biais [°/s]
        paramIMU.deriv_ome	= 0.05;	% Dérive maximale du biais [°/s /s]

    otherwise
        error('Modele IMU inconnu');
        
end;
