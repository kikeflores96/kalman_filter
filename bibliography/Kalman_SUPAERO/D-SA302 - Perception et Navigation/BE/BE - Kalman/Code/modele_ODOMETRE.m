function [paramODO] = modele_ODOMETRE(nom)

% [paramODO] = modele_ODOMETRE(nom);
%--------------------------------------------------------------------------
% Modèle des ODOMETRES
%--------------------------------------------------------------------------
%   nom :   Nom du modèle à utiliser
%
%	paramODO :	structure contenant les paramètres du modèle
%--------------------------------------------------------------------------

if nargin==0
    nom = 'model_ODO_1';
end;

switch nom
    
    case 'model_ODO_1'
        %------------------------------------------------------------------
        % Modèle d'odomètre réaliste
        %------------------------------------------------------------------
        
        % Paramètres
        paramODO.dt         = 0.01; % Pas d'échantillonnage [s]
        paramODO.dx         = 0.06; % Déplacement inter-tops [m]
        paramODO.sig_vit	= 1;    % Ecart-type du bruit [m/s]
        
    otherwise
        error('Modele ODOMETRE inconnu');
        
end;
