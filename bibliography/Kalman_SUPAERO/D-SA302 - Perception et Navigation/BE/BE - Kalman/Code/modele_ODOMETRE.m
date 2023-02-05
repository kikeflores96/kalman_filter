function [paramODO] = modele_ODOMETRE(nom)

% [paramODO] = modele_ODOMETRE(nom);
%--------------------------------------------------------------------------
% Mod�le des ODOMETRES
%--------------------------------------------------------------------------
%   nom :   Nom du mod�le � utiliser
%
%	paramODO :	structure contenant les param�tres du mod�le
%--------------------------------------------------------------------------

if nargin==0
    nom = 'model_ODO_1';
end;

switch nom
    
    case 'model_ODO_1'
        %------------------------------------------------------------------
        % Mod�le d'odom�tre r�aliste
        %------------------------------------------------------------------
        
        % Param�tres
        paramODO.dt         = 0.01; % Pas d'�chantillonnage [s]
        paramODO.dx         = 0.06; % D�placement inter-tops [m]
        paramODO.sig_vit	= 1;    % Ecart-type du bruit [m/s]
        
    otherwise
        error('Modele ODOMETRE inconnu');
        
end;
