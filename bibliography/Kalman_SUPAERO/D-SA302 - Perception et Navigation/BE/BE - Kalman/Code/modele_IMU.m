function [paramIMU] = modele_IMU(nom)

% [paramIMU] = modele_IMU(nom);
%--------------------------------------------------------------------------
% Mod�le des CAPTEURS INERTIELS
%--------------------------------------------------------------------------
%   nom :   Nom du mod�le � utiliser
%
%	paramIMU :	structure contenant les param�tres du mod�le
%--------------------------------------------------------------------------

if nargin==0
    nom = 'model_IMU_1';
end;

switch nom
    
    case 'model_IMU_1'
        %------------------------------------------------------------------
        % Mesure gyrom�trique r�aliste
        %------------------------------------------------------------------
        
        % Param�tres
        paramIMU.dt         = 0.01; % Pas d'�chantillonnage [s]
        paramIMU.sig_ome    = 0.1;	% Ecart-type du bruit [�/s]
        paramIMU.biais_ome	= 5.0;	% Amplitude maximale du biais [�/s]
        paramIMU.deriv_ome	= 0.05;	% D�rive maximale du biais [�/s /s]
        
    case 'model_IMU_2'
        %------------------------------------------------------------------
        % Mesure gyrom�trique sans biais
        %------------------------------------------------------------------
        
        % Param�tres
        paramIMU.dt         = 0.01; % Pas d'�chantillonnage [s]
        paramIMU.sig_ome    = 0.1;	% Ecart-type du bruit [�/s]
        paramIMU.biais_ome	= 0.0;	% Amplitude maximale du biais [�/s]
        paramIMU.deriv_ome	= 0.05;	% D�rive maximale du biais [�/s /s]

    otherwise
        error('Modele IMU inconnu');
        
end;
