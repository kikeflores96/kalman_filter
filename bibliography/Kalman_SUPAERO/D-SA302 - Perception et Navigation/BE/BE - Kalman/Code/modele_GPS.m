function [paramGPS] = modele_GPS(nom)

% [paramGPS] = modele_GPS(nom);
%--------------------------------------------------------------------------
% Mod?le GPS
%--------------------------------------------------------------------------
%   nom :   Nom du mod?le ? utiliser
%
%	paramGPS :	structure contenant les param?tres du mod?le
%--------------------------------------------------------------------------

if nargin==0
    nom = 'model_GPS_1';
end;

switch nom
    
    case 'model_GPS_1'
        %------------------------------------------------------------------
        % Mod?le GPS r?aliste
        %------------------------------------------------------------------

        % Param?tres
        paramGPS.dt         = 1.0;	% Pas d'?chantillonnage [s]
        paramGPS.sig_pos    = 2/3;	% Ecart-type du bruit [m]
        paramGPS.per_sau    = 30;	% P?riode moyenne entre sauts [s]
        paramGPS.amp_sau    = 10;	% Amplitude maximale du cumul des sauts [s]
    
    case 'model_GPS_2'
        %------------------------------------------------------------------
        % Mod?le GPS sans sauts
        %------------------------------------------------------------------

        % Param?tres
        paramGPS.dt         = 1.0;	% Pas d'?chantillonnage [s]
        paramGPS.sig_pos    = 2/3;	% Ecart-type du bruit [m]
        paramGPS.per_sau    = 30;	% P?riode moyenne entre sauts [s]
        paramGPS.amp_sau    = 0;	% Amplitude maximale du cumul des sauts [s]
        
    otherwise
        error('Modele GPS inconnu');
        
end;
