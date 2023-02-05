function [y, Y_r, Y_p] = observe(r, p)
%   OBSERVE transform a position in map frame 
%   to a range-and-bearing measurement.
%   
%   In: 
%       r :     robot frame     r = [r_x ; r_y ; r_alpha]
%       p :     point in global frame p = [p_x ; p_y]
%   Out:
%       y :     measurment      y = [range ; bearing]
%       Y_r:    Jacobian wrt r
%       Y_p:    Jacobian wrt p

if nargout == 1 % No Jacobians requested
    
    y   = scan(toFrame(r,p));

else %  Jacobians requested
    
    [pr, PR_r, PR_p]    = toFrame(r,p);
    [y, Y_pr] = scan(pr);
    
    % here the chain rule !
    Y_r = Y_pr * PR_r;
    Y_p = Y_pr * PR_p;
        
end
end