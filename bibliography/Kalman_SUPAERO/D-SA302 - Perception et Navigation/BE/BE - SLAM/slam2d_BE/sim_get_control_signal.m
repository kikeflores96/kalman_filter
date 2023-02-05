%SIM_GET_CONTROL_SIGNAL  Returns the control signal used in the simulator
%
%   U = SIM_GET_CONTROL_SIGNAL return the control signal U used to move the
%   simulated robot inside the simulator. U is a column vector in the form
%   [DX; DA], where DX is a forward motion and DA is the angle of rotation.
%
%   Note that U is the control signal WITHOUT THE NOISE. Like in the real
%   world you don't have access to the control noise of the system. You
%   will need to find ways to estimate the control noise to be used in the
%   estimator you will code. How would you do if it was a real robot?
%
