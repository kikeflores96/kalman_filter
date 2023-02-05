%SIM_GET_LMK_MEASUREMENT  Returns a measurement to a landmark
%
%   Y = SIM_GET_LMK_MEASUREMENT(I) returns the measurement Y of a landmark
%   I measured from the current position and orientation of the simulated
%   robot. The input I should be a number that identifies the landmark (its
%   id). The output measurement Y is a column vector [D; A], with D the
%   distance from the robot to the landmark, and A the bearing angle from
%   the robot's x-axis.
%
