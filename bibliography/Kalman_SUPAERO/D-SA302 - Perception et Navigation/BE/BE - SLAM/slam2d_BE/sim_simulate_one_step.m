%SIM_SIMULATE_ONE_STEP  Advances the simulation state of one step.
%
%   SIM_SIMULATE_ONE_STEP should be called when you want to advance the
%   simulation state one step forward. This should happen once per
%   iteration of the temporal loop in the SLAM_2D_BE (you will find a call
%   to this function already in the right place). SIM_SIMULATE_ONE_STEP
%   will move the simulated robot one step forward using the fixed control
%   signal in the simulator modified by the control noise.
%
%   The control signal without noise can be recovered with
%   SIM_GET_CONTROL_SIGNAL. The noise applied to it is internal to the
%   simulator.
%
