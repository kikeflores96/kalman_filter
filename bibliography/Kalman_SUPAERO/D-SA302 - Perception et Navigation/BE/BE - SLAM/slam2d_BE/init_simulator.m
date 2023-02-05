%INIT_SIMULATOR  Setup the simulator module
%
%   INIT_SIMULATOR should be called ONLY ONCE in the code. It will setup
%   all variables necessary for the simulation to work. The sim_* functions
%   will only work properly AFTER the call to this function.
%
%   See also sim_get_control_signal, sim_get_initial_robot_pose,
%   sim_get_lmk_measurement, sim_simulate_one_step.
%
