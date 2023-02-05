%LM_LMK_POINTER  Recover the landmark pointer
%
%   S = LM_LMK_POINTER(ID) returns the pointer S associated to a landmark
%   identified by ID. The pointer S and the ID must have been associated
%   before with a call to LM_ASSOCIATE_POINTER_TO_LMK(S, ID), otherwise the
%   pointer S returned by this function will be invalid and using may cause
%   your code to malfunction.
%
%   See also init_landmark_management, lm_associate_pointer_to_lmk.
%
