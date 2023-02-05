%INIT_LANDMARK_MANAGEMENT  Setup the landmark management
%
%   INIT_LANDMARK_MANAGEMENT should be called ONLY ONCE in the code and
%   BEFORE any call to lm_* functions.
%
%   The call will set up the variables needed for landmark management. The
%   lm_* functions will only work properly after the call to this function.
%
%   See also lm_all_lmk_ids, lm_all_lmk_pointers,
%   lm_associate_pointer_to_lmk, lm_find_non_mapped_lmk, lm_forget_lmk,
%   lm_lmk_pointer.
%
