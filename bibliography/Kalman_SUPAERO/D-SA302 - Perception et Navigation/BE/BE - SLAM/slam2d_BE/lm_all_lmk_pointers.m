%LM_ALL_LMK_POINTERS  Recover pointers to all known landmarks
%
%   PTRS = LM_ALL_LMK_POINTERS will return the row vector PTRS that is the
%   concatenation of the pointers of all landmarks known by the estimation,
%   that is all pointers that were associated to landmarks with calls to
%   LM_ASSOCIATE_POINTER_TO_LMK.
%
%   This is useful when we want to get pointers to the whole map.
%
%   See also init_landmark_management, lm_associate_pointer_to_lmk.
%
