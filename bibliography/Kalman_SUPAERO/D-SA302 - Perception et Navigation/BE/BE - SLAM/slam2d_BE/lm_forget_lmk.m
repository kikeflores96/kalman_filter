%LM_FORGET_LMK  Forget a landmark
%
%   LM_FORGET_LMK(ID) will dissassociate the landmark identified by ID from
%   the pointer S which was previously associated with a call to
%   LM_ASSOCIATE_POINTER_TO_LMK(S, ID). Also, the pointer S and the id ID
%   won't be reported anymore to calls to LM_ALL_LMK_POINTERS and
%   LM_ALL_LMK_IDS, respectively.
%
%   A call to LM_FORGET_LMK(ID) effectively removes the landmark from the
%   map, although the memory it occupies is still blocked. You should call
%   MM_FREE_SPACE(S) later if you want the spaces pointed by S to be
%   available for further calls to MM_QUERY_SPACE. The pointer S can be
%   recovered with LM_LMK_POINTER(ID) but BEFORE the call to LM_FORGET_LMK.
%
%   There is no problem to forget a landmark that was not yet associated.
%
%   This function exists as a dual of LM_ASSOCIATE_POINTER_TO_LMK, but it
%   should not be used in the BE.
%
%   See also init_landmark_management, lm_associate_pointer_to_lmk,
%   lm_lmk_pointer, lm_all_lmk_pointers, lm_all_lmk_ids, mm_free_space,
%   mm_query_space.
%
