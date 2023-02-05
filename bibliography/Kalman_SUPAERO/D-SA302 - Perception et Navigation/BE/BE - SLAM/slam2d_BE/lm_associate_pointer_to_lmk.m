%LM_ASSOCIATE_POINTER_TO_LMK  Associate a pointer to a landmark
%
%   LM_ASSOCIATE_POINTER_TO_LMK(S, ID) associates the space pointed by S in
%   the map to a landmark identified by ID. In simpler words, the
%   association means "from now on the landmark ID can be found in the map
%   in the spaces pointed by S", effectively "putting" the landmark into
%   the map.
%
%   After the association the pointer S can be recovered with a call to
%   LM_LMK_POINTER(ID). A landmark can be dissociated of a pointer with a
%   call to LM_FORGET_LMK(ID), although this is not needed on this BE. The
%   pointer S will also be reported by a call to LM_ALL_LMK_POINTERS, and
%   likewise ID will be reported in a call to LM_ALL_LMK_IDS.
%
%   The pointer S should not be associated to other landmark, and similarly
%   a landmark ID should not be associated to other pointer. Doing so will
%   make landmark management malfunction. Don't forget to block the space
%   associated to the landmark with a call to MM_BLOCK_SPACE(S) so it won't
%   be reported as available in future calls to MM_QUERY_SPACE.
%
%   See also init_landmark_management, lm_lmk_pointer, lm_forget_lmk,
%   lm_all_lmk_pointers, lm_all_lmk_ids, mm_block_space, mm_query_space.
%
