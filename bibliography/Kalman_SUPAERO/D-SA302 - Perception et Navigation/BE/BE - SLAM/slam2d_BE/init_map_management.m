%INIT_MAP_MANAGEMENT  Setup the map management
%
%   INIT_MAP_MANAGEMENT(MAPSIZE) should be called ONLY ONCE in the code and
%   BEFORE any call to mm_* functions. MAPSIZE should contain the size of
%   the map in the estimator.
%
%   The call will set up the variables needed for map management. The mm_*
%   functions will only work properly AFTER the call to this function.
%
%   See also mm_block_space, mm_free_space, mm_query_space.
%
