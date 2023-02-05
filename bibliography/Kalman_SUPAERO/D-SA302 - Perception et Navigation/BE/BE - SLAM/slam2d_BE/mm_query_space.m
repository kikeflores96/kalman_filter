%MM_QUERY_SPACE  Ask the map manager for space.
%
%   S = MM_QUERY_SPACE(N) returns a pointer S to N available spaces in the
%   map. For instance, if you want to know if there is space to hold the
%   robot state, which has size 3, you should use S = MM_QUERY_SPACE(3).
%
%   The length of S will be N if there are at least N spaces available, and
%   less than N if there are less than N spaces available. You should
%   always check the size of S with length(S) to see if there is enough
%   space to hold your variable.
%
%   Note that this DO NOT block the space in the map. You must use
%   mm_block_space(S) to block the space pointed by S.
%
%   See also mm_block_space, init_map_management.
%
