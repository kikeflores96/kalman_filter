%MM_FREE_SPACE  Liberate positions in the map.
%
%   MM_FREE_SPACE(S) will free the spaces in the map indicated by the
%   pointer S. After the call these spaces will be considered free, and
%   further calls to MM_QUERY_SPACE may return spaces pointed by S.
%
%   The input S should be a pointer returned by the function
%   MM_BLOCK_SPACE. Calling MM_FREE_SPACE with another argument than such
%   pointer may leave the map manager in a undefined state.
%
%   YOU WILL NOT NEED TO USE THIS FUNCTION IN THE BE. It exists as a dual
%   of MM_BLOCK_SPACE and may be used in future versions of the BE.
%
%   See also mm_block_space, mm_query_space, init_map_management.
%
