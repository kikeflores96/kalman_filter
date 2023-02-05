%MM_BLOCK_SPACE  Block positions in the map.
%
%   MM_BLOCK_SPACE(S) will block the spaces in the map indicated by the
%   pointer S. After the call these spaces will be considered occupied, and
%   further calls to MM_QUERY_SPACE will not return these spaces until they
%   are liberated with a call to MM_FREE_SPACE(S).
%
%   The input S should be a pointer returned by the function
%   MM_QUERY_SPACE. Calling MM_BLOCK_SPACE with another argument than such
%   pointer may leave the map manager in a undefined state.
%
%   See also mm_free_space, mm_query_space, init_map_management.
%
