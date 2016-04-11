%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%

function [hasObject, L, W, v] = smallBeamFindObject(smallBeamPos_l, smallBeamPos_w, map, small_beam, map_l, map_w)
small_num_l = small_beam / map_l;
small_num_w = small_beam / map_w;
hasObject = 0;
L = 0;
W = 0;
v = 0;
for i = 1 : small_num_l
    for j = 1 : small_num_w
        index_l = fix((smallBeamPos_l - 1)*small_beam/map_l + i);
        index_w = fix((smallBeamPos_w -1)*small_beam/map_w + j);
        if(map(index_l,index_w) ~= 0)
            hasObject = 1;
            L = (smallBeamPos_l-1)*small_beam + 0.5*small_beam;
            W = index_w*map_w;
            v = map(index_l,index_w);
        end
    end
end
end