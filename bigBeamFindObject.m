 %%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%

function [hasObject, L, W, v,map_index_w] = bigBeamFindObject(beamPos_l,beamPos_w , map, big_beam, map_l, map_w)
big_num_l = big_beam / map_l;
big_num_w = big_beam / map_w;
hasObject = 0;
L = 0;
W = 0;
v = 0;
map_index_w=0;
for i = 1:big_num_l
    for j = 1: big_num_w
        index_l = (beamPos_l-1)*big_beam/map_l + i;
        index_w = (beamPos_w-1)*big_beam/map_w + j;
        if(map(index_l,index_w) ~= 0)
            hasObject = 1;
            %获取的位置取波束中心
            L = (beamPos_l-1)*big_beam + 0.5*big_beam;
            W = index_w*map_w;
            v = map(index_l,index_w);
            map_index_w = index_w;
        end
    end    
end
end 