%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%

function  [small_l, small_w, small_v,Pos_l,Pos_w] = findFromBigBeam(beamPos_l, beamPos_w, W, small_beam, big_beam, map_l, map_w, map)
%在大波束中定位小波束的具体位置
[x,y] = size(map);
y_t = 1:y;
RW = find(y_t <= W/map_w, 1, 'last');
num = ceil(big_beam / small_beam); %大波束内包含的小波束个数
small_l =[];
small_w=[];
small_v=[];
Pos_l=0;
Pos_w=0;
for i = 1: num
    %index_l =  (beamPos_l - 1)*big_beam/map_l + i;
    %index_w = RW;
    smallBeamPos_l = (beamPos_l-1) * num + i;
    smallBeamPos_w = fix(RW*map_w/small_beam)+1;
    
    [hasObject, L, W, v] = smallBeamFindObject(smallBeamPos_l, smallBeamPos_w, map, small_beam, map_l, map_w);
    if hasObject
        small_l = [small_l L];
        small_w = [small_w W];
        small_v = [small_v v];
        Pos_l = smallBeamPos_l;
        Pos_w = smallBeamPos_w;
    end
    
end
end