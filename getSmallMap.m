%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%
function small_map = getSmallMap(beam_l,beam_w,small_beam,map,map_l,map_w)
%»ñÈ¡Ğ¡²¨Êø¾ØÕó
num_l = small_beam/map_l;
num_w = small_beam/map_w;
start_l = (beam_l - 1)*num_l + 1;
start_w = (beam_w - 1)*num_w + 1;
small_map = map(start_l:beam_l*num_l, start_w:beam_w*num_w);
end