function big_map = getBigMap(beam_l,beam_w,big_beam,map,map_l,map_w)
%»ñÈ¡´ó²¨Êø¾ØÕó
num_l = big_beam/map_l;
num_w = big_beam/map_w;
start_l = (beam_l - 1)*num_l + 1;
start_w = (beam_w - 1)*num_w + 1;
big_map = map(start_l:beam_l*num_l, start_w:beam_w*num_w);
end