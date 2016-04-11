function [ big_l,big_w,small_l,small_w ] = gettheorylocation( l,w,small_beam,big_beam,map,map_l,map_w )
%GETTHEORYLOCATION Summary of this function goes here
%  获取理论物体所在处
[x,y]=size(map);
x_t=1:x;
y_t=1:y;
RL=find(x_t<=l/map_l, 1, 'last' );
RW=find(y_t<=w/map_w, 1, 'last' );
if isempty(RL)
    RL = 1;
end
if isempty(RW)
    RW = 1;
end
l =  RL*map_l;
w = RW*map_w;
big_l = ceil(l/big_beam);
big_w = ceil(w/big_beam);
small_l = ceil(l/small_beam);
small_w = ceil(w/small_beam);
end

