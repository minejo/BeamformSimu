%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%
function [ Pos_l, Pos_w ] = getBigPosition( L,W,big_beam )
%GETBIGPOSITION Summary of this function goes here
%  根据物体的实际位置判断对应的大波束的坐标
Pos_l = ceil(L/big_beam);
Pos_w = ceil(W/big_beam);
end

