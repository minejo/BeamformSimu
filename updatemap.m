%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%

function [map, RL_pre, RW_pre] = updatemap(map,R0l,RL_pre,R0w,RW_pre,v0_w,map_l,map_w)
%改变物体的map中映射的位置
[x,y]=size(map);
x_t=1:x;
y_t=1:y;
map(RL_pre,RW_pre)=0;
RL=find(x_t<=R0l/map_l, 1, 'last' );
RW=find(y_t<=R0w/map_w, 1, 'last' );
if isempty(RL)
    RL = 1;
end
if isempty(RW)
    RW = 1;
end
map(RL,RW)=v0_w;
RL_pre=RL;
RW_pre=RW;
end