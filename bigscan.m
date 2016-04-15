%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%
function [Track_l ,Track_w, scan_count] = bigscan(time_num,map_length,map_width,R0_l,v0_l,a0_l,R0_w,v0_w,a0_w)
%%
%场景模型设置
%map_length = 80;%探测区域长度
%map_width = 50;%探测区域宽度
%%
%设定波束细节
T1 = 0.0037; %单波束驻留时间，波束切换时间不考虑

big_beam = 8; %大波束的正方形边长
T_b = map_length * map_width / (big_beam * big_beam)*T1; %大波束扫描整个区域需要的时间

t = 0:T1:time_num*T_b;
num_l = map_length / big_beam; %大波束横轴扫描次数
num_w = map_width / big_beam;%大波束纵轴扫描次数
%%
%定义场景数组
map_l = 0.05;%运动模型的分辨率
map_w = 0.01;
map=zeros(map_length/map_l, map_width/map_w); %初始map数组，初始化为-1
%运动模型设置
% R0_l = 1; %横轴初始距离
% v0_l = 3; %横轴初始速度
% a0_l = 0; %横轴加速度

R0l = R0_l + v0_l .* t + 0.5 * a0_l .* t.^2; %实时横坐标

% R0_w=45; %纵轴初始距离
% v0_w=0; %纵轴初始速度
% a0_w=-9.8;  %纵轴加速度

v = v0_w + a0_w.*t;

R0w = R0_w + v0_w .* t + 0.5 * a0_w .* t.^2;  %实时纵坐标

map(fix(R0_l/map_l), fix(R0_w/map_w)) = abs(v0_w); %雷达仅能探测距离向的速度

RL_pre = R0_l/map_l;%map更新时对应的上一时刻的值
RW_pre = R0_w/map_w;
PREL = [];
PREW = [];

[map_x map_y] = size(map);

%%
%大波束扫描方案
beamPos_w = 1;
beamPos_l = 1;%波束的位置
Rl = [];
Rw = [];
V = [];
scan_count = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%大波束顺序扫描
for i = 1: length(t)
    %fprintf('当前时间(big-scan)是%.4f\n',i*T1);
    if(R0l(i)<=map_length && R0w(i) <= map_width && R0l(i)>0 && R0w(i) >0 )
        [map, RL_pre, RW_pre] = updatemap(map,R0l(i),RL_pre,R0w(i),RW_pre,v(i),map_l,map_w); %实时更新map
    end
    
    if(RL_pre && RW_pre && RL_pre<= (map_length/map_l) && RW_pre <= (map_width/map_w) && RL_pre>0 && RW_pre >0 )
        PREL = [PREL (RL_pre+0.5)*map_l];
        PREW = [PREW (RW_pre+0.5)*map_w];
    end
    
    
    %在单个大波束内查找有没有物体
    [hasObject, L, W, vv,map_index_w] = bigBeamFindObject(beamPos_l,beamPos_w ,map,big_beam, map_l,map_w);
    if(hasObject)
        scan_count = scan_count + 1;
        fprintf('大波束(%d,%d)扫描时发现目标(%.4f, %.4f),速度为%.4f\n',beamPos_l ,beamPos_w,L,W,vv);
        Rl = [Rl L];
        Rw = [Rw W];
        V = [V vv];
    end
    beamPos_l = beamPos_l + 1;
    if(beamPos_l > num_l)
        beamPos_w = beamPos_w + 1;
        beamPos_l = 1;
    end
    
    if beamPos_w > num_w
        beamPos_w = 1;
        beamPos_l = 1;
    end   
    
end
Track_l = Rl;
Track_w = Rw;
end