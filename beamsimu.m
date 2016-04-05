%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%
%智能波束仿真
clc
clear all
close all
%%
%场景模型设置
map_length = 100;%探测区域长度
map_width = 40;%探测区域宽度
%%
%设定波束细节
T1 = 0.0037; %单波束驻留时间，波束切换时间不考虑
allow_T = 1.5; %跟踪扫描时全局容忍空白时间
big_beam = 3; %大波束的正方形边长
small_beam = 0.3; %小波束正方形边长
T_b = map_length * map_width / (big_beam * big_beam)*T1; %大波束扫描整个区域需要的时间
t = 0:T1:7*T_b;
num_l = map_length / big_beam; %大波束横轴扫描次数
num_w = map_width / big_beam;%大波束纵轴扫描次数
%%
%定义场景数组
map_l = 0.05;%运动模型的分辨率
map_w = 0.01;
map=ones(map_length/map_l, map_width/map_w)*(-1); %初始map数组，初始化为-1
%运动模型设置
R0_l = 10; %横轴初始距离
v0_l = 3; %横轴初始速度
a0_l = 0; %横轴加速度

R0l = R0_l + v0_l .* t + 0.5 * a0_l .* t.^2; %实时横坐标

R0_w=5; %纵轴初始距离
v0_w=2; %纵轴初始速度
a0_w=0;  %纵轴加速度
v = v0_w + a0_w.*t;

R0w = R0_w + v0_w .* t + 0.5 * a0_w .* t.^2;  %实时纵坐标

map(fix(R0_l/map_l), fix(R0_w/map_w)) = abs(v0_w); %雷达仅能探测距离向的速度
RL_pre = R0_l/map_l;%map更新时对应的上一时刻的值
RW_pre = R0_w/map_w;
PREL = [];
PREW = [];
%%
%波束跟踪方案
beamPos_w = 1;
beamPos_l = 1;%波束的位置
Rl = [];
Rw = [];
V = [];
for i = 1: length(t)
    [map, RL_pre, RW_pre] = updatemap(map,R0l(i),RL_pre,R0w(i),RW_pre,v(i),map_l,map_w); %实时更新map
    PREL = [PREL (RL_pre+0.5)*map_l];
    PREW = [PREW (RW_pre+0.5)*map_w];
    
     [hasObject, L, W, vv] = bigBeamFindObject(map_length, map_width, beamPos_w, beamPos_l,map,big_beam, map_l,map_w);
     beamPos_l = beamPos_l + 1;
     if(beamPos_l > num_l)
        beamPos_w = beamPos_w + 1;
        beamPos_l = 1;
    end
    
    if beamPos_w > num_w
        beamPos_w = 1;
        beamPos_l = 1;
    end 
    
    if(hasObject)
        i
        Rl = [Rl L];
        Rw = [Rw W];
    end
end

%%
%结果对比
figure;
plot(PREL,PREW);%理论曲线
hold on;
plot(Rl,Rw,'r');

