%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%
%智能波束仿真

%%
%场景模型设置
length = 50;%探测区域长度
width = 20;%探测区域宽度
%%
%设定波束细节
T1 = 0.0037; %单波束驻留时间，波束切换时间不考虑
allow_T = 1.5; %跟踪扫描时全局容忍空白时间
big_beam = 1.5; %大波束的正方形边长
small_beam = 0.3; %小波束正方形边长
T_b = length * width / (big_beam * big_beam); %大波束扫描整个区域需要的时间
t = 0:T1:25*T_b;
%%
%定义场景数组
map_l = 0.01;%运动模型的分辨率
map_w = 0.01;
map=ones(length/0.1, width/0.1)*(-1); %初始map数组，初始化为-1
%运动模型设置
R0_l = 70; %横轴初始距离
v0_l = -4; %横轴初始速度
a0_l = -3; %横轴加速度

R0l = R0_l + v0_l .* t + 0.5 * a0_l .* t.^2; %实时横坐标

R0_w=20; %纵轴初始距离
v0_w=2; %纵轴初始速度
a0_w=1; %纵轴加速度

R0w = R0_w + v0_w .* t + 0.5 * a0_w .* t.^2;  %实时纵坐标

map(R0_l, R0_w) = abs(v0_w); %雷达仅能探测距离向的速度
RL_pre = R0_l;%map更新时对应的上一时刻的值
RW_pre = R0_w;

%%
%波束跟踪方案
for i = 1: length(t)
    [map, RL_pre, RW_pre] = updatemap(map,R0l,RL_pre,R0w,RW_pre,v0_w);
    
end

%%
%结果对比
