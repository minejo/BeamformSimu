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
big_beam = 4; %大波束的正方形边长
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
%大波束扫描方案
beamPos_w = 1;
beamPos_l = 1;%波束的位置
Rl = [];
Rw = [];
V = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%大波束顺序扫描
% for i = 1: length(t)
%     [map, RL_pre, RW_pre] = updatemap(map,R0l(i),RL_pre,R0w(i),RW_pre,v(i),map_l,map_w); %实时更新map
%     PREL = [PREL (RL_pre+0.5)*map_l];
%     PREW = [PREW (RW_pre+0.5)*map_w];
%
%     %在单个大波束内查找有没有物体
%     [hasObject, L, W, vv] = bigBeamFindObject(map_length, map_width, beamPos_w, beamPos_l,map,big_beam, map_l,map_w);
%
%     beamPos_l = beamPos_l + 1;
%     if(beamPos_l > num_l)
%         beamPos_w = beamPos_w + 1;
%         beamPos_l = 1;
%     end
%
%     if beamPos_w > num_w
%         beamPos_w = 1;
%         beamPos_l = 1;
%     end
%
%     if(hasObject)
%         i
%         Rl = [Rl L];
%         Rw = [Rw W];
%         V = [V vv];
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%波束跟踪方案
RL_pre = R0_l/map_l;%map更新时对应的上一时刻的值
RW_pre = R0_w/map_w;
beamPos_w = 1;
beamPos_l = 1;%波束的位置
Trl = [];
Trw = [];
Trv = [];
s_track_time = -1;%设置初始化跟踪时间为-1
track_flag = 0; %设置为1时即开启跟踪模式
for i = 1: length(t)
    [map, RL_pre, RW_pre] = updatemap(map,R0l(i),RL_pre,R0w(i),RW_pre,v(i),map_l,map_w); %实时更新map
    PREL = [PREL (RL_pre+0.5)*map_l];
    PREW = [PREW (RW_pre+0.5)*map_w];
    if(~track_flag)
        %在单个大波束内查找有没有物体
        [hasObject, L, W, vv] = bigBeamFindObject(map_length, map_width, beamPos_w, beamPos_l,map,big_beam, map_l,map_w);
        if(hasObject)
            %如果有，在大波束内定位到具体的小波束，可以利用的信息是距离信息
            [small_l, small_w ,small_v,smallBeamPos_l,smallBeamPos_w] = findFromBigBeam(beamPos_l, beamPos_w, W, small_beam, big_beam,map_l,map_w,map);
            Trl = [Trl small_l];
            Trw = [Trw small_w];
            Trv = [Trv small_v];
            %obj_num = length(small_l);%一个区域内物体的数目
            %启用波束跟踪方案
            s_track_time = i; %开始跟踪的时间序号
            track_flag = 1;
        else
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
    else
        %使用小波束跟踪
        %[small_l, small_w, small_v] = beamTrack(smallBeamPos_l, smallBeamPos_w, map, small_beam, map_l,map_w, s_track_time, allow_T, i);
        if((i-s_track_time)*T1 > allow_T)%确保跟踪时间不超过空白容忍时间
            track_flag = 0;
            beamPos_w = 1;
            beamPos_l = 1;%重新初始化大波束的位置
        else
            %获取跟踪扫描窗
            [scan_window_l scan_window_w] = getScanWindow(smallBeamPos_l, smallBeamPos_w,map_l,map_w,small_v);
        end
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%
%结果对比
figure;
plot(PREL,PREW);%理论曲线
hold on;
plot(Trl,Trw,'r');

