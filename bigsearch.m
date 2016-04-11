%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%
function [Track_l ,Track_w] = bigsearch(R0_l,v0_l,a0_l,R0_w,v0_w,a0_w,allow_T)
%%
%场景模型设置
map_length = 80;%探测区域长度
map_width = 50;%探测区域宽度
%%
%设定波束细节
T1 = 0.0037; %单波束驻留时间，波束切换时间不考虑
big_beam = 5; %大波束的正方形边长
small_beam = 1; %小波束正方形边长
T_b = map_length * map_width / (big_beam * big_beam)*T1; %大波束扫描整个区域需要的时间
allow_T = 0.8*T_b; %跟踪扫描时全局容忍空白时间
t = 0:T1:8*T_b;
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
init_map = map;
RL_pre = R0_l/map_l;%map更新时对应的上一时刻的值
RW_pre = R0_w/map_w;
PREL = [];
PREW = [];

[map_x map_y] = size(map);
max_small_pos_l = map_x/(small_beam/map_l);
max_small_pos_w = map_y/(small_beam/map_w);
max_big_pos_l=map_x/(big_beam/map_l);
max_big_pos_w = map_y/(big_beam/map_w);

RL_pre = R0_l/map_l;%map更新时对应的上一时刻的值
RW_pre = R0_w/map_w;
beamPos_w = 1;
beamPos_l = 1;%波束的位置
BTrl = [];
BTrw = [];
BTrv = [];
s_track_time = -1;%设置初始化跟踪时间为-1
track_flag = 0; %设置为1时即开启跟踪模式
track_ing_flag = 0;%是否需要继续跟踪还是重设设立参考窗，设置为0为设立参考窗
map = init_map;

for i = 1: length(t)
    if(R0l(i)<=map_length && R0w(i) <= map_width && R0l(i)>0 && R0w(i) >0 )
        [map, RL_pre, RW_pre] = updatemap(map,R0l(i),RL_pre,R0w(i),RW_pre,v(i),map_l,map_w); %实时更新map
    end
    %PREL = [PREL (RL_pre+0.5)*map_l];
    %PREW = [PREW (RW_pre+0.5)*map_w];
    if(~track_flag)
        [hasObject, L, W, vv,map_index_w] = bigBeamFindObject(beamPos_l,beamPos_w ,map,big_beam, map_l,map_w);
        if(hasObject)
            fprintf('大波束(%d,%d)扫描时发现目标(%.4f, %.4f),速度为%.4f\n',beamPos_l, beamPos_w, L,W,vv);
            BTrl = [BTrl L];
            BTrw = [BTrw W];
            BTrv = [BTrv vv];
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
        %使用大波束跟踪
        if((i-s_track_time)*T1 > allow_T)%确保跟踪时间不超过空白容忍时间
            fprintf('当前时间%.4f,超过容忍时间，退出跟踪模式，重新用大波束进行全局扫描\n',i*T1);
            disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
            track_flag = 0;
            track_ing_flag = 0;
            beamPos_w = 1;
            beamPos_l = 1;%重新初始化大波束的位置
        else
            if(~track_ing_flag)
                fprintf('获取跟踪扫描窗');
                [scan_window_l ,scan_window_w] = getScanWindow(beamPos_l, beamPos_w,max_big_pos_l,max_big_pos_w,map_w,big_beam,vv,T1,0);
                k=1; %扫描窗的定位序列
                track_ing_flag=1;
                window_num = length(scan_window_l);%扫描窗数目
                fprintf(' 跟踪扫描窗长度为%d\n扫描窗为：', window_num);
                for p = 1: window_num
                    fprintf('(%d,%d)',scan_window_l(p),scan_window_w(p));
                end
                fprintf('\n');
                fprintf('大波束跟踪中,当前时间%.4f\n',i*T1);
            end
            
            fprintf('大波束(%d,%d)扫描中...\n',scan_window_l(k),scan_window_w(k));
            [hasObject, L, W, V,map_index_w] = bigBeamFindObject(scan_window_w(k), scan_window_l(k),map,big_beam, map_l,map_w);
            if(hasObject)
                BTrl = [BTrl L];
                BTrw = [BTrw W];
                BTrv = [BTrv V];
                beamPos_l = scan_window_l(k);
                beamPos_w = scan_window_w(k);
                track_ing_flag=0;
                fprintf('大波束(%d,%d)发现目标(%.4f,%.4f)，速度为%.4f，重新开始跟踪流程\n',scan_window_l(k),scan_window_w(k),L,W,V);
            else
                k=k+1;
                if(k>window_num)
                    k=1;
                    track_flag = 0;
                    track_ing_flag = 0;
                    fprintf('目标跟踪丢失，重新进行全局扫描\n');
                end
            end
        end        
    end    
end
Track_l=BTrl;
Track_w=BTrw;
end