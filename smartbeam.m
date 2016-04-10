%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%
%智能扫描方案
clc
clear all
close all
%%
%场景模型设置
map_length = 80;%探测区域长度
map_width = 50;%探测区域宽度
%%
%设定波束细节
T1 = 0.0037; %单波束驻留时间，波束切换时间不考虑
allow_T = 0.5; %跟踪扫描时全局容忍空白时间
big_beam = 5; %大波束的正方形边长
small_beam = 1; %小波束正方形边长
big_has_small_num = ceil(big_beam/small_beam); %大波束内包含的小波束个数
T_b = map_length * map_width / (big_beam * big_beam)*T1; %大波束扫描整个区域需要的时间
t = 0:T1:8*T_b;
num_l = map_length / big_beam; %大波束横轴扫描次数
num_w = map_width / big_beam;%大波束纵轴扫描次数
%%
%定义场景数组
map_l = 0.05;%运动模型的分辨率
map_w = 0.01;
map=ones(map_length/map_l, map_width/map_w)*(-1); %初始map数组，初始化为-1
%运动模型设置
R0_l = 1; %横轴初始距离
v0_l = 3; %横轴初始速度
a0_l = 0; %横轴加速度

R0l = R0_l + v0_l .* t + 0.5 * a0_l .* t.^2; %实时横坐标

R0_w=45; %纵轴初始距离
v0_w=0; %纵轴初始速度
a0_w=-9.8;  %纵轴加速度
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

%%
%智能波束方案
beamPos_w = 1;
beamPos_l = 1;%波束的位置
Trl = [];
Trw = [];
Trv = [];
big_beam_track_flag = 0; %大波束跟踪模式
find_from_big_beam_flag = 0;
small_beam_track_flag = 0;%小波束跟踪模式
track_flag = 0;%进入跟踪模式

track_ing_flag = 0;%是否需要继续跟踪还是重设设立参考窗，设置为0为设立参考窗
Objects = [];%存储全局扫描得到的物体
Big_Objects = [];%存储大波束分时跟踪时的物体
Small_Objects = [];%存储从大波束中获取的小波束的物体
for i = 1:length(t)
    if(R0l(i)<=map_length && R0w(i) <= map_width && R0l(i)>0 && R0w(i) >0 )
        [map, RL_pre, RW_pre] = updatemap(map,R0l(i),RL_pre,R0w(i),RW_pre,v(i),map_l,map_w); %实时更新map
    end
    PREL = [PREL (RL_pre+0.5)*map_l];
    PREW = [PREW (RW_pre+0.5)*map_w];
    if(~track_flag)
        %全局扫描
        [hasObject, L, W, vv,map_index_w] = bigBeamFindObject(beamPos_w, beamPos_l,map,big_beam, map_l,map_w);
        if(hasObject)
            fprintf('大波束(%d,%d)扫描时发现目标(%.4f, %.4f),速度为%.4f\n',beamPos_l ,beamPos_w, L,W,vv);
            %发现目标，将相关信息存入Object数组,每行分别表示(横向距离，纵向距离，速度， 横向波束位置，纵向波束位置)
            Objects = [Objects; L W vv beamPos_l beamPos_w];
        else
            beamPos_l = beamPos_l + 1;
            if(beamPos_l > num_l)
                beamPos_w = beamPos_w + 1;
                beamPos_l = 1;
            end
            
            if beamPos_w > num_w
                %全局扫描结束
                beamPos_w = 1;
                beamPos_l = 1;
                if ~isempty(Objects)
                    track_flag = 1;
                    big_beam_track_flag = 1;
                    s_track_time = i;
                end
            end
        end
    else
        %开始跟踪
        if((i-s_track_time)*T1 > allow_T)%确保跟踪时间不超过空白容忍时间
            fprintf('当前时间%.4f,超过容忍时间，退出跟踪模式，重新用大波束进行全局扫描\n',i*T1);
            disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
            track_flag = 0;
            %track_ing_flag = 0;
            beamPos_w = 1;
            beamPos_l = 1;%重新初始化大波束的位置
        else
            if(big_beam_track_flag)
                %根据Objects里的信息分别确立大波束跟踪扫描窗，锁定物体大致方位
                object_num = size(Objects, 1);
                Big_Scan_Window_l = cell(1, object_num);
                Big_Scan_Window_w = cell(1, object_num);
                for k = 1: object_num
                    [scan_window_l ,scan_window_w] = getScanWindow(Objects(i,4), Objects(i,5),max_big_pos_l,max_big_pos_w,map_w,big_beam,Objects(i,3),T1);
                    Big_Scan_Window_l{k} = scan_window_l;
                    Big_Scan_Window_w{k} = scan_window_w;
                end
                s = 1; %大波束跟踪目标序号
                s_window = 1;%扫描窗的定位序列
                beamPos_l = Big_Scan_Window_l{s}(s_window);
                beamPos_w = Big_Scan_Window_w{s}(s_window);
                [hasObject, L, W, V,map_index_w] = bigBeamFindObject(beamPos_l, beamPos_w,map,big_beam, map_l,map_w);
                fprintf('正在跟踪第%d个目标\n', s);
                if(hasObject)
                    fprintf('大波束跟踪时发现目标(%.4f, %.4f),速度为%.4f\n', L,W,V);
                    Big_Objects = [Big_Objects; L W V beamPos_l beamPos_w map_index_w];
                    s = s + 1;
                    s_window = 1;
                    if( s > object_num)
                        find_from_big_beam_flag = 1;%大波束跟踪阶段完成，把每个目标定位到小波束里去
                        big_beam_track_flag = 0;
                    end
                else
                    s_window = s_window + 1;
                    if(s_window > length(Big_Scan_Window_l{s}))
                        fprintf('第%d个目标跟踪丢失\n', s);
                        s = s + 1;
                    end
                    
                end
            end
            
            if(find_from_big_beam_flag)
                %在具体的大波束内定位到物体所在的小波束
                for k = 1: object_num
                    found_object =0;
                    for j = 1: big_has_small_num
                        smallBeamPos_l = (Big_Objects(k, 4)-1) * num + j;
                        smallBeamPos_w = ceil(Big_Objects(k, 6)/(small_beam/map_w));
                        [hasObject, L, W, V] = smallBeamFindObject(smallBeamPos_l, smallBeamPos_w, map, small_beam, map_l, map_w);
                        i=i+1;
                        if hasObject
                            found_object = 1;
                            Small_Objects = [Small_Objects; L W V smallBeamPos_l smallBeamPos_w];
                            fprintf('大波束（%d,%d)定位到小波束(%d,%d)，发现目标(%.4f, %.4f),速度为%.4f\n', Big_Objects(k, 4),Big_Objects(k, 5),smallBeamPos_l,smallBeamPos_w,L,W,V);
                            break;
                        end
                    end
                    if(~found_object)
                        fprintf('第%d个目标定位到小波束跟踪丢失\n', k);
                    end
                end
                fprintf('大波束定位到小波束完成\n');
                find_from_big_beam_flag = 0;
                small_beam_track_flag = 1;
            end
            
            
            
            
            fprintf('大波束(%d,%d)跟踪中...\n',scan_window_l(k),scan_window_w(k));
        end
    end
end
