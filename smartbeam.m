%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%
%智能扫描方案
function [TRACK_L,TRACK_W,Global_count,PREL,PREW] = smartbeam(time_num,map_length,map_width,R0_l,v0_l,a0_l,R0_w,v0_w,a0_w,allow_T)
%%
%场景模型设置
%map_length = 160;%探测区域长度
%map_width = 80;%探测区域宽度
%%
%设定波束细节
T1 = 0.0037; %单波束驻留时间，波束切换时间不考虑

big_beam = 8; %大波束的正方形边长
small_beam = 1; %小波束正方形边长
big_has_small_num = ceil(big_beam/small_beam); %大波束内包含的小波束个数
T_b = map_length * map_width / (big_beam * big_beam)*T1; %大波束扫描整个区域需要的时间
%allow_T = 0.4*T_b; %跟踪扫描时全局容忍空白时间

t = 0:T1:time_num*T_b;
num_l = map_length / big_beam; %大波束横轴扫描次数
num_w = map_width / big_beam;%大波束纵轴扫描次数
%%
%定义场景数组
map_l = 0.05;%运动模型的分辨率
map_w = 0.01;
map=zeros(map_length/map_l, map_width/map_w); %初始map数组，初始化为0
%运动模型设置
% R0_l = 1; %横轴初始距离
% v0_l = 3; %横轴初始速度
% a0_l = 0; %横轴加速度

R0l = R0_l + v0_l .* t + 0.5 * a0_l .* t.^2; %实时横坐标

% R0_w=45; %纵轴初始距离
% v0_w=0; %纵轴初始速度
% a0_w=-9;  %纵轴加速度
v = v0_w + a0_w.*t;

R0w = R0_w + v0_w .* t + 0.5 * a0_w .* t.^2;  %实时纵坐标

map(fix(R0_l/map_l), fix(R0_w/map_w)) = v0_w; %雷达仅能探测距离向的速度
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
horizental_trend = 0;%0表示未知，-1表示向左，1表示向右
small_track_ing_flag = 0;%是否需要继续跟踪还是重设设立参考窗，设置为0为设立参考窗
Objects = [];%存储全局扫描得到的物体
Global_count = 0;
Result = {};
for i = 1:length(t)
    if(R0l(i)<=map_length && R0w(i) <= map_width && R0l(i)>0 && R0w(i) >0 )
        [map, RL_pre, RW_pre] = updatemap(map,R0l(i),RL_pre,R0w(i),RW_pre,v(i),map_l,map_w); %实时更新map
    else
        break;
    end
    if(RL_pre && RW_pre && RL_pre<= (map_length/map_l) && RW_pre <= (map_width/map_w) && RL_pre>0 && RW_pre >0 )
        PREL = [PREL (RL_pre+0.5)*map_l];
        PREW = [PREW (RW_pre+0.5)*map_w];
    end
    if(~track_flag)
        %全局扫描
        %Objects = [];
        [hasObject, L, W, vv,map_index_w] = bigBeamFindObject(beamPos_l,beamPos_w,map,big_beam, map_l,map_w);
        if(hasObject)
            fprintf('全局扫描大波束(%d,%d)扫描时发现目标(%.4f, %.4f),速度为%.4f\n',beamPos_l ,beamPos_w, L,W,vv);
            %发现目标，将相关信息存入Object数组,每行分别表示(横向距离，纵向距离，速度， 横向波束位置，纵向波束位置)
            Objects = [Objects; L W vv beamPos_l beamPos_w];
        end
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
                [big_l,big_w,small_l,small_w ] = gettheorylocation( R0l(i),R0w(i),small_beam,big_beam,map,map_l,map_w );
                fprintf('当前理论物体运动方位(%.4f,%.4f)，理论波束方位(%d,%d)\n',R0l(i),R0w(i),big_l, big_w);
                fprintf(2,'大波束全局扫描结束，当前时间%.4f\n',i*T1);
                track_flag = 1;
                s_track_time = i;
                %大波束跟踪参数设计
                big_beam_track_flag = 1;
                big_beam_track_object = 1;%大波束跟踪目标序号
                big_beam_track_window = 1;%大波束跟踪扫描窗的定位序列
                object_num = size(Objects, 1);
                Big_Scan_Window_l = cell(1, object_num);
                Big_Scan_Window_w = cell(1, object_num);
                Big_Objects = cell(1, object_num);%存储大波束分时跟踪时的物体
                if(Global_count == 0)
                    Result = cell(1, object_num);%用于存在过程中得到的信息
                end
                Global_count = Global_count + 1;
            end
        end
        
    else
        %开始跟踪
        if((i-s_track_time)*T1 > allow_T)%确保跟踪时间不超过空白容忍时间
            fprintf(2,'当前时间%.4f,超过容忍时间，退出跟踪模式，重新用大波束进行全局扫描\n',i*T1);
            disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
            track_flag = 0;
            beamPos_w = 1;
            beamPos_l = 1;%重新初始化大波束的位置
            big_beam_track_flag = 0;
            find_from_big_beam_flag = 0;
            small_beam_track_flag = 0;
            Objects = [];
            search_time = 2;%默认每个扫描窗范围可以重复扫描的次数
            search_count = 0;
        else
            if(big_beam_track_flag)
                %根据Objects里的信息分别确立大波束跟踪扫描窗，锁定物体大致方位
                for k = 1: object_num
                    fprintf(2,'获取大波束跟踪扫描窗,横向运动趋势为%d\n',horizental_trend);
                    [scan_window_l ,scan_window_w] = getScanWindow(Objects(k,4), Objects(k,5),max_big_pos_l,max_big_pos_w,map_w,big_beam,Objects(k,3),T1,0.5*T_b,horizental_trend);
                    Big_Scan_Window_l{k} = scan_window_l;
                    Big_Scan_Window_w{k} = scan_window_w;
                end
                
                
                beamPos_l = Big_Scan_Window_l{big_beam_track_object}(big_beam_track_window);
                beamPos_w = Big_Scan_Window_w{big_beam_track_object}(big_beam_track_window);
                fprintf('正在跟踪第%d个物体，跟踪扫描窗长度为%d\n扫描窗为：', big_beam_track_object,length(Big_Scan_Window_l{big_beam_track_object}));
                for p = 1: length(Big_Scan_Window_l{big_beam_track_object})
                    fprintf('(%d,%d)',Big_Scan_Window_l{big_beam_track_object}(p),Big_Scan_Window_w{big_beam_track_object}(p));
                end
                fprintf('\n');
                [big_l,big_w,small_l,small_w ] = gettheorylocation( R0l(i),R0w(i),small_beam,big_beam,map,map_l,map_w );
                fprintf('当前理论物体运动方位(%.4f,%.4f)，理论波束方位(%d,%d)\n',R0l(i),R0w(i),big_l,big_w);
                fprintf('大波束跟踪中，正在跟踪第%d个目标，当前大波束为(%d,%d)\n',big_beam_track_object,beamPos_l,beamPos_w);
                big_map_test = getBigMap(beamPos_l, beamPos_w, big_beam,map,map_l,map_w);
                [hasObject, L, W, V,map_index_w] = bigBeamFindObject(beamPos_l, beamPos_w,map,big_beam, map_l,map_w);
                
                if(hasObject)
                    fprintf('大波束(%d,%d)跟踪时发现目标(%.4f, %.4f),速度为%.4f\n',beamPos_l,beamPos_w, L,W,V);
                    Big_Objects{big_beam_track_object} = [L W V beamPos_l beamPos_w map_index_w];
                   % Result{big_beam_track_object} = [Result{big_beam_track_object};L W V  0 beamPos_l beamPos_w];%第4列表示后面跟的是大波束参数还是小波束参数，0为大波束，1为小波束
                    big_beam_track_object= big_beam_track_object + 1;
                    big_beam_track_window = 1;
                else
                    big_beam_track_window = big_beam_track_window + 1;
                    if(big_beam_track_window > length(Big_Scan_Window_l{big_beam_track_object}))
                        fprintf('第%d个目标大波束跟踪丢失\n', big_beam_track_object);
                        if(search_count < search_time-1)
                            big_beam_track_window = 1;
                            search_count = search_count + 1;
                            fprintf(2,'第%d个目标大波束开始第二遍跟踪....................................\n', big_beam_track_object);
                        else
                            big_beam_track_object = big_beam_track_object + 1;
                            big_beam_track_window = 1;
                        end
                    end
                end
                
                if( big_beam_track_object > object_num)
                    find_from_big_beam_flag = 1;%大波束跟踪阶段完成，把每个目标定位到小波束里去
                    big_beam_track_flag = 0;
                    %大波束定位小波束参数初始化
                    big_find_small_object = 1;%大波束定位小波束物体序号
                    if(Big_Objects{big_find_small_object}(3) > 0)
                        big_find_small_window = 1;%大波束定位小波束窗口序号
                        update_unit = 1;
                    else
                        big_find_small_window = big_has_small_num;
                        update_unit = -1;
                    end
                    Small_Objects = cell(1, object_num);%存储从大波束中获取的小波束的物体
                end
            else
                if(find_from_big_beam_flag)
                    %在具体的大波束内定位到物体所在的小波
                    found_object =0;
                    if ~isempty(Big_Objects{big_find_small_object})
                        smallBeamPos_l = (Big_Objects{big_find_small_object}(4)-1) * big_has_small_num + big_find_small_window;
                        smallBeamPos_w = ceil(Big_Objects{big_find_small_object}(6)/(small_beam/map_w));
                        
                        [big_l,big_w,small_l,small_w ] = gettheorylocation( R0l(i),R0w(i),small_beam,big_beam,map,map_l,map_w );
                        fprintf('当前理论物体运动方位(%.4f,%.4f)，理论大波束方位(%d,%d)，理论小波束方位(%d,%d)\n',R0l(i),R0w(i),big_l,big_w,small_l,small_w);
                        fprintf('大波束(%d,%d)中小波束(%d,%d)中定位物体\n',Big_Objects{big_find_small_object}(4),Big_Objects{big_find_small_object}(5),smallBeamPos_l,smallBeamPos_w);
                        [hasObject, L, W, V] = smallBeamFindObject(smallBeamPos_l, smallBeamPos_w, map, small_beam, map_l, map_w);
                        if hasObject
                            found_object = 1;
                            Small_Objects{big_find_small_object} = [L W V smallBeamPos_l smallBeamPos_w];
                            Result{big_find_small_object} = [Result{big_find_small_object}; L W V 1 smallBeamPos_l smallBeamPos_w];
                            fprintf('大波束（%d,%d)定位到小波束(%d,%d)，发现目标(%.4f, %.4f),速度为%.4f\n', Big_Objects{big_find_small_object}(4),Big_Objects{big_find_small_object}(5),smallBeamPos_l,smallBeamPos_w,L,W,V);
                            big_find_small_object = big_find_small_object + 1;
                        else
                            big_find_small_window = big_find_small_window+update_unit;
                            if big_find_small_window > big_has_small_num  || big_find_small_window < 1
                                if ~found_object
                                    fprintf('第%d个目标定位到小波束定位丢失\n', big_find_small_object);
                                end
                                big_find_small_object = big_find_small_object + 1;
                            end
                        end
                    else
                        big_find_small_object = big_find_small_object+ 1;
                        fprintf(2,'第%d个目标大波束定位丢失\n', big_find_small_object);
                    end
                    
                    if(found_object)
                        if(big_find_small_object < object_num )
                            if(Big_Objects{big_find_small_object}(3) > 0)
                                big_find_small_window = 1;%大波束定位小波束窗口序号
                                update_unit = 1;
                            else
                                big_find_small_window = big_has_small_num;
                                update_unit = -1;
                            end
                        end
                    end
                    
                    if(big_find_small_object > object_num)
                        if(~cellfun(@isempty,Small_Objects))
                            fprintf(2,'大波束定位到小波束完成\n');
                            find_from_big_beam_flag = 0;
                            small_beam_track_flag = 1;
                            fprintf('开始小波束跟踪中,当前时间%.4f\n',i*T1);
                            %小波束跟踪相关参数
                            small_track_ing_flag = 1;%确定是否需要重新设定扫描窗
                            small_track_object = 1;%跟踪物体序列
                            small_track_window = 1;%跟踪窗序列
                            Small_Scan_Window_l = cell(1,object_num);
                            Small_Scan_Window_w = cell(1,object_num);
                            Tracking_Objects = cell(1, object_num);
                            small_tracking_status = zeros(1, object_num);%0表示没发现目标，1表示成功跟踪，2表示跟丢了
                            small_tracking_continue = 0;
                            
                            if ~isempty(Small_Objects{small_track_object})
                                pre_l_window = Small_Objects{small_track_object}(4);
                            else
                                pre_l_window = 0;
                            end
                            search_time = 2;%默认每个扫描窗范围可以重复扫描的次数
                            search_count = 0;
                        else
                            fprintf(2,'重新启动大波束跟踪......................................................\n');
                            find_from_big_beam_flag = 0;
                            %大波束跟踪参数设计
                            big_beam_track_flag = 1;
                            big_beam_track_object = 1;%大波束跟踪目标序号
                            big_beam_track_window = 1;%大波束跟踪扫描窗的定位序列
                        end
                    end
                else
                    if(small_beam_track_flag)
                        if ~isempty(Small_Objects{small_track_object})
                            if(small_track_ing_flag)
                                fprintf(2,'获取小波束跟踪扫描窗,横向运动趋势为%d\n',horizental_trend);
                                [scan_window_l ,scan_window_w] = getScanWindow(Small_Objects{small_track_object}(4), Small_Objects{small_track_object}(5),max_small_pos_l,max_small_pos_w,map_w,small_beam,Small_Objects{small_track_object}(3),T1,0,horizental_trend);
                                Small_Scan_Window_l{small_track_object} = scan_window_l;
                                Small_Scan_Window_w{small_track_object} = scan_window_w;
                                small_track_ing_flag = 0;
                                t_window_num = length( Small_Scan_Window_l{small_track_object});
                                fprintf(' 正在跟踪第%d个物体，跟踪扫描窗长度为%d\n扫描窗为：', small_track_object,t_window_num);
                                for p = 1: t_window_num
                                    fprintf('(%d,%d)',scan_window_l(p),scan_window_w(p));
                                end
                                fprintf('\n');
                            end
                            
                            %test_small_map = getSmallMap(Small_Scan_Window_l{small_track_object}(small_track_window),Small_Scan_Window_w{small_track_object}(small_track_window),small_beam,map,map_l,map_w);
                            
                            [big_l,big_w,small_l,small_w ] = gettheorylocation( R0l(i),R0w(i),small_beam,big_beam,map,map_l,map_w );
                            fprintf('小波束(%d,%d)扫描中...当前时间%.4f\n',Small_Scan_Window_l{small_track_object}(small_track_window),Small_Scan_Window_w{small_track_object}(small_track_window),i*T1);
                            fprintf('当前理论物体运动方位(%.4f,%.4f)，理论小波束方位(%d,%d)\n',R0l(i),R0w(i),small_l, small_w);
                            [hasObject, L, W, V] = smallBeamFindObject(Small_Scan_Window_l{small_track_object}(small_track_window), Small_Scan_Window_w{small_track_object}(small_track_window), map, small_beam, map_l, map_w);
                            if(hasObject)
                                small_tracking_status(small_track_object) = 1;
                                search_count = 0;
                                temp_horizental_trend = Small_Scan_Window_l{small_track_object}(small_track_window) - pre_l_window;
                                if(temp_horizental_trend > 0)
                                    horizental_trend = 1;
                                else
                                    if (temp_horizental_trend < 0)
                                        horizental_trend = -1;
                                    end
                                end
                                pre_l_window = Small_Scan_Window_l{small_track_object}(small_track_window);
                                fprintf('小波束(%d,%d)发现目标(%.4f, %.4f),速度为%.4f...当前时间%.4f\n',Small_Scan_Window_l{small_track_object}(small_track_window),Small_Scan_Window_w{small_track_object}(small_track_window),L,W,V,i*T1);
                                Tracking_Objects{small_track_object} = [L W V Small_Scan_Window_l{small_track_object}(small_track_window) Small_Scan_Window_w{small_track_object}(small_track_window)];
                                Result{small_track_object} = [Result{small_track_object}; L W V 1 Small_Scan_Window_l{small_track_object}(small_track_window) Small_Scan_Window_w{small_track_object}(small_track_window)];
                                small_track_object = small_track_object + 1;
                            else
                                small_track_window = small_track_window + 1;
                                if(small_track_window > t_window_num)
                                    if(search_count < search_time-1)
                                        small_track_window = 1;
                                        search_count =search_count + 1;
                                        fprintf(2,'小波束目标跟踪丢失，重新搜索一遍.......................................................................\n');
                                    else
                                        small_track_window = 1;
                                        small_tracking_status(small_track_object) = 2;
                                        fprintf(2,'小波束目标跟踪丢失，跟踪下一个目标\n');
                                        small_track_object = small_track_object + 1;
                                    end
                                end
                            end
                        else
                            small_track_object=small_track_object+1;
                        end
                        
                        
                        if(small_track_object > object_num)
                            for p =1 : object_num
                                if(small_tracking_status(p) == 1)
                                    small_tracking_continue = 1;
                                end
                            end
                            
                            if (small_tracking_continue)
                                small_track_object=1;
                                small_track_ing_flag = 1;
                                small_track_window = 1;
                                Small_Objects = Tracking_Objects;
                                small_tracking_continue = 0;
                                fprintf(2,'继续下一轮跟踪\n');
                            else
                                small_beam_track_flag = 0;
                                track_flag = 0;
                                Objects = [];
                                fprintf(2,'小波束目标全部跟踪丢失，重新进行全局扫描\n');
                            end
                        end
                    end
                    
                end
            end
        end
    end
end
TRACK_L = Result{1}(:,1);
TRACK_W = Result{1}(:,2);