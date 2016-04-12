%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Chao Li %%%
%%%%%%%%%%%%%%%%%%%%%%%

function [scan_window_l scan_window_w] = getScanWindow(BeamPos_l, BeamPos_w,max_pos_l,max_pos_w,map_w, beam_radius,v, T1,time_delay,horizental_trend)
%确定跟踪扫描的扫描窗
scan_window_l = BeamPos_l;
scan_window_w = BeamPos_w;
scan_radius = ceil((abs(v) * (T1+time_delay))/beam_radius); %扫描窗的半径
if(scan_radius == 0)
    scan_radius = 1;
end


%优先在物体运动方向搜寻
for i = 1: scan_radius
    if (v > 0)%水平方向未知，垂直向上运动
        if horizental_trend == 0
            if i == 1
                scan_window_l = [scan_window_l BeamPos_l-1:BeamPos_l+1  ones(1, 2)*(BeamPos_l+1) BeamPos_l:-1:BeamPos_l-1  (BeamPos_l-1)];
                scan_window_w = [scan_window_w  ones(1, 3)*(BeamPos_w+1) BeamPos_w:-1:BeamPos_w-1 ones(1, 2)*(BeamPos_w-1) BeamPos_w];
            else
                scan_window_l = [scan_window_l BeamPos_l-i:BeamPos_l+i ones(1, 2*i)*(BeamPos_l+i) BeamPos_l+i-1:-1:BeamPos_l-i ones(1, 2*i-1)*(BeamPos_l-i)];
                scan_window_w = [scan_window_w  ones(1, 2*i+1)*(BeamPos_w+i) BeamPos_w+i-1:-1:BeamPos_w-i ones(1, 2*i)*(BeamPos_w-i) BeamPos_w-i+1:BeamPos_w+i-1];
            end
        end
        
        if horizental_trend == 1 %横向向右运动
            if i == 1
                scan_window_l = [scan_window_l BeamPos_l:BeamPos_l+1  BeamPos_l+1];
                scan_window_w = [scan_window_w  ones(1, 2)*(BeamPos_w+1) BeamPos_w];
            else
                 scan_window_l = [scan_window_l BeamPos_l:BeamPos_l+i ones(1, i)*(BeamPos_l+i)];
                scan_window_w = [scan_window_w ones(1, i+1)*(BeamPos_w+i) BeamPos_w+i-1:-1:BeamPos_w];
            end
        end
        
        if horizental_trend == -1 %横向向左运动
            if i == 1
                scan_window_l = [scan_window_l BeamPos_l:-1:BeamPos_l-1  BeamPos_l-1];
                scan_window_w = [scan_window_w  ones(1, 2)*(BeamPos_w+1) BeamPos_w];
            else
                scan_window_l = [scan_window_l BeamPos_l:-1:BeamPos_l-i ones(1, i)*(BeamPos_l-i)];
               scan_window_w = [scan_window_w ones(1, i+1)*(BeamPos_w+i) BeamPos_w+i-1:-1:BeamPos_w];
            end
        end
        
    else %垂直向下运动
        if horizental_trend == 0 %横向未知
            if i == 1
                scan_window_l = [scan_window_l BeamPos_l-1:BeamPos_l+1  ones(1, 2)*(BeamPos_l+1) BeamPos_l:-1:BeamPos_l-1  (BeamPos_l-1)];
                scan_window_w = [scan_window_w  ones(1, 3)*(BeamPos_w-1) BeamPos_w:BeamPos_w+1 ones(1, 2)*(BeamPos_w+1) BeamPos_w];
            else
                scan_window_l = [scan_window_l BeamPos_l-i:BeamPos_l+i ones(1, 2*i)*(BeamPos_l+i) BeamPos_l+i-1:-1:BeamPos_l-i ones(1, 2*i-1)*(BeamPos_l-i)];
                scan_window_w = [scan_window_w ones(1, 2*i+1)*(BeamPos_w-i) BeamPos_w-i+1:BeamPos_w+i ones(1, 2*i)*(BeamPos_w+i) BeamPos_w+i-1:-1:BeamPos_w-i+1];
            end
        end
        
        if horizental_trend == -1 %横向向左运动
            if i == 1
                scan_window_l = [scan_window_l BeamPos_l:-1:BeamPos_l-1  BeamPos_l-1];
                scan_window_w = [scan_window_w  ones(1, 2)*(BeamPos_w-1) BeamPos_w];
            else
                scan_window_l = [scan_window_l BeamPos_l:-1:BeamPos_l-i ones(1, i)*(BeamPos_l-i)];
                scan_window_w = [scan_window_w ones(1, i+1)*(BeamPos_w-i) BeamPos_w-i+1:BeamPos_w];
            end
        end
        
        if horizental_trend == 1 %横向向右运动
            if i == 1
                scan_window_l = [scan_window_l BeamPos_l:BeamPos_l+1  BeamPos_l+1];
                scan_window_w = [scan_window_w  ones(1, 2)*(BeamPos_w-1) BeamPos_w];
            else
                scan_window_l = [scan_window_l BeamPos_l:BeamPos_l+i ones(1, i)*(BeamPos_l+i)];
                scan_window_w = [scan_window_w ones(1, i+1)*(BeamPos_w-i) BeamPos_w-i+1:BeamPos_w];
            end
        end
    end
    
    
end
[scan_window_l,scan_window_w] = passfilterWindow(scan_window_l, 1, max_pos_l, scan_window_w, 1,max_pos_w );
end

