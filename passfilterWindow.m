function [scan_window_l,scan_window_w]  = passfilterWindow(window_l,min_l,max_l, window_w,min_w, max_w)
%过滤超出边界的window position出现
scan_window_l = [];
scan_window_w = [];
for i=1:length(window_l)
    if(window_l(i)>=min_l && window_l(i)<=max_l && window_w(i)>=min_w && window_w(i)<=max_w)
        scan_window_l = [scan_window_l window_l(i)];
        scan_window_w = [scan_window_w window_w(i)];
    end
end
end
