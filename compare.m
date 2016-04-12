clc
clear all
close all
map_length = 160;%探测区域长度
map_width = 80;%探测区域宽度

%运动模型设置
R0_l = 10; %横轴初始距离
v0_l = 4; %横轴初始速度
a0_l = 1; %横轴加速度

R0_w=75; %纵轴初始距离
v0_w=0; %纵轴初始速度
a0_w=-2;  %纵轴加速度
%T_b = 0.592;
allow_T = 1.5;
%智能波束方案
[s_TRACK_L,s_TRACK_W,global_count] = smartbeam(map_length,map_width,R0_l,v0_l,a0_l,R0_w,v0_w,a0_w,allow_T);
%大波束扫描方案
[b_Track_l, b_Track_w] = bigscan(map_length,map_width,R0_l,v0_l,a0_l,R0_w,v0_w,a0_w);
% %大波束搜索方案
[bs_Track_l, bs_Track_w] = bigsearch(map_length,map_width,R0_l,v0_l,a0_l,R0_w,v0_w,a0_w,allow_T);
% 
% %结果对比
hold on
plot(bs_Track_l,bs_Track_w,'c+');
hold on
plot(b_Track_l,b_Track_w,'kV');
%  %xlabel('探测区域横向坐标/m');
% % ylabel('探测区域纵向坐标/m');
% % legend('物体理论运动轨迹','大波束跟踪','智能波束扫描','大波束扫描','Location','northwest');
xlabel('landscape orientation distance/m');
ylabel('vertical orientation distance/m');
legend('object theory track','Smart beam tracking scanning','Wide beam tracking scanning','Wide beam scanning the whole area in sequence','Location','northeast');
