clc
clear all
close all
map_length = 160;%探测区域长度
map_width = 40;%探测区域宽度

%运动模型设置
R0_l = 50; %横轴初始距离
v0_l = 3; %横轴初始速度
a0_l = 0; %横轴加速度

R0_w=40; %纵轴初始距离
v0_w=-2; %纵轴初始速度
a0_w=-5;  %纵轴加速度
%T_b = 0.592;
allow_T = 1.5;
time_num = 15;
%智能波束方案
[s_TRACK_L,s_TRACK_W,global_count] = smartbeam(time_num,map_length,map_width,R0_l,v0_l,a0_l,R0_w,v0_w,a0_w,allow_T);
%大波束扫描方案
[b_Track_l, b_Track_w] = bigscan(time_num,map_length,map_width,R0_l,v0_l,a0_l,R0_w,v0_w,a0_w);
% %大波束搜索方案
[bs_Track_l, bs_Track_w,T_b] = bigsearch(time_num,map_length,map_width,R0_l,v0_l,a0_l,R0_w,v0_w,a0_w,allow_T);
% 
% %结果对比
hold on
plot(bs_Track_l,bs_Track_w,'c+');
hold on
plot(b_Track_l,b_Track_w,'kV');
%  %xlabel('探测区域横向坐标/m');
% % ylabel('探测区域纵向坐标/m');
% % legend('物体理论运动轨迹','大波束跟踪','智能波束扫描','大波束扫描','Location','northwest');
xlabel('Horizontal distance/m');
ylabel('Vertical distance/m');
legend('Object theory track','Smart beam tracking scheme','Wide beam tracking scheme','Wide beam scanning scheme','Location','northeast');
axis([0 map_length 0 map_width]);
text(80, 80, ['Horizontal length of detection area: ',num2str(map_length),'m']);
text(80, 18, ['Longitudinal length of detection area: ',num2str(map_width),'m']);

text(80, 16, ['Horizontal initial  distance: ',num2str(R0_l),'m']);
text(80, 14, ['Horizontal initial  velocity: ',num2str(v0_l),'m/s']);
text(80, 12, ['Horizontal  acceleration: ',num2str(a0_l),'m/s2']);

text(80, 10, ['Longitudinal initial distance: ',num2str(R0_w),'m']);
text(80, 08, ['Longitudinal initial velocity: ',num2str(v0_w),'m/s']);
text(80, 6, ['Longitudinal acceleration: ',num2str(a0_w),'m/s2']);
