%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 《控制之美-卷二》 代码
% 作者：王天威，黄军魁
% 清华大学出版社
% 程序名称：Linear_Regression.m
% 程序功能：简单线性回归案例，解析解 （2.4节案例）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 程序初始化，清空工作空间，缓存，
clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%
% 定义z向量
z = [183; 175; 187; 185; 176; 176; 185; 191; 195; 185; 174; 180; 178; 170; 184];
% 定义x向量
x = [75; 71; 83; 74; 73; 67; 79; 73; 88; 80; 81; 78; 73; 68; 71];
% 扩展x向量
x = [ones(length(x),1) x];
% 定义y向量
y = zeros(2,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%
% 求解最优y
y = inv(transpose (x)*x)*transpose (x)*z;
%%%%%%%%%%%%%%%%%结果%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(1, 'position',[150 150 1000 400]);
% 定义横坐标
x_draw = 65:0.1:90;
% 散点图
scatter(x(:,2),z, 80,"r");
hold on;
% 线型图
plot (x_draw, y(2)*x_draw+ y(1));
grid on;

