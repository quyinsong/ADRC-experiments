% Date : 10th March 2022
% 非线性PID控制器实验
% 参考文献：非线性PID控制器  作者：韩京清
% 仿真示例： 例2
clc
clear all
close all
% 数值仿真方法： 前向欧拉法
ts = 0.01;
tfinal = 30;
Ns = tfinal/ts;
% initial
x = [0 0]'; % palnt state
x1 = [0 0]'; x2 = [0 0]'; % TD state
R = 3; deta = 0.0005*R;
z1 = 0;
% simulation
for k = 1:1:Ns
    % 期望跟踪信号
    vd = sign(10-k*ts);
    % TD1 对期望信号进行跟踪
    x1_d = [x1(2);
            -R*sat0(x1(1)-vd+abs(x1(2))*x1(2)/(2*R),deta)];
    x1 = euler2(x1_d,x1,ts);
    % TD2 对反馈信号进行跟踪
    y = x(1);
    x2_d = [x2(2);
            -R*sat0(x2(1)-y+abs(x2(2))*x2(2)/(2*R),deta)];
    x2 = euler2(x2_d,x2,ts);  
    % 计算误差
    z1_d = x1(1) - x2(1);
    z1 = euler2(z1_d,z1,ts); % 误差积分
    z2 = x1(1) - x2(1); % 误差
    z3 = x1(2) - x2(2); % 误差微分
    % 误差非线性变换
    z11 = fal(z1,1,0.5);
    z22 = fal(z2,1,0.5);
    z33 = fal(z3,1,0.5);
    % PID控制器
    k1 = 0; k2 = 4; k3 = 2;
    u = k1*z11+k2*z22+k3*z33;
    % plant
    x_d = [-0.2*x(1)^2-0.1*x(1)*x(2)+x(2);
         0.5/(1+x(2))^2+0.2*x(1)*exp(1+abs(x(1)/10))+u];
    x = euler2(x_d,x,ts);
    % 输出状态
    xout(k,:) = x';
    vdout(k) = vd;
    tout(k) = (k-1)*ts;
end
% plot
figure()
plot(tout,xout(:,1),'r-',tout,vdout,'b--','linewidth',2)
legend('跟踪信号x1','期望信号vd');