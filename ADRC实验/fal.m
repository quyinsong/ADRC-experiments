function y = fal( x,a,deta )
%FAL 误差变换函数
% Date : 10th March 2022
% 跟踪微分器（TD）实验
% 参考文献：非线性跟踪-微分器  作者：韩京清
if deta<=0
    error('deta must be > 0!');end
if abs(x) >= deta
   y = abs(x)^a*sign(x);
else
    y = x/deta^(1-a);
end

end
