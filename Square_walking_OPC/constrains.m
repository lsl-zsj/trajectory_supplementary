function [c,ceq]=constrains(x)


ceq1=x(1)^2+x(2)^2+x(3)^2+x(4)^2-1;
ceq2=x(5)^2+x(6)^2+x(7)^2+x(8)^2-1;

c = [];
ceq=[ceq1;ceq2];


end