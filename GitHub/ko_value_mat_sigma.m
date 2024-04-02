function J=ko_value_mat_sigma(xx)
 
 global w
 EE=w;
 maxv=max(abs(w));
 g = @(sigma,l) (integral(@(x) 1/l*exp(- sigma^2*(1-exp(-x.^2/(2*sigma^2*l^2)))),-5,5));
 logpe=@(x,sigma,l) (sigma^2*(1-exp(-x.^2/(2*sigma^2*l^2))));
 
 SigmaJ=xx(1);
 l=1;
 %obtain the candidate pdf using Sigma and l
 c=g(SigmaJ,l);
 ci=1/c;
 Nlogci=length(EE)*(log(ci)-log(l));
 logL=logpe(EE,SigmaJ,l);
 J=-sum(logL)+Nlogci;
 % convert maximization to minimization
 J=-J;  
end