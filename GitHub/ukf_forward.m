function ukff=ukf_forward(kf,u,z)

% kf : the kalman fitler instance 
Q=kf.Q;
R=kf.R;
F=kf.F;
H=kf.H;
G=kf.G; % 
len=kf.len;
n=kf.n;  %numer of states
m=kf.m;  %numer of measurements
% stored state
statef_=zeros(n,len);
statef=zeros(n,len);
covf_=zeros(n,n,len);
covf=zeros(n,n,len);

%%
% Reference: Julier, SJ. and Uhlmann, J.K., Unscented Filtering and
% Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3, pp.401-422, 2004. 
% Maximum Correntropy Unscented Filter

%% obtain weights
alpha=1e-3;                                 %default, tunable
ki=3-n;   % or 0                                   %default, tunable (3-n)
beta=2;                                     %default, tunable
lambda = alpha^2*(n+ki)-n;                   %scaling factor
c = n+lambda;                                %scaling factor
Wm=[lambda/c 0.5/c+zeros(1,2*n)];           %weights for means
Wc=Wm;
Wc(1)=Wc(1)+(1-alpha^2+beta);               %weights for covariance
sqr_c= sqrt(c);
for i=1:len
    if(i==1)
        x=kf.x0;
        P=kf.P0;
    end
    X_sig=sigmas(x,P,sqr_c);        %sigma points around x
    %% prediction
    sig_n=2*n+1; % sigma point number
    x_pre=zeros(n,1);
    X_pre=zeros(n,sig_n);
    for k=1:sig_n                 
        X_pre(:,k)=F*X_sig(:,k)+G*u(:,i);  % nonlinear transformation      
        x_pre=x_pre+Wm(k)*X_pre(:,k);       
    end
    Xe=X_pre-x_pre(:,ones(1,sig_n));
    Px=Xe*diag(Wc)*Xe'+Q; 
    x_=x_pre; % assignment
    P_=Px; % assignment
    %% measurement
    y_pre=zeros(m,1);
    Y_pre=zeros(n,sig_n);
    for k=1:sig_n                 
        Y_pre(:,k)=H*X_sig(:,k);  % nonlinear transformation      
        y_pre=y_pre+Wm(k)*Y_pre(:,k);       
    end
    Ye=Y_pre-y_pre(:,ones(1,sig_n));
    Py=Ye*diag(Wc)*Ye'+R; 
    %% fusion
    Pxy=Xe*diag(Wc)*Ye';  
    K=Pxy/(Py);
    x=x_+K*(z(:,i)-y_pre);
    P=P_-K*Py*K';    
    % store the data
    statef_(:,i)=x_';
    statef(:,i)=x';
    covf_(:,:,i)=P_;
    covf(:,:,i)=P;
end

ukff.statef_=statef_;
ukff.statef=statef;
ukff.covf_=covf_;
ukff.covf=covf;


end




function [y,Y,P,Y1]=ut(f,X,Wm,Wc,n,R)
%Unscented Transformation
%Input:
%        f: nonlinear map
%        X: sigma points
%       Wm: weights for mean
%       Wc: weights for covraiance
%        n: numer of outputs of f
%        R: additive covariance
%Output:
%        y: transformed mean
%        Y: transformed smapling points
%        P: transformed covariance
%       Y1: transformed deviations

L=size(X,2);
y=zeros(n,1);
Y=zeros(n,L);
for k=1:L                   
    Y(:,k)=f(X(:,k));       
    y=y+Wm(k)*Y(:,k);       
end
Y1=Y-y(:,ones(1,L));
P=Y1*diag(Wc)*Y1'+R;          
end

function X=sigmas(x,P,c)
%Sigma points around reference point
%Inputs:
%       x: reference point
%       P: covariance
%       c: coefficient
%Output:
%       X: Sigma points

A = c*chol(P)';      % A'A=P  (sqrt of P)
Y = x(:,ones(1,numel(x)));
X = [x Y+A Y-A]; 
end
