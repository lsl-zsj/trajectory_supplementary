function [out,qsab] = SAB_New_MKMC(acc, gyr, mag, t, stdAcc, stdGyro, stdMag, sigma_acc,sigma_mag)
% Sabatini 2011 - Estimating three dimensional orientation of human body parts by inertial-magnetic sensing (Sensors, 2011)
% --------------- INPUT ---------------
% acc            = Nx3 (m/s^2)
% gyr            = Nx3 (rad/s)
% mag            = Nx3 (a.u.) normalized units
% t              = Nx1 (s)
% stdAcc         = 1x1 (m/s^2) inverse weight to assign to the accelerometer measurements
% stdGyro        = 1x1 (rad/s) inverse weight to assign to the gyroscope measurements
% stdMag         = 1x1 (a.u.)  inverse weight to assign to the magnetometer measurements
% thAcc          = 1x1 (m/s^2) threshold value for accelerometer signal
% thMag          = 1x1 (a.u.) threshold value for magnetometer signal
% sigma_acc      = kernel bandwidth for the accelerometer
% sigma_mag      = kernel bandwidth for the magnetometer       
% --------------- OUTPUT ---------------
% qsab           = Nx4 [qx qy qz qw], the scalar part is at the END of the quaternion
% Accelerometer data
ax = -acc(:,1); ay = -acc(:,2); az = -acc(:,3);
% Gyroscope data
wx = gyr(:,1); wy = gyr(:,2); wz = gyr(:,3);
% Magnetometer data
hx = mag(:,1); hy = mag(:,2); hz = mag(:,3);
xPriori=zeros(4,length(t));
xPost=zeros(4,length(t));
sigma_y=[sigma_acc*ones(3,1);sigma_mag*ones(3,1)]; % kernel bandwidth vector
%[qin,qmag,qacc,L] = initQuaternion(-acc(1,:),mag(1,:)); % reference magnetic vector
%L=[L(1)^2+L(2)^2,0,L(3)]';
% if ~exist('qin','var')
%     [qin,~,~,L] = initialEKFquat(-acc(1,:),mag(1,:)); % 
%     qin=[-qin(2:end); qin(1)]; %da loc a glob
% else
%     [~,~,~,L] = initialEKFquat(-acc(1,:),mag(1,:));
% end
%% init
accr=-acc(1,:); % gravity neagtive 
magr=mag(1,:);
% NED coordinate
r_down=accr';
r_east=cross(accr',magr');
r_north= cross(r_east, r_down);
r_down=r_down/norm(r_down);
r_east=r_east/norm(r_east);
r_north=r_north/norm(r_north);
% R_*g=accr'   R_*m=magr'
R_=[r_north,r_east,r_down]; % rotation matrix of earth frame to sensor frame
Q__ = quaternion(R_, 'rotmat', 'frame');
Q__ =compact(Q__); % 
qin=[Q__(2:4),Q__(1)]; % this is identical to qin from "initQuaternion"
L=R_'*magr';  % reference magnetic vector
if isrow(qin)
    qin=transpose(qin);
end
xPost(1:4,1)=qin; %
%PROCESS NOISE
SIGMA_g=stdGyro^2*eye(3);
%Constants
g=9.81;
h=[sqrt(L(1).^2+L(2).^2);0;L(3)]; %Earth's magnetic field (global Frame)
dt=mean(diff(t)); %only to initialize the SIGMA_m matrix
%SIGMA_m=dt*stdBiasMag_in^2*eye(3);
CSI=[[0 -xPost(3,1) xPost(2,1);
    xPost(3,1) 0 -xPost(1,1)
    -xPost(2,1) xPost(1,1) 0]+xPost(4,1)*eye(3);...
    -xPost(1:3,1)'];
Q=[(dt/2)^2*CSI*SIGMA_g*CSI'];
Ppost=Q; %posterior initial guess covariance error matrix
%Accelerometer
std_acc=stdAcc;
%Magnetometer
std_mag=stdMag;
R=[std_acc^2*eye(3) zeros(3);
   zeros(3) std_mag^2*eye(3)];
% Chol decomposition
Bra=std_acc; % chol decomposition of std_acc^2
Brm=std_mag; % chol decomposition of std_mag^2
br=[Bra*eye(3) zeros(3);
        zeros(3) Brm*eye(3)];
warning off
for i=1:length(t)-1
    %dt = t(i+1) - t(i);
    dt=1/400;
    % PREDICTION STEP
    omega=0.5*[0 wz(i) -wy(i) wx(i)
        -wz(i) 0 wx(i) wy(i)
        wy(i) -wx(i) 0 wz(i)
        -wx(i) -wy(i) -wz(i) 0];%skew symmetric see. eq.38

  %  F=[eye(4)+omega*dt+0.5*(omega*dt)^2 zeros(4,3)
  %      zeros(3,4) eye(3)]; %linearized to increase computational speed
    F=[expm(omega*dt)];
    %Project the state ahead
    xPriori(:,i)=F*xPost(:,i);
    
    CSI=[[0 -xPost(3,i) xPost(2,i);
        xPost(3,i) 0 -xPost(1,i)
        -xPost(2,i) xPost(1,i) 0]+xPost(4,i)*eye(3);...
        -xPost(1:3,i)'];  % eq.74 
        
    Q=[(dt/2)^2*CSI*SIGMA_g*CSI']; % process covariance
    
    %Compute the a priori covariance matrix
    Ppriori= F*Ppost*F'+Q;
    %% UPDATE STEP
    % if(0)
    %  q_last=quaternion([xPriori(4,i),xPriori(1,i),xPriori(2,i),xPriori(3,i)]);
    %  mq=quaternion([0,mag(i,:)]);
    %  h_Earth=q_last*mq*conj(q_last);% h = quaternProd(q, quaternProd([0 Magnetometer], quaternConj(q)));
    %  h_Earth=compact(h_Earth);
    %  h = [norm([h_Earth(2) h_Earth(3)]) 0 h_Earth(4)]';
    % end
     %% Linearize the measurement equation: Jacobian
    q1=xPriori(1,i);
    q2=xPriori(2,i);
    q3=xPriori(3,i);
    q4=xPriori(4,i);
    H=[2*g*[q3 -q4 q1 -q2
        q4 q3 q2 q1
        -q1 -q2 q3 q4];
        2*[q1*h(1)+q3*h(3) -q2*h(1)-q4*h(3) -q3*h(1)+q1*h(3) q4*h(1)-q2*h(3)
        q2*h(1)+q4*h(3) q1*h(1)+q3*h(3) -q4*h(1)+q2*h(3) -q3*h(1)+q1*h(3)
        q3*h(1)-q1*h(3) q4*h(1)-q2*h(3) q1*h(1)+q3*h(3) q2*h(1)+q4*h(3)]
        ]; % eq.24
    C=[q1^2-q2^2-q3^2+q4^2 2*(q1*q2+q3*q4) 2*(q1*q3-q2*q4)
        2*(q1*q2-q3*q4) -q1^2+q2^2-q3^2+q4^2 2*(q2*q3+q1*q4)
        2*(q1*q3+q2*q4) 2*(q2*q3-q1*q4) -q1^2-q2^2+q3^2+q4^2]; % eq.24
    z_predict=[C zeros(3); zeros(3) C]*[0;0;g;h];
    %% this is MKMC part
    z=[ax(i);ay(i);az(i);hx(i);hy(i);hz(i)]; %measurement vector
    ze=z-z_predict; % innovation
    er=br\ze;
    Er(i,:)=ze;
    Ernorm(i,:)=er;
    cnt=2;
    num=cnt;
    while(num>0)
       if(num==cnt)
        X_tlast=xPriori(:,i); 
       else  
        X_tlast=X_t; 
       end
       % y-g(x)
       q1=X_tlast(1);
       q2=X_tlast(2);
       q3=X_tlast(3);
       q4=X_tlast(4);
       C=[q1^2-q2^2-q3^2+q4^2 2*(q1*q2+q3*q4) 2*(q1*q3-q2*q4)
        2*(q1*q2-q3*q4) -q1^2+q2^2-q3^2+q4^2 2*(q2*q3+q1*q4)
        2*(q1*q3+q2*q4) 2*(q2*q3-q1*q4) -q1^2-q2^2+q3^2+q4^2]; % eq.24
       z_pre=[C zeros(3); zeros(3) C]*[0;0;g;h];
       ze=z-z_pre; % innovation
       er=br\ze;
       diay=exp(-er.*er./sigma_y);
       for k=1:6
        if(diay(k)<1e-8)
           diay(k)=1e-8;
        end
       end
       Cy=diag(diay);
       R_1=br/Cy*br';
       %Compute the Kalman Gain
       K_1=Ppriori*H'*(H*Ppriori*H'+R_1)^-1;
       X_t=xPriori(:,i)+K_1*(z-z_predict); 
       num=num-1;
       thresh=norm(X_t-X_tlast)/(norm(X_tlast)+1e-3);
       THE(i,cnt-num)=thresh;
       if(thresh<1e-6)
         break;
       end
    end
    xPost(:,i+1)=X_t;
    xPost(1:4,i+1)=xPost(1:4,i+1)/norm(xPost(1:4,i+1));%normailize
    Ppost=(eye(4)-K_1*H)*Ppriori*(eye(4)-K_1*H)'+K_1*R*K_1';
    % Measurement covariance
    %R=[std_acc^2*eye(3) zeros(3);
     %   zeros(3) std_mag^2*eye(3)];
    %Compute the Kalman Gain
    %K=Ppriori*H'*(H*Ppriori*H'+R)^-1;
    
    %Compute the a posteriori state estimate
    %z_predict=[C zeros(3); zeros(3) C]*[0;0;g;h];
    %xPost(:,i+1)=xPriori(:,i)+K*(z-z_predict);

    %if(i==length(t)-100)
    %    xPost(:,i+1)=qin;
    %end
    % store the data
    QQ(:,:,i)=Q;
    FF(:,:,i)=F;
    statef_(:,i)=xPriori(:,i);
    statef(:,i)=X_t;
    covf_(:,:,i)=Ppriori;
    covf(:,:,i)=Ppost;
end




qsab=xPost(1:4,:)';
out.THE=THE;
out.Er=Er;
out.Ernorm=Ernorm;
out.h=h;
out.g=g;

%
out.Q=QQ;
out.F=FF;
out.statef_=statef_;
out.statef=statef;
out.covf_=covf_;
out.covf=covf;
end


function [z,qacc,qmag,l] = initialEKFquat(acc,mag)
% Valenti 2015 - Keeping a Good Attitude: A Quaternion-Based Orientation
% Filter for IMUs and MARGs (Sensors, 2015)

% Implementation: Marco Caruso (Politecnico di Torino) 
% Date: 21/11/2019

% --------------- INPUT ---------------
% acc            = 1x3 (m/s^2)
% mag            = 1x3 (a.u.) normalized units

% --------------- OUTPUT ---------------
% z              = 1x4 [qw qx qy qz], the scalar part is at the BEGINNING of the quaternion
% qacc           = 1x4 [qw qx qy qz], the scalar part is at the BEGINNING of the quaternion
% qmag           = 1x4 [qw qx qy qz], the scalar part is at the BEGINNING of the quaternion
% l              = 1x3 measured field rotated in the global horizontal plane

n=norm(acc);
ax=acc(1)/n; ay=acc(2)/n; az=acc(3)/n;
if az>=0
    qacc=[sqrt((az+1)/2) -ay/sqrt(2*(az+1)) ax/sqrt(2*(az+1)) 0]';
else
    qacc=[-ay/sqrt(2*(1-az)) sqrt((1-az)/2) 0 ax/sqrt(2*(1-az))]';
end

hx=mag(1); hy=mag(2); hz=mag(3);
l=quatrotmatr(qacc)'*[hx;hy;hz];
T=l(1)^2+l(2)^2;
if l(1)>=0
    qmag=[sqrt(T+l(1)*sqrt(T))/sqrt(2*T);...
        0;0;...
        l(2)/(sqrt(2)*sqrt(T+l(1)*sqrt(T)))];
else
    qmag=[l(2)/(sqrt(2)*sqrt(T-l(1)*sqrt(T)));...
        0;0;...
        sqrt(T-l(1)*sqrt(T))/sqrt(2*T)];
end
z=quatmultiply(qacc',qmag');

end
