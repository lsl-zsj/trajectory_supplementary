function J = nonlinear_func(x)

global q_mc_ q_imu_
a=x(1:4);
b=x(5:8);
a_q=quaternion(a);
b_q=quaternion(b);
len=length(q_mc_);

for i=1:len
    target=q_mc_(i,:).*a_q;
    measurement=b_q.*q_imu_(i,:);
    target_m=compact(target);
    measurement_m=compact(measurement);
    err1=target_m-measurement_m;
    err2=target_m+measurement_m;
    E1=err1*err1';
    E2=err2*err2';
    E(i)=min(E1,E2);
end


J=double(sum(E));



end