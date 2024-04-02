function kf=kfInstanceInit(mkc_sigma,c_sigma,kf,type)

switch type
    case 'process'
    sigmau=mkc_sigma;
    sigma_p=[sigmau sigmau]';
    sigma_r=[1000 1000]';
    sigma_pb=[1000 1000]';
    sigma_qb=[sigmau sigmau]';
    kf.sigma_p=sigma_p;
    kf.sigma_r=sigma_r;
    kf.sigma_pb=sigma_pb;
    kf.sigma_qb=sigma_qb;
    kf.sigma=c_sigma;
    case 'measurement'
    sigma_p=[1000 1000]';
    sigma_r=[mkc_sigma mkc_sigma]';
    sigma_pb=[1000 1000]';
    sigma_qb=[1000 1000]';
    kf.sigma_p=sigma_p;
    kf.sigma_r=sigma_r;
    kf.sigma_pb=sigma_pb;
    kf.sigma_qb=sigma_qb;
    kf.sigma=c_sigma;  
end

end