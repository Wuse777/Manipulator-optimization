function [theta,N] = single_point_theta(d,a,point,n,m)
%single_point_theta 计算机械臂单点灵活度
%利用旋转矩阵遍历计算点中所有z轴方向，绕定系旋转，先绕x轴旋转0-pi范围，再绕z轴旋转0-2*pi范围
%在获得一个符合z轴方向的末端坐标系后，还需要对z轴旋转2*pi进行搜索
% d,a机械臂参数
% point:坐标点，为1x3的矩阵
% n:x轴旋转方向数目
% m:z轴旋转角度数目
% theta:该点的灵活度
% N:总样本数，为了保证角度分布均匀，pi是线性的，而2*pi是头尾相接的，故有所不同
% 按照下述方法所得N=2*n^2-6*n+6

N=0;

%计数变量
count=0;

for i=0:n-1
    for j=0:(2*n-3)
        flag=0;
        rot_xz=rotz(j/(2*n-2)*2*pi)*rotx(i/(n-1)*pi)*eye(3);%xy旋转
        for k=1:m
            if m==1 
                %m为1，不旋转
                rot_xyz=rot_xz;
            else
                rot_xyz=rot_xz*rotz(k/(m-1)*2*pi);
            end
            T=eye(4);
            T(1:3,1:3)=rot_xyz;
            T(1:3,4)=point;
            %判断逆解
            
            flag=ikine_judgement(T,d,a);
        end
        if flag==1
            count=count+1;
        end
        N=N+1;
        %z与定系重合，不需要旋转
        if (i==0||i==n-1)
            break
        end
    end      
end
theta=count/N;

end

