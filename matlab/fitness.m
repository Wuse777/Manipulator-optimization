function result = fitness(x)
%目标函数
%为1/lambda

%机械臂参数
a=[0,408,376,0,0,0];
d=[121.5,140.5,-121.5,102.5,102.5,94];

%优化6段长
d(1)=x(1);a(2)=x(2);a(3)=x(3);d(4)=x(4);d(5)=x(5);d(6)=x(6);

%优化2段长
% a(2)=x(1);a(3)=x(2);


%计数变量
N_workspace=0;
N_cal=0;

%定义计算空间最大距离
L_max=1200;

%均分边长为L_max*2的立方体,且仅取立方体在半径为L_max球的部分以减少无效点计算
%进一步减少计算量，在不考虑关节角限制的情况下以xoz,yoz平面切割球体计算1/4球

n_length=20; %在卦限内取点的间隔，实际上为n_length+1个点
N=0;
%计算总点数
for i=0:n_length*2
    for j=0:n_length*2
        for k=0:n_length*2
            point=[i,j,k]/(n_length*2)*2*L_max-L_max;
            if norm(point)<L_max
                N=N+1;
            end
        end
    end
end

N_total_cal=0;
%计算总计算点数
for i=0:n_length
    for j=0:n_length
        for k=0:n_length*2
            point=[i/(n_length)*L_max,j/(n_length)*L_max,k/(n_length*2)*2*L_max-L_max];
            if norm(point)<L_max
                N_total_cal=N_total_cal+1;
            end
        end
    end
end

lambda_data=zeros(N,4);

%单点旋转取向，用于single_point_theta函数
n_single_point_direction=6;
n_single_point_direction_count=2*n_single_point_direction^2-6*n_single_point_direction+6;
m_rotz=6;


% 
% fprintf("开始运行\n计算空间总点数为%d个\n实际计算总点数为%d个\n每个工作球上的均匀样本点为%d个\n样本点对应的旋转矩阵个数为%d个\n",N,N_total_cal,n_single_point_direction_count,m_rotz);
% fprintf("==========================\n");


t1=clock;
for i=0:n_length
    for j=0:n_length
        for k=0:n_length*2
            point=[i/(n_length)*L_max,j/(n_length)*L_max,k/(n_length*2)*2*L_max-L_max];
            if norm(point)<L_max
                t2=clock;
                N_cal=N_cal+1;
%                 fprintf("开始计算第%d个点，点的坐标为[%.2f,%.2f,%.2f]\n",N_cal,point(1),point(2),point(3));
                theta=single_point_theta(d,a,point,n_single_point_direction,m_rotz);
%                 fprintf("第%d个计算点计算完成\n还需计算%d个点\n",N_cal,N_total_cal-N_cal);
                if theta>0
                    lambda_data(N_workspace+1,:)=[point theta];
                    N_workspace=N_workspace+1;
%                     fprintf("该点在工作空间内，灵活度为%.4f\n",theta);
                    if (i/(n_length)*L_max>0)&&(j/(n_length)*L_max>0)
                        lambda_data(N_workspace+1:N_workspace+3,:)=...
                            [ -point(1) point(2) point(3) theta;point(1) -point(2) point(3) theta;-point(1) -point(2) point(3) theta];
                        N_workspace=N_workspace+3;
%                         fprintf("补充其余三个对称点\n");
                    elseif (i/(n_length)*L_max==0)&&(~(j/(n_length)*L_max==0))
                        lambda_data(N_workspace+1,:)=[point(1) -point(2) point(3) theta];
                        N_workspace=N_workspace+1;
%                         fprintf("补充其余一个对称点\n");
                    elseif (~(i/(n_length)*L_max==0))&&(j/(n_length)*L_max==0)
                        lambda_data(N_workspace+1,:)=[ -point(1) point(2) point(3) theta];
                        N_workspace=N_workspace+1;
%                         fprintf("补充其余一个对称点\n");
                    end
%                     fprintf("目前计算的有效工作空间点位个数为%d\n",N_workspace);
                else
%                     fprintf("该点在工作空间外\n");
%                     fprintf("目前计算的工作空间点位个数为%d\n",N_workspace);
                end
                t3=clock;
%                 fprintf("该点计算费时: %.4f 秒\n",etime(t3,t2));
%                 fprintf("总时间: %.4f 秒\n",etime(t3,t1));
%                 fprintf("==========================\n");
            end
        end
    end
end

lambda_data(all(lambda_data==0,2),:)=[];
% fprintf("计算结束，开始绘图\n");

% 不剖切
% scatter3(lambda_data(:,1),lambda_data(:,2),lambda_data(:,3),10,lambda_data(:,4),'filled');
% grid on;
% h=colorbar;
% set(get(h,'label'),'string','姿态概率系数');

%沿yoz平面剖切，取xoy平面上半部分
% p=lambda_data(:,1)>=0;
% A=lambda_data(p,:);
% p=find(A(:,3)>=0);
% A=A(p,:);
% 
% scatter绘图
% scatter3(A(:,1),A(:,2),A(:,3),100,A(:,4),'filled');
% grid on;
% h=colorbar;
% set(get(h,'label'),'string','姿态概率系数');
% axis equal;


alpha=0.55;

if isempty(lambda_data)
    result=inf;
else
%     n_lambda=sum(lambda_data(:,4)>alpha);
%     lambda=n_lambda/N_workspace;
%     result=-lambda;
    result=-sum(lambda_data(:,4)==1)*(L_max/n_length)^3*10^(-9);
end
% fprintf("运行结束\n每个工作球上的均匀样本点为%d个\n每个z轴朝向对应的旋转矩阵个数为%d个\n",n_single_point_direction_count,m_rotz);
% printf("计算空间内的点有%d个\n",N);
% fprintf("实际计算了%d个点，其中在工作空间内的点为%d个\n",N_cal,N_workspace);
% fprintf("满足姿态概率系数>%.2f的点的个数为%d个\n",alpha,sum(lambda_data(:,4)>alpha));
% fprintf("alpha=%.2f时可操作度为%.4f\n",alpha,lambda);


end

