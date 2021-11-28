function judgement = ikine_judgement(T,d,a)
% 判断T位姿是否存在逆解
% 综合解析法与数值法，在奇异位姿处使用数值法逆解
% judgement:判断值,1有解，0无解
% T:位姿矩阵
% d,a:机械臂参数

%     a=[0,408,376,0,0,0];
%     d=[121.5,140.5,-121.5,102.5,102.5,94];
    
    nx=T(1,1);ny=T(2,1);nz=T(3,1);
    ox=T(1,2);oy=T(2,2);oz=T(3,2);
    ax=T(1,3);ay=T(2,3);az=T(3,3);
    px=T(1,4);py=T(2,4);pz=T(3,4);
    
    %求解关节角1
    m=d(6)*ay-py;  n=ax*d(6)-px; 
    
    if ~isreal(sqrt(m^2+n^2-(d(2)+d(3)+d(4))^2))
        %theta1无解，舍去
        judgement=0;
%     elseif m^2+n^2-(d(2)+d(3)+d(4))^2==0
%         %处于奇异位姿，使用数值法求解
%         judgement=sub_judgement(T,d,a);
    else
        %theta1有解，继续逆解步骤
        theta1(1,1)=atan2(m,n)-atan2(d(2)+d(3)+d(4),sqrt(m^2+n^2-(d(2)+d(3)+d(4))^2));  
        theta1(1,2)=atan2(m,n)-atan2(d(2)+d(3)+d(4),-sqrt(m^2+n^2-(d(2)+d(3)+d(4))^2));
        
        if any((ax*sin(theta1)-ay*cos(theta1))>1) || any((ax*sin(theta1)-ay*cos(theta1))<-1)
            %acos运算要求-1到1，不符合舍去
            judgement=0;
%         elseif any((ax*sin(theta1)-ay*cos(theta1))==1) || any((ax*sin(theta1)-ay*cos(theta1))==-1)
%             %对应sin(theta5)==0的奇异位姿，使用数值法求解
%             judgement=sub_judgement(T,d,a);
        else
            %求解关节角5
            theta5(1,1:2)=acos(ax*sin(theta1)-ay*cos(theta1));
            theta5(2,1:2)=-acos(ax*sin(theta1)-ay*cos(theta1));  
            
            %求解关节角6
            mm=nx*sin(theta1)-ny*cos(theta1); nn=ox*sin(theta1)-oy*cos(theta1);
            theta6=atan2(mm,nn)-atan2(-sin(theta5),0);
            
            %求解关节角3
            mmm=-d(5)*(sin(theta6).*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6).*(ox*cos(theta1)+oy*sin(theta1))) ...
                -d(6)*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1);
            nnn=pz-d(1)-az*d(6)-d(5)*(oz*cos(theta6)+nz*sin(theta6));
            
            if any(any(((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)))>1)) || any(any(((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)))<-1))
                %acos运算要求-1到1，不符合舍去
                judgement=0;
%             elseif any(any(((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)))==1)) || any(any(((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)))==-1))
%                 %奇异位姿，使用数值法计算
%                 judgement=sub_judgement(T,d,a);
            else
                %无需计算，存在逆解
                judgement=1;
            end
        end
    end
end

