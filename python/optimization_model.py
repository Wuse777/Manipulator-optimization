# -*- coding: utf-8 -*- #
 
# ------------------------------------------------------------------
# File Name:        optimization_model.py
# Author:           Yuanhai Huang
# Version:          1.2
# Created:          2021/11/27
# Description:      Main Function:    mathematical model for manipulator optimization in python
#                   This file builds up the mathematical model which is consistent with Matlab version, but comparing with Matlab it runs much slower.
#                   Python is no good at processing loops. The irony is that codes for optimization methods need to be optimized. 
#                   I strongly recommend you to use Matlab version if you can.
#                   
# Function List:    ikine_judgement() -- ikine judgement for one transformation matrix
#                   single_point_theta() -- calculate posture probability coefficient for one position
#                   fitness() -- calculate dexterous workspace volume with certain structsize and return fitness value
# History:
#       <author>        <version>       <time>  
#       Yuanhai Huang  1.1         2021/11/27    
#       Zhi Li         1.0         2021/11/20  
# ------------------------------------------------------------------


import numpy as np

from scipy.spatial.transform import Rotation as R
import time

import numba as nb

def ikine_judgement(T,d,a):
    nx,ny,nz=T[0,0],T[1,0],T[2,0]
    ox,oy,oz=T[0,1],T[1,1],T[2,1]
    ax,ay,az=T[0,2],T[1,2],T[2,2]
    px,py,pz=T[0,3],T[1,3],T[2,3]
    m=d[5]*ay-py
    n=ax*d[5]-px
    
    if m**2+n**2-(d[1]+d[2]+d[3])**2<0:
        judgement=0
    else:
        theta1=np.zeros((1,2))
        theta1[0,0]=np.arctan2(m,n)-np.arctan2(d[1]+d[2]+d[3],np.sqrt(m**2+n**2-(d[1]+d[2]+d[3])**2)) 
        theta1[0,1]=np.arctan2(m,n)-np.arctan2(d[1]+d[2]+d[3],-np.sqrt(m**2+n**2-(d[1]+d[2]+d[3])**2))
        if np.any((ax*np.sin(theta1)-ay*np.cos(theta1))>1) or np.any((ax*np.sin(theta1)-ay*np.cos(theta1))<-1):
            judgement=0
        else:
            theta5=np.zeros((2,2))
            theta5[0,:]=np.arccos(ax*np.sin(theta1)-ay*np.cos(theta1))
            theta5[1,:]=-np.arccos(ax*np.sin(theta1)-ay*np.cos(theta1))
            mm=np.repeat(nx*np.sin(theta1)-ny*np.cos(theta1),axis=0,repeats=1)
            nn=np.repeat(ox*np.sin(theta1)-oy*np.cos(theta1),axis=0,repeats=1)
            theta6=np.zeros((2,2))
            theta6=np.arctan2(mm,nn)-np.arctan2(-np.sin(theta5),0)
            mmm=-d[4]*((np.sin(theta6)*(nx*np.cos(theta1)+ny*np.sin(theta1)))+np.cos(theta6)*(ox*np.cos(theta1)+oy*np.sin(theta1)))-\
                    d[5]*(ax*np.cos(theta1)+ay*np.sin(theta1))+px*np.cos(theta1)+py*np.sin(theta1)
            nnn=pz-d[0]-az*d[5]-d[4]*(oz*np.cos(theta6)+nz*np.sin(theta6))
            if np.any(((mmm**2+nnn**2-a[1]**2-a[2]**2)/(2*a[1]*a[2]))>1) or np.any(((mmm**2+nnn**2-a[1]**2-a[2]**2)/(2*a[1]*a[2]))<-1):
                judgement=0
            else:
                judgement=1
    return judgement

def single_point_theta(d,a,point,n,m):


    N=0
    count=0

    # caution:difference in 'for' between matlab and python
    # in matlab using a:b includes b
    # in python using range(a,b) does not include b
    for i in range(0,n):
        for j in range(0,2*n-2):
            flag=0
            rot_tmp1=R.from_euler('z',j/(2*n-2)*2*np.pi)
            rot_tmp2=R.from_euler('x',i/(n-1)*np.pi)
            rot_xz=np.dot(np.asarray(rot_tmp1.as_matrix()),np.asarray(rot_tmp2.as_matrix()))
            for k in range(1,m+1):
                if m==1:
                    rot_xyz=rot_xz
                else:
                    rot_tmp3=R.from_euler('z',k/(m-1)*2*np.pi)
                    rot_xyz=np.dot(rot_xz,np.asarray(rot_tmp3.as_matrix()))
                T=np.eye(4)
                T[0:3,0:3]=rot_xyz
                T[0:3,3]=point
                flag=ikine_judgement(T,d,a)
            if flag==1:
                count=count+1
            N=N+1
            if (i==0 or i==n-1):
                break
            
    theta=count/N
    return theta

def fitness(x):
    a=[0,408,376,0,0,0]
    d=[121.5,140.5,-121.5,102.5,102.5,94]
    d[0]=x[0];a[1]=x[1];a[2]=x[2];d[3]=x[3];d[4]=x[4];d[5]=x[5]
    N_workspace=0
    N_cal=0
    L_max=1200
    n_length=20
    N=0
    # caution:difference in 'for' between matlab and python
    # in matlab using a:b includes b
    # in python using range(a,b) does not include b
    for i in range(0,n_length*2+1):
        for j in range(0,n_length*2+1):
            for k in range(0,n_length*2+1):
                point=np.array([i,j,k])/(n_length*2)*2*L_max-L_max
                if np.linalg.norm(point)<L_max:
                    N=N+1
    N_total_cal=0
    for i in range(0,n_length+1):
        for j in range(0,n_length+1):
            for k in range(0,n_length*2+1):
                point=np.array([i/(n_length)*L_max,j/(n_length)*L_max,k/(n_length*2)*2*L_max-L_max])
                if np.linalg.norm(point)<L_max:
                    N_total_cal=N_total_cal+1
    lambda_data=np.zeros((N,4))
    n_single_point_direction=6
    n_single_point_direction_count=2*n_single_point_direction**2-6*n_single_point_direction+6
    m_rotz=6
#    print(lambda_data)
    for i in range(0,n_length+1):
        for j in range(0,n_length+1):
            for k in range(0,n_length*2+1):
                point=np.array([i/(n_length)*L_max,j/(n_length)*L_max,k/(n_length*2)*2*L_max-L_max])
                if np.linalg.norm(point)<L_max:
                    N_cal=N_cal+1
                    theta=single_point_theta(d,a,point,n_single_point_direction,m_rotz)
                    if theta>0:
                        lambda_data[N_workspace,:]=np.array([point[0],point[1],point[2],theta])
                        N_workspace=N_workspace+1
                        if ((i/(n_length)*L_max>0) and (j/(n_length)*L_max>0)):
                             lambda_data[N_workspace:N_workspace+3,:]=np.array([[-point[0],point[1],point[2],theta],[point[0],-point[1],point[2],theta],[-point[0],-point[1],point[2],theta]])
                             N_workspace=N_workspace+3
                        elif (i/(n_length)*L_max==0) and (not(j/(n_length)*L_max==0)):
                             lambda_data[N_workspace,:]=np.array([[point[0],-point[1],point[2],theta]])
                             N_workspace=N_workspace+1
                        elif (not (i/(n_length)*L_max==0)) and (j/(n_length)*L_max==0):
                             lambda_data[N_workspace,:]=np.array([[-point[0],point[1],point[2],theta]])
                             N_workspace=N_workspace+1
    
    if np.all(lambda_data==0):
        result=np.inf
    else:
        lambda_data=lambda_data[~(lambda_data == 0).all(1)]
        result=-sum(lambda_data[:,3]==1)*(L_max/n_length)**3*10**(-9)

    return result

if __name__ == "__main__":
    print('hello world')

    print("test for ikine_judgement")
    T=np.array([[1,0,0,400],[0,1,0,400],[0,0,1,800],[0,0,0,1]])
    print(T)
    a=[0,408,376,0,0,0]
    d=[121.5,140.5,-121.5,102.5,102.5,94]
    print(ikine_judgement(T,d,a))

    print("test for single_point_theta")
    point=np.array([600,400,400])
    print(single_point_theta(d,a,point,6,6))

    print("test for fitness")
    time_start=time.time()
    #x=[121.5,408,376,102.5,102.5,94]
    x=[100.0483, 402.2005, 424.8482, 93.4348, 80.9030, 93.9749]
    print(fitness(x))
    time_end=time.time()
    print('fitness cost',time_end-time_start)





