#!/usr/bin/env python3
import numpy as np
import bisect

def read_pose_file(pose_file):
    data=[]
    with open(pose_file,'r') as f:
        next(f)
        for line in f:
            if not line.strip():
                continue
            vals=line.strip().split()
            T=float(vals[1])
            px=float(vals[2])
            py=float(vals[3])
            pz=float(vals[4])
            Rot=list(map(float,vals[9:18]))
            Rot_mat=np.array(Rot).reshape(3,3)
            data.append((T,px,py,pz,Rot_mat))
    return data

def read_imu_file(imu_file):
    data=[]
    with open(imu_file,'r') as f:
        for line in f:
            if not line.strip():
                continue
            vals=line.strip().split()
            T=float(vals[0])
            lin_acc=[float(vals[1]),float(vals[2]),float(vals[3])]
            ang_vel=[float(vals[4]),float(vals[5]),float(vals[6])]
            data.append((T,lin_acc,ang_vel))
    data=sorted(data,key=lambda x:x[0])
    return data

def interpolate_imu(imu_data,t):
    times=[d[0] for d in imu_data]
    idx=bisect.bisect_left(times,t)
    if idx==0:
        return imu_data[0][1],imu_data[0][2]
    if idx>=len(imu_data):
        return imu_data[-1][1],imu_data[-1][2]
    t1,la1,av1=imu_data[idx-1]
    t2,la2,av2=imu_data[idx]
    w=(t-t1)/(t2-t1)
    la=np.array(la1)*(1-w)+np.array(la2)*w
    av=np.array(av1)*(1-w)+np.array(av2)*w
    return la,av

def exp_map(omega):
    angle=np.linalg.norm(omega)
    if angle<1e-12:
        return np.eye(3)
    axis=omega/angle
    K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
    return np.cos(angle)*np.eye(3)+(1-np.cos(angle))*np.outer(axis,axis)+np.sin(angle)*K

def motionModel(x,lin_acc,ang_vel,dt):
    p=x[0:3]
    phi=x[3:6]
    v=x[6:9]
    b_acc=x[9:12]
    b_gyr=x[12:15]
    R=exp_map(phi)
    acc_world=R@(lin_acc - b_acc)+np.array([0,0,9.81])
    p_new=p+v*dt+0.5*acc_world*(dt**2)
    v_new=v+acc_world*dt
    omega=(ang_vel - b_gyr)
    phi_new=phi+omega*dt
    x_new=np.hstack([p_new,phi_new,v_new,b_acc,b_gyr])
    return x_new

def motionModelJacobian(x,lin_acc,ang_vel,dt):
    p=x[0:3]
    phi=x[3:6]
    v=x[6:9]
    b_acc=x[9:12]
    b_gyr=x[12:15]
    R=exp_map(phi)
    acc=(lin_acc - b_acc)
    omega=(ang_vel - b_gyr)
    Fx=np.eye(15)
    Fx[0:3,6:9]=np.eye(3)*dt
    Fx[0:3,9:12]=-0.5*(R)*dt**2
    Fx[6:9,9:12]=-R*dt
    w_skew=np.array([[0,-omega[2],omega[1]],[omega[2],0,-omega[0]],[-omega[1],omega[0],0]])
    a_skew=np.array([[0,-acc[2],acc[1]],[acc[2],0,-acc[0]],[-acc[1],acc[0],0]])
    Fx[3:6,3:6]=exp_map(omega*dt)
    Fx[3:6,12:15]=-np.eye(3)*dt
    Fx[6:9,3:6]=-R@a_skew*dt
    return Fx

def measurementModel(x):
    return x[0:3]

def measurementModelJacobian():
    H=np.zeros((3,15))
    H[0,0]=1
    H[1,1]=1
    H[2,2]=1
    return H

def main():
    pose_file='drone_pose_6D.txt'
    imu_file='imu_data.txt'
    output_file='ego_velocity.txt'
    pose_data=read_pose_file(pose_file)
    imu_data=read_imu_file(imu_file)
    p=np.array([pose_data[0][1],pose_data[0][2],pose_data[0][3]])
    phi=np.zeros(3)
    v=np.zeros(3)
    b_acc=np.zeros(3)
    b_gyr=np.zeros(3)
    x=np.hstack([p,phi,v,b_acc,b_gyr])
    P=np.eye(15)*0.01
    Q=np.eye(15)*0.001
    Rm=np.eye(3)*0.08
    prev_time=pose_data[0][0]
    f_out=open(output_file,'w')
    for i in range(1,len(pose_data)):
        Time,px,py,pz,Rot_mat=pose_data[i]
        dt=Time-prev_time
        prev_time=Time
        la,av=interpolate_imu(imu_data,Time)
        x_pred=motionModel(x,la,av,dt)
        Fx=motionModelJacobian(x,la,av,dt)
        P=Fx@P@Fx.T+Q
        z=np.array([px,py,pz])
        H=measurementModelJacobian()
        y=z - measurementModel(x_pred)
        S=H@P@H.T+Rm
        K=P@H.T@np.linalg.inv(S)
        x_up=x_pred+K@y
        P=(np.eye(15)-K@H)@P
        x=x_up
        p_up=x[0:3]
        phi_up=x[3:6]
        v_up=x[6:9]
        R_inv_txt=Rot_mat.T
        ego_lin_vel=R_inv_txt@v_up
        f_out.write(f"{Time} {ego_lin_vel[0]} {ego_lin_vel[1]} {ego_lin_vel[2]}\n")
    f_out.close()

if __name__=="__main__":
    main()
