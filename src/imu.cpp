#include "imu.h"

Imu::Imu(std::string imu_name,time_t start_time) : 
        name(imu_name) ,
        ax(0),
        ay(0),
        az(0),
        vyaw(0),
        vpich(0),
        vroll(0),
        yaw(0),
        pitch(0),
        roll(0),
        state_vector_updated(false),
        stamp_from(start_time)
    {
        for(int i=0; i<15;i++)
        {
            state_vector[i]= 0;
            bias_vector[i]= 0;
        }
           
    }

Imu::Imu(std::string imu_name) : 
        name(imu_name) ,
        ax(0),
        ay(0),
        az(0),
        vyaw(0),
        vpich(0),
        vroll(0),
        yaw(0),
        pitch(0),
        roll(0),
        state_vector_updated(false),
        stamp_from(time(NULL))
    {
        for(int i=0; i<15;i++)
        {
            state_vector[i]= 0;
            bias_vector[i]= 0;
        }
           
    }

void Imu::Update_state(double r_ax,double r_ay,double r_az,double r_vyaw,double r_vpich,double r_vroll,double d_yaw,double d_pitch,double d_roll,time_t t)
{
    ax=r_ax;
    ay=r_ay;
    az=r_az;
    state_vector[0]= state_vector[0]+state_vector[3]*difftime(t,stamp_from)+0.500*ax*difftime(t,stamp_from)*difftime(t,stamp_from);
    state_vector[1]= state_vector[1]+state_vector[4]*difftime(t,stamp_from)+0.500*ay*difftime(t,stamp_from)*difftime(t,stamp_from);
    state_vector[2]= state_vector[2]+state_vector[5]*difftime(t,stamp_from)+0.500*az*difftime(t,stamp_from)*difftime(t,stamp_from);
    state_vector[3]= ax*difftime(t,stamp_from)+ state_vector[3];
    state_vector[4]= ax*difftime(t,stamp_from)+ state_vector[4];
    state_vector[5]= ax*difftime(t,stamp_from)+ state_vector[5];
    state_vector[6] = ax;
    state_vector[7] = ay;
    state_vector[8] = az;
    state_vector[9]= r_vyaw;
    state_vector[10] = r_vpich;
    state_vector[11] = r_vroll;
    state_vector[12] = d_yaw;
    state_vector[13] = d_pitch;
    state_vector[14] = d_roll;
    q = Eigen::AngleAxisd(d_roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(d_pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(d_yaw, Eigen::Vector3d::UnitZ());
    stamp_to = t;
}

void Imu::Update_state(double r_ax,double r_ay,double r_az,double r_vyaw,double r_vpich,double r_vroll,Eigen::Quaternion<double> dq,time_t t)
{
    ax=r_ax;
    ay=r_ay;
    az=r_az;
    state_vector[0]= state_vector[0]+state_vector[3]*difftime(t,stamp_from)+0.500*ax*difftime(t,stamp_from)*difftime(t,stamp_from);
    state_vector[1]= state_vector[1]+state_vector[4]*difftime(t,stamp_from)+0.500*ay*difftime(t,stamp_from)*difftime(t,stamp_from);
    state_vector[2]= state_vector[2]+state_vector[5]*difftime(t,stamp_from)+0.500*az*difftime(t,stamp_from)*difftime(t,stamp_from);
    state_vector[3]= ax*difftime(t,stamp_from)+ state_vector[3];
    state_vector[4]= ax*difftime(t,stamp_from)+ state_vector[4];
    state_vector[5]= ax*difftime(t,stamp_from)+ state_vector[5];
    state_vector[6] = ax;
    state_vector[7] = ay;
    state_vector[8] = az;
    state_vector[9]= r_vyaw;
    state_vector[10] = r_vpich;
    state_vector[11] = r_vroll;
    auto euler = dq.toRotationMatrix().eulerAngles(0, 1, 2);
    state_vector[12] = euler[2];
    state_vector[13] = euler[1];
    state_vector[14] = euler[0];
    stamp_to = t;
}

ImuQueue::ImuQueue(std::shared_ptr<Imu> imu,double T,int max_size) :
queue_size(max_size), 
relative_queue(max_size),
absolute_queue(max_size),
sampling_period(T)
{
    device = imu;   
}

Eigen::Transform<double,3,Eigen::Affine> ImuQueue::getTransform(time_t t)
{
    if(absolute_queue.size()>0)
    {
        if(difftime(t ,absolute_queue[0].stamp_to)<0 && difftime(t ,absolute_queue[absolute_queue.size()-1].stamp_to)>0)
        {
            for (auto state:absolute_queue)
            {
                if(t>state.stamp_from && t<=state.stamp_to)
                {
                    Eigen::Transform<double,3,Eigen::Affine> res = Eigen::Translation<double,3>(state.state_vector[0],state.state_vector[1],state.state_vector[2])*state.q.toRotationMatrix();
                    return res;
                }
            }
        } 
    }
    else 
    {
        #ifdef DEBUG
            std::cout<<"requested transform out of timestamp range"<<std::endl;
        #endif
    }
        
}

Eigen::Transform<double,3,Eigen::Affine> ImuQueue::get_Transform(time_t t_from, time_t t_to)
{
    return getTransform(t_to)*getTransform(t_from).inverse();
}

Eigen::Transform<double,3,Eigen::Affine> ImuQueue::get_OriginTransform()
{
    Imu state = absolute_queue[absolute_queue.size()-1];
    Eigen::Transform<double,3,Eigen::Affine> res = Eigen::Translation<double,3>(state.state_vector[0],state.state_vector[1],state.state_vector[2])*state.q.toRotationMatrix();
    return res;

}

void ImuQueue::stack_absolute(Imu imu)
{
    absolute_queue.push_front(imu);
}

void ImuQueue::stack_relative(Imu imu)
{
    relative_queue.push_front(imu);
}