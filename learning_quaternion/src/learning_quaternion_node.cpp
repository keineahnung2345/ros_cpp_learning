#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//http://wiki.ros.org/tf2/Tutorials/Quaternions
//https://www.guyuehome.com/33233

//https://answers.ros.org/question/258425/converting-python-tf-code-to-c/
static geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
    // http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
    geometry_msgs::Quaternion q;
    double t0 = cos(yaw * 0.5);
    double t1 = sin(yaw * 0.5);
    double t2 = cos(roll * 0.5);
    double t3 = sin(roll * 0.5);
    double t4 = cos(pitch * 0.5);
    double t5 = sin(pitch * 0.5);
    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}

int main(){
    {
        tf2::Quaternion myq;
        myq.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
        myq.normalize();
        ROS_INFO_STREAM(myq);  // Print the quaternion components (0,0,0,1)
        // [ INFO] [1628338453.151875668]: 0x7ffdb8638a60
        ROS_INFO("%f  %f  %f  %f" , myq.x(), myq.y(), myq.z(), myq.w());  // Print the quaternion components (0,0,0,1)
        // [ INFO] [1628338858.477592400]: 0.000000  0.000000  0.000000  1.000000
    }

    {
        tf2::Quaternion quat_tf;
        geometry_msgs::Quaternion quat_msg;
        quat_msg.x = 0;
        quat_msg.y = 0;
        quat_msg.z = 0;
        quat_msg.w = 1;
        ROS_INFO("%f  %f  %f  %f" , quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);  // Print the quaternion components (0,0,0,1)
        
        tf2::convert(quat_msg , quat_tf);
        ROS_INFO("%f  %f  %f  %f" , quat_tf.x(), quat_tf.y(), quat_tf.z(), quat_tf.w());  // Print the quaternion components (0,0,0,1)
        // or
        tf2::fromMsg(quat_msg, quat_tf);
        ROS_INFO("%f  %f  %f  %f" , quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);  // Print the quaternion components (0,0,0,1)
        // or for the other conversion direction
        quat_msg = tf2::toMsg(quat_tf);
        ROS_INFO("%f  %f  %f  %f" , quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);  // Print the quaternion components (0,0,0,1)
    }
    
    {
        // roll_radians, pitch_radians, yaw_radians
        geometry_msgs::Quaternion quat_msg;
        quat_msg = createQuaternionFromRPY(3.14159, 0, 0);
        ROS_INFO("%f  %f  %f  %f" , quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);  // Print the quaternion components (0,0,0,1)
    }
        
    {
        geometry_msgs::Quaternion quat_msg;
        quat_msg.x = 0;
        quat_msg.y = 0;
        quat_msg.z = 0;
        quat_msg.w = 1;

        tf2::Quaternion q_orig, q_rot, q_new;
        
        // Get the original orientation of 'commanded_pose'
        tf2::convert(quat_msg, q_orig);
        
        double r=3.14159, p=0, y=0;  // Rotate the previous pose by 180* about X
        q_rot.setRPY(r, p, y);
        
        q_new = q_rot*q_orig;  // Calculate the new orientation
        q_new.normalize();
        
        // Stuff the new rotation back into the pose. This requires conversion into a msg type
        tf2::convert(q_new, quat_msg);
        ROS_INFO("%f  %f  %f  %f" , quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);  // Print the quaternion components (0,0,0,1)
    }
    
    {    
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::Quaternion q_rot_back, q_new_back;

        q_orig.setRPY( 0, 0, 0 );

        q_rot.setRPY( 0, 1, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
        q_rot.normalize();

        q_new = q_rot*q_orig;  // Calculate the new orientation
        q_new.normalize();

        ROS_INFO("%f  %f  %f  %f" , q_rot.x(), q_rot.y(), q_rot.z(), q_rot.w());  // Print the quaternion components (0,0,0,1)
        ROS_INFO("%f  %f  %f  %f" , q_new.x(), q_new.y(), q_new.z(), q_new.w());  // Print the quaternion components (0,0,0,1)
        q_rot_back = q_rot.inverse();
        q_new_back = q_rot_back*q_new;  // Calculate the new orientation
        q_new_back.normalize();
        ROS_INFO("%f  %f  %f  %f" , q_rot_back.x(), q_rot_back.y(), q_rot_back.z(), q_rot_back.w());  // Print the quaternion components (0,0,0,1)
        ROS_INFO("%f  %f  %f  %f" , q_new_back.x(), q_new_back.y(), q_new_back.z(), q_new_back.w());  // Print the quaternion components (0,0,0,1)
    }
    
    {    
        // qr * q1 = q2
        // qr = q2 * (q1)^(-1)
        tf2::Quaternion q1, q2, qr;

        q1.setRPY( 1, 0, 0 );
        q2.setRPY( 0, 1, 0 );
        
        qr = q2  * q1.inverse();
        ROS_INFO("%f  %f  %f  %f" , q1.x(), q1.y(), q1.z(), q1.w());  // Print the quaternion components (0,0,0,1)
        ROS_INFO("%f  %f  %f  %f" , q2.x(), q2.y(), q2.z(), q2.w());  // Print the quaternion components (0,0,0,1)
        ROS_INFO("%f  %f  %f  %f" , qr.x(), qr.y(), qr.z(), qr.w());  // Print the quaternion components (0,0,0,1)
    }
    return 0;
}

