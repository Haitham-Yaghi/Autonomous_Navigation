#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>                  
#include <geometry_msgs/Twist.h>
#include <string>
#include <iostream>
using namespace std;

ros::Publisher pub;

string status = "Error";
//float linear_x = 0;
//float angular_z = 0;
geometry_msgs::Twist message;
/*
void counter(float angular, float linear)
{
    float inv_angular = -1*angular;
    message.linear.x = linear;
    message.angular.z = inv_angular;

    cout<<"countering";
    pub.publish(message);

}
void action_left(){
        ros::Time start_time = ros::Time::now(); //0
        ros::Duration duration(2.0);  // 5 seconds
        //int x = 5;
        while (ros::ok()) {
          message.linear.x=0.3;
            message.angular.z=0.7;
            cout<<"  "<<status;
            cout<<"  "<<right;
            //cout<<"  "<<center;
            cout<<"   "<<left;
            cout<<"\n";
            pub.publish(message);
        if (start_time > duration) {
            //x+=1;
            break;
        }
        
        }
   
}

void action_right(){
    ros::Time start_time = ros::Time::now();
        ros::Duration duration(2.0);  // 5 seconds
        //int x = 5;
        while (ros::ok()) {
           message.linear.x=0.3;
            message.angular.z=-0.7;
            cout<<"  "<<status;
            cout<<"  "<<right;
            //cout<<"  "<<center;
            cout<<"   "<<left;
            cout<<"\n";
            pub.publish(message);
        if (start_time > duration) {
            //x+=1;
            break;
        }
        
        }
 */
void checkcase(float right , float center , float left)
{
   if ( right > 0.7  && center > 0.7 &&  left < 0.7)
    {
        status = "NO OBSTACLE!";
        //cout<<"no object";
        message.linear.x =0.4;
        message.angular.z=0;
        cout<<"  "<<status;
        cout<<"  "<<right;
        cout<<"  "<<center;
        cout<<"   "<<left;
        cout<<"\n"; 
        pub.publish(message);
        
        
    }
    
    else if ( right > 0.7  &&  center < 0.7 &&  left < 0.7)
    {
        status = "OBSTACLE CENTER!";  message.linear.x=0.3;
        message.linear.x=0;
        message.angular.z=1;
        // cout<<"  "<<status;
        // cout<<"  "<<right;
        // cout<<"  "<<center;
        // cout<<"   "<<left;
        // cout<<"\n"; 
         pub.publish(message);

    }
    
    else if ( right < 0.7 &&  center < 0.7 &&  left <0.7 )
    {
        status = "OBSTACLE RIGHT!";
        int x = 0;
        ros::Time startTime = ros::Time::now();
        ros::Duration loopDuration(0.3); // 5 seconds
        
        while (ros::Time::now() < startTime+loopDuration && x==0)
        {
        message.linear.x=0.3;
        message.angular.z=0.7; //left
        pub.publish(message);
        if((right > 0.7  &&  center < 0.7 &&  left < 0.7) || ( right < 0.7 && center > 0.7 && left < 0.7)) {
            x=1;
            break;
        }
        }

        x = 0;
        ros::Time startTime2 = ros::Time::now();
        ros::Duration loopDuration2(0.3); // 5 seconds
        while (ros::Time::now() < startTime2+loopDuration2 && x==0)
        {
        message.linear.x=0.3;
        message.angular.z=-0.7; //right
        pub.publish(message);
        if( (right > 0.7  &&  center < 0.7 &&  left < 0.7) || ( right < 0.7 && center > 0.7 && left < 0.7)) {
            x=1;
            break;
        }
        }

        // message.linear.x=0.3;
        // message.angular.z=0.7;
        
        // message.linear.x=0.3;
        // message.angular.z=-0.7;

        cout<<"  "<<status;
        cout<<"  "<<right;
        cout<<"  "<<center;
        cout<<"   "<<left;
        cout<<"\n";
        pub.publish(message);
    
       //action_left();
       //action_right();
       

       //ros::Duration five_seconds(5.0);

        
    }
    else if ( right < 0.7 && center > 0.7 && left < 0.7)
    {
        int x = 0;
        status = "OBSTACLE LEFT!";
        ros::Time startTime = ros::Time::now();
        ros::Duration loopDuration(0.3); // 5 seconds
        while (ros::Time::now() < startTime+loopDuration && x==0)
        {
        message.linear.x=0.3;
        message.angular.z=-0.7;
        pub.publish(message);
        if((right < 0.7 &&  center < 0.7 &&  left <0.7) || (right > 0.7  &&  center < 0.7 &&  left < 0.7)) {
            x=1;
            break;
        }
        }
        x = 0;
        ros::Time startTime2 = ros::Time::now();
        ros::Duration loopDuration2(0.3); // 5 seconds
        while (ros::Time::now() < startTime2+loopDuration2 && x==0)
        {
        message.linear.x=0.3;
        message.angular.z=0.7;
        pub.publish(message);
        if((right < 0.7 &&  center < 0.7 &&  left <0.7) || (right > 0.7  &&  center < 0.7 &&  left < 0.7)) {
            x=1;
            break;
        }
        }
        // message.linear.x=0.3;
        // message.angular.z=-0.7;
        // sleep(2);
        // message.linear.x=0.3;
        // message.angular.z=0.7;
        // sleep(2);
        // cout<<"  "<<status;
        // cout<<"  "<<right;
        // cout<<"  "<<center;
        // cout<<"   "<<left;
        // cout<<"\n";
        // pub.publish(message);
        
    }
    /*
    else if ( 0 < right < 1  && center <= 1 &&  left < 0 )
    {
        status = "OBSTACLE RIGHT AND LEFT!";
       message.linear.x=0.4;
        message.angular.z=0.4;
        pub.publish(message);
    }
    */
   
    /*
    else if ( right > 0.7  &&  center < 0.7 &&  left < 0.7 )
    {
        status = "OBSTACLE CENTER AND LEFT!";
        message.linear.x=0.3;
        message.angular.z=-0.7;
        cout<<"  "<<status;
        cout<<"  "<<right;
        cout<<"  "<<center;
        cout<<"   "<<left;
        cout<<"\n"; 
        pub.publish(message);
    }
    */

    /*
    else if ( right < 0.7  &&  center < 0.7 &&  left > 0.7 )
    {
        status = "OBSTACLE CENTER AND RIGHT!";
        message.linear.x=0.3;
        message.angular.z=0.7;
        cout<<"  "<<status;
        cout<<"  "<<right;
        cout<<"  "<<center;
        cout<<"   "<<left;
        cout<<"\n"; 
        pub.publish(message);
    }
    */

    /*
    else if ( right < 1  && center < 1 &&  left < 1 )
    {
        status = "OBSTACLE AHEAD!";
        message.linear.x =0;
        message.angular.z=0.8;
        pub.publish(message);
    }
    

    //message.linear.x = linear_x;
    //message.angular.z = angular_z;
   // pub.publish(message);

*/

   
}


float smallestofarray(float arr[] , int n)
{
    float temp = 0.71;
    for(int i=0; i<n; i++) 
    {
      if(temp>arr[i]) 
      {
         temp=arr[i];
      }
   }
   return temp;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    int length = msg->ranges.size(); //720
    int lendiv3 = length/3; //240
    float right[length/3] , center[length/3] , left[length/3];
    for( int i=0 ; i < length ; i++)
    {
        if(i < lendiv3)
        {
            right[i] = msg->ranges[i];
            
        }
        else if ((i > lendiv3-1) && (i < 2*lendiv3))
        {
            center[i-240] = msg->ranges[i];
        }
        else if (((2*lendiv3)-1) && (i < length))
        {
            left[i - 480] = msg->ranges[i];
        }        
    }
    float rightmin = smallestofarray(right , length/3);
    float centermin = smallestofarray(center , length/3);
    float leftmin = smallestofarray(left , length/3);

    checkcase(rightmin , centermin , leftmin);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoid");
    ros::NodeHandle n("~");

   
    ros::Subscriber sub = n.subscribe("/scan", 1, laser_callback);
    ros::Rate loop_rate(50);
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    message.linear.x = 0;
    message.linear.y = 0;
    message.linear.z = 0;
    message.angular.x = 0;
    message.angular.y = 0;
    message.angular.z = 0;
    
     while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}