#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <stdio.h>

#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define RANGE (1024 / 2)
double X=0,Y=0;
double Xgoal=0.10,Ygoal=0.10;
double prevmvt=0;
double orientation=0, dir=0;

void compute_odometry() {
  double l = wb_differential_wheels_get_left_encoder();
  double r = wb_differential_wheels_get_right_encoder();
  double dl = l / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
  double dr = r / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH;               // delta orientation
  double mvt;
  if (da<=0){
  da=-da;
  }
  while (da>=6.28){//modulo 6.28
    da=da-6.28;
  }
  orientation=da/6.28*360;
  
  mvt=(dl+dr)/2;
  if (mvt<0) {
    mvt=-mvt;
  }  
  mvt=mvt-prevmvt;
  
  theta = theta+da
  X=X+sin(theta)*mvt;
  Y=Y+cos(theta)*mvt;
  printf("estimated distance covered by left wheel: %g m.\n",dl);
  printf("estimated distance covered by right wheel: %g m.\n",dr);
  //printf("estimated change of orientation: %g rad.\n",da);
  //printf("estimated change of orientation: %g mvt.\n",mvt);
  //printf("estimated change of orientation: %g prevmvt.\n",prevmvt);
  printf("estimated change of orientation: %g sin.\n",sin(da));
  printf("estimated change of orientation: %g cos.\n",cos(da));
  printf("orientation: %g.\n",orientation);
  printf("X %g \n",X);
  printf("Y %g \n",Y);
  
  prevmvt=prevmvt+mvt;

  
}

int main(int argc, char *argv[]) {

  /* define variables */
  WbDeviceTag distance_sensor[8];
  int i,j;
  
  
  int Wall=0;
  double speed[2];
  double sensors_value[8];
  /* initialize Webots */
  wb_robot_init();

  /* get and enable devices */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera,TIME_STEP*16);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer,TIME_STEP*4);
  wb_differential_wheels_enable_encoders(TIME_STEP*4);


  for (i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],TIME_STEP*4);
  }

  /* main loop */
  for (;;) {
    
    
    /* get sensors values */
    for (i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }
    const double *a = wb_accelerometer_get_values(accelerometer);
    double Xd=0, Yd=0;
    /* compute odometry and speed values*/
    compute_odometry();
    
    //default instruction
    speed[0] = 200.0;
    speed[1] = 200.0;
    
    Xd=Xgoal-X;
    Yd=Ygoal-Y;
    dir=tan(Yd/Xd);
    /*
    while (dir>=6.28){//modulo 6.28
      dir=dir-6.28;
    }
    */
    
    
    printf("dir %g.\n",dir);
    /*
    if ((dir<0.90*orientation)||(dir>1.10*orientation)){
      if (dir<orientation){
        speed[0] = 200.0;
        speed[1] = -200.0;
      }
      if (dir>orientation){
        speed[0] = -200.0;
        speed[1] = 200.0;
      }
      Wall=0;
    }
    */
      
//  This is to stop infront of the wall
      if (sensors_value[0]>500||sensors_value[7]>500)
      {
          speed[0]=0.0;
          speed[1]=0.0;
      }
//
      
      
// This is to move the robot in a square form
      if(da>=1.57)
       {
          wb_differential_wheels_set_encoders(0,0);
       }
      if(rightw>=1900)
       {
          speed[0]=-200;
          speed[1]=200;
       }
      if(da>=1.57)
       {
          wb_differential_wheels_set_encoders(0,0);
       }
      
//
      
    
      
// This part is to follow the wall or obstacle
// and keep in contact with the wall
      
      // This condition is to keep in contact with the wall
    if (sensors_value[5]<=400&&sensors_value[5]>=350)
      {
        speed[0]=100.0;
        speed[1]=150.0;
        Wall=1;
      }
      // This condition is find the wall incase it losses
    if (sensors_value[5]<150 && Wall==1)
     {
       speed[0]=100.0;
       speed[1]=250.0;
     }
      // This condition is turn right and stay with the wall
      // Collision avoidance
    if (sensors_value[0]>500||sensors_value[7]>500||sensors_value[6]>500||  sensors_value[1]>500)
     {
       speed[0]=150.0;
       speed[1]=-150.0;
       Wall=1;
     }
////
    /*
    if (sensors_value[5]<50&&Wall==1){
      for (j=0;j<20;j++){
        speed[0]=-150.0;
        speed[1]=150.0;
      }
    Wall=0;
    }
    */
    
      
// This part is the Bug Algorithm
//        to reach the goal and then stop
    if (((X>0.98*Xgoal)&&(X<1.02*Xgoal))&&((Y>0.98*Ygoal)&&(Y<1.02*Ygoal))){
      speed[0] = 0.0;
      speed[1] = 0.0;
      printf("goal reach");
    }
      
      
    /* set speed values */
    wb_differential_wheels_set_speed(speed[0],speed[1]);
    /* perform a simulation step */
    wb_robot_step(TIME_STEP);
  }

  return 0;
}
