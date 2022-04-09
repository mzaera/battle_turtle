
//+++++++++++++++++++++++++++++++LLIBRERIES+++++++++++++++++++++++++++++++++++

#include <SharpIR.h>
#include <Encoder.h>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#define PI 3.1415926535897932384626433832795
#define NORMALIZE(z) atan2(sin(z), cos(z))

//++++++++++++++++++++++++++++++SENSORS_MOTORS+++++++++++++++++++++++++++++++++

int M1_Direction = 5;
int M2_Direction = 6; //6
int M1 = 4;
int M2 = 9;//9

SharpIR sensor_L( SharpIR::GP2Y0A21YK0F, A2 );
SharpIR sensor_F( SharpIR::GP2Y0A21YK0F, A3 );
SharpIR sensor_R( SharpIR::GP2Y0A21YK0F, A4 );

Encoder encL(18, 19);
Encoder encR(2, 3);

//+++++++++++++++++++++++++++++++++++TEMPS+++++++++++++++++++++++++++++++++++

int period =100;
unsigned long Time = 0;
unsigned long currentMillis;


//++++++++++++++++++++++++++++++VARIABLES_MOVIMENT+++++++++++++++++++++++++++++

const float r=0.016, b=0.093, c=3576.0*2; //Dimensions robot

long oldPosition_L  = 0, newPosition_L, N_L ; //Pels encoders
long oldPosition_R  = 0, newPosition_R, N_R ;

float x=0.0, y=0.0; //Posicio actual
float ang = 0.0; 

float v,w;  //velocitats actuals
float wl, wr;

float v_f,w_f;  //velocitats desitjades
float wl_f, wr_f;

float Ep_R=0.0, Ep_L=0.0, Ei_R=0.0, Ei_L=0.0, Ed_R=0.0 , Ed_L=0.0 ; //errors PID
float prev_Ep_R, prev_Ep_L;

float Kp = 28.333333 , Ki = 0.5*(period/1000.0) , Kd = 0.125*(period/1000.0); //constants PID

int vel_R=0, vel_L=0; //velocitats per els motors

//++++++++++++++++++++++++++++++++++++ROS+++++++++++++++++++++++++++++++++++++

ros::NodeHandle  nh;

std_msgs::Float32 front_distance_msg;
std_msgs::Float32 right_distance_msg;
std_msgs::Float32 left_distance_msg;
geometry_msgs::Pose2D pose_msg;

ros::Publisher pub_front("front_distance", &front_distance_msg);
ros::Publisher pub_right("right_distance", &right_distance_msg);
ros::Publisher pub_left("left_distance", &left_distance_msg);
ros::Publisher pub_pose("pose", &pose_msg);


void pose_Cb( const geometry_msgs::Pose2D& pose_info);
void vel_Cb( const geometry_msgs::Twist& vel_info);

ros::Subscriber<geometry_msgs::Pose2D> sub_pose("set_pose", pose_Cb );
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", vel_Cb );

float linear_reb, angular_reb, auxiliar;

//********************************************************************************************************************//
//**************************************************_SETUP_**********************************************************//
//******************************************************************************************************************//

void setup() { 

  pinMode (M1_Direction, OUTPUT); 
  pinMode (M2_Direction, OUTPUT); 
  pinMode (M1, OUTPUT); 
  pinMode (M2, OUTPUT); 

  nh.initNode();

  nh.advertise(pub_front);
  nh.advertise(pub_right);
  nh.advertise(pub_left);
  nh.advertise(pub_pose);

  nh.subscribe(sub_pose);
  nh.subscribe(sub_vel);

  delay(500);
          
}

//********************************************************************************************************************//
//**************************************************_LOOP_***********************************************************//
//******************************************************************************************************************//

void loop() {
  
 currentMillis = millis();
 
 if(currentMillis > Time + period){
    Time = currentMillis ; 
    
    moviment(linear_reb , angular_reb);

    actualitza();
    
    pub_front.publish( &front_distance_msg );
    pub_right.publish( &right_distance_msg );
    pub_left.publish( &left_distance_msg );
    pub_pose.publish( &pose_msg );

    

  }
  nh.spinOnce();
}

void actualitza(){
  
  float sensor_F_temp = sensor_F.getDistance();
  float sensor_R_temp = sensor_R.getDistance();
  float sensor_L_temp = sensor_L.getDistance();

  front_distance_msg.data = sensor_F_temp/100;
  right_distance_msg.data = sensor_R_temp/100;
  left_distance_msg.data = sensor_L_temp/100;

  pose_msg.x = x;
  pose_msg.y = y;
  pose_msg.theta  = ang;
}

void pose_Cb( const geometry_msgs::Pose2D& pose_info){
  
  x = pose_info.x;
  y = pose_info.y;
  ang = pose_info.theta;
}

void vel_Cb( const geometry_msgs::Twist& vel_info){
  
  linear_reb = vel_info.linear.x;
  angular_reb = vel_info.angular.z;
  
  auxiliar = vel_info.linear.y;
}


//********************************************************************************************************************//
//*************************************************_MOVMENT_*********************************************************//
//******************************************************************************************************************//

void moviment(float linear, float angular){

  Final_Pos(linear,angular);
  encoder_L(); 
  encoder_R();
  Actual_Pos();
  PID();
  motor();
}

//NUM1******FINAL_POS*******Actualitza la velocitat desitjada comprobant que sigui correcte. 
void Final_Pos(float linear, float angular){                
  v_f=linear; 
  w_f=angular;

  if(v_f>0.087){ v_f= 0.087; }
  if(w_f>1.75){ w_f= 1.75; }
  
  if(v_f<-0.087){ v_f= -0.087; }
  if(w_f<-1.75){ w_f= -1.75; }
  
  int w_test = -22.449*v_f + 1.9763;
  if(w_f>w_test){
    w_f=w_test;
  }
  
  wl_f= (v_f-(b*w_f/2.0))/r;
  wr_f=(v_f+(b*w_f/2.0))/r;
}

//NUM2***********ENCODERS*****Actualitza el valor dels encoders
void encoder_L(){
  newPosition_L = encL.read();
  N_L=newPosition_L-oldPosition_L;
  oldPosition_L = newPosition_L;   
}

void encoder_R(){
  newPosition_R = encR.read(); 
  N_R=newPosition_R-oldPosition_R;
  oldPosition_R = newPosition_R;  
}
//NUM3******ACTUAL_POS*******Actualitza la velocitat actual. 
void Actual_Pos(){
  v=(2.0*PI*r/c)*((N_R+N_L)/2.0)*(1.0/(period/1000.0)); 
  w=(2.0*PI*r/c)*((N_R-N_L)/b)*(1.0/(period/1000.0)); 

  if(auxiliar != 1.1){
    ang = ang + w * (period/1000.0) * 0.87; //el 0.9 me le tret de la patilla direm que es calibracio experimental
    ang= NORMALIZE(ang);
    x = ( x + v * cos(ang) * (period/1000.0) ); //x2-x1 = v * (t2- t1) * cos(ang)
    y = ( y + v * sin(ang) * (period/1000.0) );
  }
  wl= (v-(b*w/2.0))/r;
  wr=(v+(b*w/2.0))/r;
}

//NUM4********PID*********Envia la velocitat desitjada al motor. 
void PID(){
  prev_Ep_R = Ep_R;
  prev_Ep_L = Ep_L;
 
  Ep_R = wr_f - wr;
  Ep_L = wl_f - wl;

  Ed_R= Ep_R - prev_Ep_R;
  Ed_L= Ep_L - prev_Ep_L;

  Ei_R = Ei_R + Ep_R;
  Ei_L = Ei_L + Ep_R;

  float G_R = (Kp * Ep_R) + ( Ki * Ei_R * (period/1000.0) ) + ( Kd * Ed_R * (1.0/(period/1000.0)) );
  float G_L = (Kp * Ep_L) + ( Ki * Ei_L * (period/1000.0) ) + ( Kd * Ed_L * (1.0/(period/1000.0)) );
    
  vel_R = vel_R + G_R;
  vel_L = vel_L + G_L;
}
 
//NUM5*****MOTOR_CONTROL******Envia el valor correcte als motors
void motor(){

  int Correct_R, Correct_L;
  int dir_R=0,dir_L=1;

  if(vel_R<0){  
    dir_R=1;
    Correct_R = - vel_R;  
  }else{
    dir_R=0;
    Correct_R = vel_R;
  }     

  if(vel_L<0){  
    dir_L=1;
    Correct_L = - vel_L;  
  }else{
    dir_L=0;
    Correct_L = vel_L;
  }

  if(Correct_R>255){  
    Correct_R = 255;  
  }

  if(Correct_L>255){  
    Correct_L = 255;  
  }
     
  digitalWrite(M1_Direction, dir_R);
  digitalWrite(M2_Direction,dir_L); 
  analogWrite(M1, Correct_R);
  analogWrite(M2, Correct_L);
}
