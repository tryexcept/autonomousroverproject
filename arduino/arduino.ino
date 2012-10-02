/* 
 * arp - autonomous rover project
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <Servo.h>

void requestUltrasonicRead() {
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
                               // the address specified in the datasheet is 224 (0xE0)
                               // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x51));      // command sensor to measure in "centimetres" (0x50)
                               // use 0x51 for inches
                               // use 0x52 for ping microseconds
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(65);                   // datasheet suggests at least 65 milliseconds
  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(112); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting1
  // step 4: request reading from sensor
  Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112
}

int getRange_Ultrasound() {
  int tmp = 0;
  requestUltrasonicRead();

  // step 5: receive reading from sensor
  if(2 <= Wire.available())    // if two bytes were received
  {
    tmp = Wire.read();  // receive high byte (overwrites previous reading)
    tmp = tmp << 8;    // shift high byte to be high 8 bits
    tmp |= Wire.read(); // receive low byte as lower 8 bits
  }
  return tmp;
}

sensor_msgs::LaserScan ls_msg;
std_msgs::String string_msg;
ros::Publisher pub_laserscan( "/base_scan", &ls_msg);
ros::Publisher pub_debug( "/arpdebug", &string_msg);

void messageCb( const std_msgs::Empty& toggle_msg){
    tone(8, 262, 100);
}

ros::Subscriber<std_msgs::Empty> subbeep("/beep", messageCb );

void setupUltrasonic()
{
    Wire.begin();
    Wire.beginTransmission(112); // transmit to device #112 (0x70)
    Wire.write(byte(0x02));      // sets register pointer to range register (0x02)
    Wire.write(byte(0x5d));      
    Wire.endTransmission();      // stop transmitting
}

ros::NodeHandle  nodeHandle;
void setupPubSub()
{
    ls_msg.header.frame_id = "laser_frame";
    ls_msg.ranges = (float *) malloc(50*sizeof(float));
    ls_msg.angle_min = 0;
    ls_msg.angle_max = 50;
    ls_msg.angle_increment = 1;
    ls_msg.scan_time = 1;
    nodeHandle.initNode();
    nodeHandle.advertise(pub_laserscan);
    nodeHandle.advertise(pub_debug);
    nodeHandle.subscribe(subbeep);
}

Servo myservo;  
void setupServo()
{
    myservo.attach(9);
    myservo.write(25);
    // make sure it gets there
    delay(2000);
}

void setupMotors()
{
    const int speedA = 3;
    const int dirA = 12;
    const int dirB = 13;  
    const int speedB = 11; 
    pinMode (dirA, OUTPUT);
    pinMode (dirB, OUTPUT);
    pinMode (speedA, OUTPUT);
    pinMode (speedB, OUTPUT);
}

void beep()
{
    tone(8, 262, 100);
}

void setup()
{
    setupServo();
    setupUltrasonic();
    setupPubSub();
    setupMotors();

    // tell the world you are ready
    for (int i=0; i < 10; ++i)
    {
        beep();
        delay(100);
    }
}

int servo_pos = 25;
bool flag = true;
void sweepAndScan()
{
    ls_msg.ranges[servo_pos] = (float) getRange_Ultrasound();
    if (flag) {
        --servo_pos;
    }
    else {
        ++servo_pos;
    }
    myservo.write(servo_pos);
    if (servo_pos == 50) 
    {
        ls_msg.header.stamp = nodeHandle.now();
        pub_laserscan.publish(&ls_msg);
        flag = true;
    }
    else if(servo_pos == 0) {
        flag = false;
    }
    else if (servo_pos % 5 == 0) {
        string_msg.data = "Hello";
        pub_debug.publish(&string_msg);
    }
}

void loop()
{
    sweepAndScan();
    nodeHandle.spinOnce();
}
