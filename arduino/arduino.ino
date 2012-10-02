/* 
 * arp - autonomous rover project
 */

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16MultiArray.h>
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

std_msgs::Int16MultiArray ls_msg;
ros::Publisher pub_laserscan( "/arp_base_scan", &ls_msg);

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
    ls_msg.data_length = 50;
    nodeHandle.initNode();
    nodeHandle.advertise(pub_laserscan);
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
int fullscan[50];
void sweepAndScan()
{
    fullscan[servo_pos] = getRange_Ultrasound();
    if (flag) {
        --servo_pos;
    }
    else {
        ++servo_pos;
    }
    myservo.write(servo_pos);
    if (servo_pos == 50) 
    {
        flag = true;
        //ls_msg.data = &fullscan;
        pub_laserscan.publish(&ls_msg);
    }
    else if(servo_pos == 0) {
        flag = false;
    }
}

void loop()
{
    sweepAndScan();
    nodeHandle.spinOnce();
}
