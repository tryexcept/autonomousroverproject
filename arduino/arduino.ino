/* 
 * arp - autonomous rover project
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <Wire.h>
#include <Servo.h>

ros::NodeHandle  nodeHandle;
Servo myservo;  
void beep()
{
    tone(8, 262, 100);
}
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
  delay(70);                   // datasheet suggests at least 65 milliseconds
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
ros::Publisher pub_laserscan( "/base_scan", &ls_msg);

void messageCb( const std_msgs::Empty& toggle_msg){
    tone(8, 262, 100);
}

ros::Subscriber<std_msgs::Empty> subbeep("/beep", messageCb );


//void onReceive(int howMany)
//{
//  tone(8, 262, 100);
//  tone(8, 262, 100);
//  tone(8, 262, 100);
//  tone(8, 262, 100);
//  tone(8, 262, 100);
// 
//  //doesn't work
//  // step 5: receive reading from sensor
//  if(2 <= Wire.available())    // if two bytes were received
//  {
//    reading = Wire.read();  // receive high byte (overwrites previous reading)
//    reading = reading << 8;    // shift high byte to be high 8 bits
//    reading |= Wire.read(); // receive low byte as lower 8 bits
//  }
//  requestUltrasonicRead();
//}

int pos = 10;
bool direction = true;
void serv()
{
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    pos = direction?pos+10:pos-10;
    if (pos == 120)
        direction = true;
    else if (pos == 0)
        direction = false;

}

void gmapmode()
{
    for (int i = 26; i < 50; ++i)
    {
        myservo.write(i);
        delay(20);
    }
    for (int i = 50; i > 0; i--)
    {
        myservo.write(i);
        delay(20);
    }
}

void setupUltrasonic()
{
    Wire.begin();
    //Wire.onReceive(onReceive);
    Wire.beginTransmission(112); // transmit to device #112 (0x70)
    Wire.write(byte(0x02));      // sets register pointer to range register (0x02)
    Wire.write(byte(0x5d));      
    Wire.endTransmission();      // stop transmitting
}
void setupPubSub()
{
//    range_msg.radiation_type = sensor_msgs::::ULTRASOUND;
//    range_msg.header.frame_id = "/ultrasound";
//    range_msg.field_of_view = 0.1;  // fake
//    range_msg.min_range = 0;
//    range_msg.max_range = 70;

    nodeHandle.initNode();
    nodeHandle.advertise(pub_laserscan);
    nodeHandle.subscribe(subbeep);
}
void setupServo()
{
    myservo.attach(9);
    myservo.write(25);
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

void setup()
{
    setupServo();
    setupUltrasonic();
    setupPubSub();
    setupMotors();
    beep();
    delay(100);
    beep();
    delay(70);
    beep();
}

void publish()
{
    int range = getRange_Ultrasound();
    ls_msg.header.stamp = nodeHandle.now();
    //ls_msg.
    pub_laserscan.publish(&ls_msg);
}

int servo_pos = 25;

void loop()
{
    long range_time;
    // publish the adc value at most every 250 milliseconds
    // since it takes that long for the sensor to stablize
    // FIXME: public spin lock - i rather have a timer interrupt or something
    //if ( millis() >= range_time )
    //{
    //    range_time =  millis() + 250;
    publish();
    //}
    nodeHandle.spinOnce();
}
