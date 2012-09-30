/* 
 * arp - autonomous rover project
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Empty.h>
#include <Wire.h>
#include "pitches.h"

ros::NodeHandle  nh;
int melody[] = {NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};

// durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = { 4, 8, 8, 4,4,4,4,4 };
char frameid[] = "/ultrasound";
volatile int reading = 0;

void requestUltrasonicRead() {
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
                               // the address specified in the datasheet is 224 (0xE0)
                               // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x50));      // command sensor to measure in "centimetres" (0x50)
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

float getRange_Ultrasound() {
  
  requestUltrasonicRead();

  // step 5: receive reading from sensor
  if(2 <= Wire.available())    // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  return reading;
}

void beep() 
{
    tone(8, NOTE_C4, 100);
}

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

void messageCb( const std_msgs::Empty& toggle_msg){
	beep();
}

ros::Subscriber<std_msgs::Empty> sub("/beep", messageCb );

void onReceive(int howMany)
{
  //doesn't work
  // step 5: receive reading from sensor
  if(2 <= Wire.available())    // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  requestUltrasonicRead();
}

void setup()
{
  Wire.begin();
  
  Wire.onReceive(onReceive);
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
  Wire.write(byte(0x02));      // sets register pointer to range register (0x02)
  Wire.write(byte(0x5d));      
  Wire.endTransmission();      // stop transmitting

  
  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(sub);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0;
  range_msg.max_range = 70;
  beep();
  delay(100);
  beep();
  requestUltrasonicRead();
}

//void music() 
//{
//  gc iterate over the notes of the melody:
//  for (int thisNote = 0; thisNote < 8; thisNote++) {
//
//    gc to calculate the note duration, take one second 
//    gc divided by the note type.
//    gce.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
//    int noteDuration = 1000/noteDurations[thisNote];
//    tone(8, melody[thisNote],noteDuration);
//
//    gc to distinguish the notes, set a minimum time between them.
//    gc the note's duration + 30% seems to work well:
//    int pauseBetweenNotes = noteDuration * 1.30;
//    delay(pauseBetweenNotes);
//    gc stop the tone playing:
//    noTone(8);
//  }
//}

long range_time;
void loop()
{
  //publish the adc value every 250 milliseconds
  //since it takes that long for the sensor to stablize
  if ( millis() >= range_time ){
    range_msg.range = getRange_Ultrasound();
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 250;
  }
  nh.spinOnce();
}

