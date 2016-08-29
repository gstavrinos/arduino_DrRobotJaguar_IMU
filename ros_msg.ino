#include <ros.h>
#include <std_msgs/Int16.h>
#include <Wire.h>
#include <std_msgs/String.h>


#define GYROADDR 0x68
#define COMPASSADDR 0x1e
#define ACCELADDR 0x53    

ros::NodeHandle nh;

std_msgs::Int16 msg;
ros::Publisher accelx("accelx", &msg);
ros::Publisher accely("accely", &msg);
ros::Publisher accelz("accelz", &msg);
ros::Publisher compx("compx", &msg);
ros::Publisher compy("compy", &msg);
ros::Publisher compz("compz", &msg);
ros::Publisher gyrox("gyrox", &msg);
ros::Publisher gyroy("gyroy", &msg);
ros::Publisher gyroz("gyroz", &msg);

union XYZBuffer {
      struct {
        short x,y,z;
      } value;
      byte buff[6];
    };
     
    void changeEndian(union XYZBuffer *xyz) {
      for (int i=0;i<6;i+=2) {
        byte t=xyz->buff[i];
        xyz->buff[i]=xyz->buff[i+1];
        xyz->buff[i+1]=t;
      }
    }
     
    // Generically useful reading into a union type
    void readXYZ(int device,union XYZBuffer *xyz) {    
      Wire.requestFrom(device, 6);      
      long start=millis();
      while (!Wire.available() && (millis()-start)<100);
      if (millis()-start<100) {
        for (int i=0;i<6;i++)
          xyz->buff[i]=Wire.read();
      }
    }
     
    void setupAccel(int device) {
      // Check ID to see if we are communicating
      Wire.beginTransmission(device);
      Wire.write(0x00); // One Reading
      Wire.endTransmission();
      Wire.requestFrom(device,1);
      while (!Wire.available());  
      byte ch=Wire.read();
      Serial.print("Accel id is 0x");
      Serial.println(ch,HEX);
      // Should output E5
     
      // https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
      // Page 16
      Wire.beginTransmission(device);
      Wire.write(0x2d);
      Wire.write(0x08);
      Wire.endTransmission();
      Wire.beginTransmission(device);
      Wire.write(0x38);
      Wire.write(0x84);
      Wire.endTransmission();
     
    }
    void readAccel(int device,union XYZBuffer *xyz) {
      Wire.beginTransmission(device);
      Wire.write(0x32); // One Reading
      Wire.endTransmission();
      readXYZ(device,xyz);
    }
     
    void setupCompass(int device) {
      // Check ID to see if we are communicating
      Serial.print("Compass id is ");
      Wire.beginTransmission(device);
      Wire.write(10); // One Reading
      Wire.endTransmission();
      Wire.requestFrom(device,2);
      while (!Wire.available());
      char ch=Wire.read();
      Serial.print(ch);  
      ch=Wire.read();
      Serial.println(ch);
      // Should output H4  
     
    // Page 18
    // at http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf
      Wire.beginTransmission(device);
      Wire.write(0x00); Wire.write(0x70);
      Wire.endTransmission();
      Wire.beginTransmission(device);
      Wire.write(0x01); Wire.write(0xA0);
      Wire.endTransmission();
      Wire.beginTransmission(device);
      Wire.write(0x02); Wire.write(0x00); //  Reading
      Wire.endTransmission();
      delay(6);
    }
    void readCompass(int device,union XYZBuffer *xyz) {
      readXYZ(device,xyz);
      changeEndian(xyz);
      Wire.beginTransmission(device);
      Wire.write(0x03);
      Wire.endTransmission();
    }
     
    void setupGyro(int device) {
      // Check ID to see if we are communicating
      Wire.beginTransmission(device);
      Wire.write(0x00); // One Reading
      Wire.endTransmission();
      Wire.requestFrom(device,1);
      while (!Wire.available());  
      byte ch=Wire.read();
      Serial.print("Gyro id is 0x");
      Serial.println(ch,HEX);  
      // Should output 69
    }
    void readGyro(int device,union XYZBuffer *xyz) {
      // https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
      // page 20
      Wire.beginTransmission(device);
      Wire.write(0x1d);
      Wire.endTransmission();
      readXYZ(device,xyz);
      changeEndian(xyz);  
    }
     
    void pad(int width,int number) {
      int n=abs(number);
      int w=width;
      if (number<0) w--;
      while (n>0) {
        w--;
        n/=10;
      }
      if (number==0) w--;
      for (int i=0;i<w;i++) Serial.print(' ');
    }
     
    void output(union XYZBuffer xyz) {
    //  pad(6,xyz.value.x);
      Serial.print(xyz.value.x);
      Serial.print(',');
    //  pad(6,xyz.value.y);
      Serial.print(xyz.value.y);
      Serial.print(',');
    //  pad(6,xyz.value.z);
      Serial.print(xyz.value.z);
    }

void setup()
{
  nh.initNode();
  nh.advertise(accelx);
  nh.advertise(accely);
  nh.advertise(accelz);
  nh.advertise(compx);
  nh.advertise(compy);
  nh.advertise(compz);
  nh.advertise(gyrox);
  nh.advertise(gyroy);
  nh.advertise(gyroz);
  //Serial.begin(57600);  // start serial for output
  Wire.begin();        // join i2c bus (address optional for master)
  setupCompass(COMPASSADDR);
  setupAccel(ACCELADDR);
  setupGyro(GYROADDR);
}

void loop()
{
  union XYZBuffer comp,gyro,accel;
  /*int l1,l2,l3,l4;
  l1=analogRead(0);
  l2=analogRead(1);
  l3=analogRead(2);
  l4=analogRead(3);*/
  readAccel(ACCELADDR,&accel);
  readCompass(COMPASSADDR,&comp);
  readGyro(GYROADDR,&gyro);
  msg.data = accel.value.x;
  accelx.publish( &msg );
  msg.data = accel.value.y;
  accely.publish( &msg );
  msg.data = accel.value.z;
  accelz.publish( &msg );
  msg.data = comp.value.x;
  compx.publish( &msg );
  msg.data = comp.value.y;
  compy.publish( &msg );
  msg.data = comp.value.z;
  compz.publish( &msg );
  msg.data = gyro.value.x;
  gyrox.publish( &msg );
  msg.data = gyro.value.y;
  gyroy.publish( &msg );
  msg.data = gyro.value.z;
  gyroz.publish( &msg );
  nh.spinOnce();
  delay(5);
}

