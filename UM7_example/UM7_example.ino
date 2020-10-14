#include <UM7.h>
#include <Messenger.h>
#include <Chrono.h>

UM7 imu;
bool activated = false;
Chrono timee;
Messenger message = Messenger();

void messageReady() {
  while ( message.available() ) {
    if ( message.checkString("act") ) {
      activated = true;
      Serial.println("activated");
      Serial.send_now();
    } else if ( message.checkString("off") ) {
      activated = false; Serial.println("off");
    }
    // Clean up the serial port
    while (message.available())
      message.readInt();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600); // This is connected to Computer 
  Serial1.begin(115200); // This is connected to IMU UM7
  message.attach(messageReady);
  if (Serial.available() > 0) Serial.read();

}

void loop() {
  char p = 'D';
  // put your main code here, to run repeatedly:
  // if (activated) Serial.println("activated");
  if (Serial1.available() > 0 && activated) {
    p = Serial1.read();
    if (imu.encode(p) ) {
      // Serial.println("The imu was encoded correctly ") ;
      Serial.print("GX: ");
      Serial.print(imu.Gyro_RAW_X);
      Serial.print(" GY: ");
      Serial.print(imu.Gyro_RAW_Y);
      Serial.print(" GZ: ");
      Serial.print(imu.Gyro_RAW_Z);
      Serial.print(" rsG: ");
      Serial.print(imu.res_gyro_raw);
      Serial.print(" GRwTm: ");
      Serial.print(imu.Gyro_RAW_Time, 3);
      Serial.send_now();
      Serial.print(" AcX: ");
      Serial.print(imu.Accel_RAW_X);
      Serial.print(" AcY: ");
      Serial.print(imu.Accel_RAW_Y);
      Serial.print(" AcZ: ");
      Serial.print(imu.Accel_RAW_Z);
      Serial.print(" rAc: ");
      Serial.print(imu.res_acc_raw);
      Serial.print(" AcRwTm: ");
      Serial.print(imu.Accel_RAW_Time, 3);
      Serial.send_now();
      Serial.print(" MX: ");
      Serial.print(imu.Mag_RAW_X);
      Serial.print(" MY: ");
      Serial.print(imu.Mag_RAW_Y);
      Serial.print(" MZ: ");
      Serial.print(imu.Mag_RAW_Z);
      Serial.print(" rM: ");
      Serial.print(imu.res_mag_raw);
      Serial.print(" MgRwTm: ");
      Serial.print(imu.Mag_RAW_Time, 3);
      Serial.send_now();
      Serial.print(" TmpC: ");
      Serial.print(imu.Temperature_degrees_C);
      Serial.print(" TmC: ");
      Serial.println(imu.Temperature_Time);
      
    }
    
    /*

      if (imu.encode(p) ) {  // Reads byte from buffer.  Valid packet returns true.
      Serial.print("Here is something: ");
      Serial.println(imu.angular_rate_x.f);
      /*Serial.println("Here pitch");
      Serial.println(imu.pitch, HEX);

      Serial.println(imu.roll);
      Serial.println(imu.yaw, HEX);
      Serial.println(imu.roll_rate);
      }
    */

/* ***************************
    Serial.print("byte: ");
    if ( p == 's' || p == 'n' || p == 'p')
      Serial.println(p);
    else
      Serial.println(p, HEX);
****************************** */ 
    Serial.send_now();
    //}
  }
  if (timee.hasPassed(100) && activated) {
    if (p != 'D')
    {
      //Serial.println(p);
      //Serial.send_now();
    }
    timee.restart();
  }
  while ( Serial.available() )  message.process(Serial.read () );
  delayMicroseconds(100);
}
