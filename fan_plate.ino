#include <LiquidCrystal.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
int16_t ax, ay, az,gx, gy, gz;
float Acceleration_aci;
float Gyro_aci;
float Toplam_aci;
float gecen_zaman, time, onceki_zaman;
float rad_to_deg = 180/3.141592654;
LiquidCrystal lcd(12,11,10,9,8,7);
double setdeger=0;
double giris;
double cikis;
double kp,ki,kd;
PID myPID(&giris, &cikis, &setdeger, kp, ki, kd, DIRECT);
void setup() {
Wire.begin();
Wire.beginTransmission(0x68);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);
lcd.begin(16,2);
myPID.SetOutputLimits(0,255);
myPID.SetMode(AUTOMATIC);

pinMode(5,OUTPUT);
}
void aci_hesapla(){
Wire.beginTransmission(0x68);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(0x68,6,true);

onceki_zaman = time;
time = millis();
gecen_zaman = (time - onceki_zaman) / 1000;
ax=Wire.read()<<8|Wire.read();
ay=Wire.read()<<8|Wire.read();
az=Wire.read()<<8|Wire.read();
Acceleration_aci = atan((ay/16384.0)/sqrt(pow((ax/16384.0),2)+pow((az/16384.0),2)))*rad_to_deg;
Wire.beginTransmission(0x68);
Wire.write(0x43);
Wire.endTransmission(false);
Wire.requestFrom(0x68,4,true);
gx=Wire.read()<<8|Wire.read();
Gyro_aci = gz/131.0;
Toplam_aci = 0.98 *(Toplam_aci + Gyro_aci*gecen_zaman) + 0.02*Acceleration_aci;
return Toplam_aci;
}
void loop() {
kp=analogRead(A1)*0.01;
ki=analogRead(A2)*0.01;
setdeger=map(analogRead(A0),0,1023,0,60);
kd=analogRead(A3)*0.01;
lcd.setCursor(0,0);
lcd.print("S:");
lcd.setCursor(2,0);
lcd.print(setdeger);
lcd.setCursor(5,0);
lcd.print("R:");
lcd.setCursor(7,0);
lcd.print(Toplam_aci);
lcd.setCursor(10,0);
lcd.print("P:");
lcd.setCursor(12,0);
lcd.print(kp);
lcd.setCursor(0,1);
lcd.print("I:");
lcd.setCursor(2,1);
lcd.print(ki);
lcd.setCursor(7,1);
lcd.print("D:");
lcd.setCursor(9,1);
lcd.print(kd);
myPID.SetTunings(kp, ki, kd);
aci_hesapla();
giris=Toplam_aci;
myPID.Compute();

analogWrite(5,cikis);
Serial.print(setdeger);
Serial.print("\t");
Serial.println(giris);
}
