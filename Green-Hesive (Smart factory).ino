#include <Servo.h> 
Servo myservo;

#define ldr1 A0 
#define ldr2 A1 
#define curent A2
#define volt A4 

#define echoPin 4
#define trigPin 3 
#define stirer 8
#define pump 7 
int sti = 1;
int sto = 1;
int pos = 90; 
int tolerance = 20;
int cmd = -1;
float cur = 0;
float Vout = 0.00;
float Vin = 0.00;
float R1 = 10000.00; // resistance of R1 (100K) 
float R2 = 1000.00; // resistance of R2 (10K)
int batterypercent;
int val = 0;
int flag = 0;
int blue = 9;
int green = 10;
int yellow = 11;
int red = 12;
int ultr = 1;
int stirerlight = 5;
int pumplight = 6;
int sol = 1;
int l = 150;
void setup(){
//Motors
pinMode(pump, OUTPUT);
pinMode(stirer, OUTPUT);
// lamps
pinMode(blue, OUTPUT); 
pinMode(green, OUTPUT); 
pinMode(yellow, OUTPUT);
pinMode(red, OUTPUT);  
pinMode(stirerlight, OUTPUT);
pinMode(pumplight, OUTPUT);  
// Battery percentage
pinMode(volt, INPUT);
// Current
pinMode(curent, INPUT);
// Ultrasonic
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
// Solar Tracker
myservo.attach(2); 
pinMode(ldr1, INPUT); 
pinMode(ldr2, INPUT);
myservo.write(pos);  

delay(1000);
Serial.begin(9600);      
}
void loop(){
  if(Serial.available() > 0){
    cmd = Serial.read();
    flag = 1;
  }
  if(flag == 1){
         if(cmd == '1') sol = 1;
    else if(cmd == '6') sti = 1;
    else if(cmd == '2') ultr = 0;
    else if(cmd == '4') ultr = 1;
    else if(cmd == '8') sti = 0;
    else if(cmd == '3') sol = 0;
    flag = 0;
  }
    if(sol == 1) {
    int val1 = analogRead(ldr1);
    int val2 = analogRead(ldr2);
      
    if((abs(val1 - val2) <= tolerance) || (abs(val2 - val1) <= tolerance)){}else {
    if(val1 > val2){pos = pos+1;}
    if(val1 < val2){pos = pos-1;}}
    if(pos > 135) {pos = 135;}
    if(pos < 0) {pos = 45;} 
    myservo.write(pos);
  }else{
  float average = 0;
  if(sti == 1){
  for(int i = 0; i < 15; i++) 
  {
    average = average + (.0264 * analogRead(A2) -13.51) / 50;
    delay(1);
  }
  }
  if(sto == 1){
  val = analogRead(volt);
   Vout = (val * 5.00) / 1024.00;
   Vin = Vout / (R2/(R1+R2)); 
   if (Vin<0.09){Vin=0.00;}
   batterypercent = 123 - (123/pow((1+pow((Vin/3.7),80)), 0.165)) ;
   if(batterypercent == 0){batterypercent = 10;}
  sto = 0;
  }
 if((sti == 1)&&(average < 1.2)){
  digitalWrite(stirer,HIGH);
  analogWrite(stirerlight, l);
  }else{
  digitalWrite(stirer,LOW);
  analogWrite(stirerlight, 0);
    sti = 0;
  }
  long duration;
  int distance; 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  if(ultr == 1) wp(distance); else wp(5);
 bp(batterypercent);
 Serial.print("BatteryPercent : ");
 Serial.println(batterypercent);
 delay(200);
 Serial.print("UltraSonic : ");
 Serial.println(distance);
 delay(200);
 Serial.print("Current : ");
 Serial.println(average, 2);
 delay(185);
 Serial.print("ServoPosition : ");
 Serial.println(pos);
 delay(200);
 myservo.write(90);}
 delay(50);
 cmd = 65;
}

void wp(int ul){  
  int u = ul;
  if(u < 10){
  digitalWrite(pump,HIGH);
  analogWrite(pumplight, l);
  }else{
  digitalWrite(pump,LOW);
  analogWrite(pumplight, 0);  
  }
}
void bp(int x){
int b = x;
  if(b > 80){
        analogWrite(blue, l);
        analogWrite(green, l);
        analogWrite(yellow, l);
        analogWrite(red, l);}
  else if(b > 60){
        analogWrite(green, l);
        analogWrite(yellow, l);
        analogWrite(red, l);}
else if(b > 30){
        analogWrite(yellow, l);
        analogWrite(red, l);}
else{
        analogWrite(red, l);}
}

int ultra(){
  long duration;
  int distance; 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}
