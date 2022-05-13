#include <EEPROM.h>
#include "GravityTDS.h"

#define SensorPin A2      // the pH meter Analog output is connected with the Arduino’s Analog
#define TdsSensorPin A1
GravityTDS gravityTds;

//----------

float temperature = 25,tdsValue = 0;
int solenoidPin_stage1 = 4, solenoidPin_stage2_acid = 7, solenoidPin_stage2_base = 2, phPin = 13; //This is the output pin on the Arduino we are using
float tdsValue1 = 0, tdsValue2 = 0, tdsValue3 = 0;
bool stage3 = false, done = false;

unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;
float phvalue1 = 7, phvalue2 = 7, phvalue3 = 7, phvalueavg = 0;

void setup()
{
    Serial.begin(115200);
    gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
    gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
    gravityTds.begin();  //initialization

    // put your setup code here, to run once:
    pinMode(solenoidPin_stage1, OUTPUT); //Sets the pin as an output
    pinMode(solenoidPin_stage2_acid, OUTPUT);
    pinMode(solenoidPin_stage2_base, OUTPUT);
}
 
void loop()
{
  //digitalWrite(solenoidPin_stage1, HIGH);
  //delay(10000);
    //temperature = readTemperature();  //add your temperature sensor and read it
    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
    Serial.print(tdsValue,0);
    Serial.println("ppm");

    //-----------

    tdsValue3 = tdsValue2;
    tdsValue2 = tdsValue1;
    tdsValue1 = tdsValue;

    Serial.println("TDS_1 : " + String(tdsValue1) + "   " + "TDS_2 : " + String(tdsValue2) + "   " + "TDS_3 : " + String(tdsValue3));
    

    if (tdsValue1 <= 1200 && tdsValue2 <= 1200 && tdsValue3 <= 1200) {
      digitalWrite(solenoidPin_stage1, HIGH);  //Switch Solenoid ON
      Serial.println("Solenoid OPEN");
      done = false;
    } else {
      digitalWrite(solenoidPin_stage1, LOW);   //Switch Solenoid OFF
      Serial.println("Solenoid Closed");
      done = true;
    }
    if (done == false) {
        stage3 = true;
      } else {
        stage3 = false;
      }

    //--------------- pH ------------------
    if (stage3) {
      for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
      { 
        buf[i]=analogRead(SensorPin);
        delay(10);
      }
      for(int i=0;i<9;i++)        //sort the analog from small to large
      {
        for(int j=i+1;j<10;j++)
        {
          if(buf[i]>buf[j])
          {
            temp=buf[i];
            buf[i]=buf[j];
            buf[j]=temp;
          }
        }
      }
      avgValue=0;
      for(int i=2;i<8;i++)                      //take the average value of 6 center sample
        avgValue+=buf[i];
      float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
      phValue=3.5*phValue;   
      
      phvalue3 = phvalue2;
      phvalue2 = phvalue1;
      phvalue1 = phValue;

      phvalueavg = (phvalue1+phvalue2+phvalue3)/3;
      Serial.print("    pH:");
      Serial.print(phvalueavg,2);
      Serial.println(" ");


      if (phvalueavg > 8.4) {
        digitalWrite(solenoidPin_stage2_acid, HIGH);
        delay(1500);
        digitalWrite(solenoidPin_stage2_acid, LOW);
      } else if (phvalueavg < 6.5) {
        digitalWrite(solenoidPin_stage2_base, HIGH);
        delay(1500);
        digitalWrite(solenoidPin_stage2_base, LOW);
      }
        done = true;
    }
    delay(2000);
    
    Serial.println("-------------------------------------");
}
