#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>

LIDARLite myLidarLite;
Servo myservo;

const int range = 60;
const int Step = 3;
const int l = (range/Step)+1;
float dist[l];

void setup()
{
  myLidarLite.begin(0, true); 
  myLidarLite.configure(0);
  pinMode(9,OUTPUT);
  myservo.attach(9); 
  Serial.begin(57600);
}

void loop()
{

  unsigned long int t2=millis();
  
  for(int pos = 90-(range/2); pos <= 90+(range/2); pos += Step) 
  {

    myservo.write(pos);
    delay(15);
    unsigned long int t=millis();
    float d=myLidarLite.distance()/100.0;
    if(d<0.1)
      d=30.0;
    else if (d>30.0)
      d=30.0;
    dist[(pos-range)/Step] = d;
    
    while (millis()-t < 10);
  }
//  Serial.println(millis()-t2);
  Serial.print("1st,");
  for (int i=0; i<l; i++)
  {
    Serial.print(dist[i]);
    Serial.print(",");
  }
  Serial.println();
  myservo.write(90-(range/2));
  delay(350);

  /*
  for(int pos = 90+(range/2); pos >= 90-(range/2); pos -= Step) 
  {

    myservo.write(90);
    delay(40);
    unsigned long int t=millis();
    dist[(pos-range)/Step] = myLidarLite.distance();
    while (millis()-t < 10);
  }
  Serial.print("2nd,");
  for (int i=0; i<l; i++)
  {
    Serial.print(dist[i]);
    Serial.print("\t");
  }
  Serial.println();
  */
  

//  Serial.println(millis()-t2);
}
