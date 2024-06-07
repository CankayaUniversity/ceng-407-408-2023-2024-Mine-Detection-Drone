#define res 9

void setup()
{
  Serial.begin(115200);
  pinMode(8,OUTPUT);
  pinMode(res,INPUT);
  digitalWrite(8,HIGH);
  delay(100);
  digitalWrite(8,LOW);
  delay(100);
  digitalWrite(8,HIGH);
  delay(100);
  digitalWrite(8,LOW);
  delay(100);
}
void loop(){
  int adcVal = 0;
  int adcVal2 = 0;
  float referance = 0;
  float voltage = 0;
  float voltage2 = 0;
  float val = 0;
  
  
  if (digitalRead(res) == 1){
    start:
    adcVal = analogRead(0);
    adcVal2 = analogRead(1);
    voltage = adcVal * 5.0f / 1023.0f;
    voltage2 = adcVal2 * 5.0f / 1023.0f;
    val= (voltage + voltage2)/2;
    referance = val;
    while (true){  
      adcVal = analogRead(0);
      adcVal2 = analogRead(1);
      voltage = adcVal * 5.0f / 1023.0f;
      voltage2 = adcVal2 * 5.0f / 1023.0f;
      val = (voltage + voltage2)/2;
      if (val >= (referance + 0.15) || val <= (referance - 0.15)){
        digitalWrite(8,HIGH);
        delay(20);
      }
      else{
        digitalWrite(8,LOW);
        delay(20);
      }
      Serial.print(val);
      Serial.print("-");
      Serial.println(referance);
      delay(10);

      if(digitalRead(res) == 1){
        referance = 0;
        goto start;
      }
    }
  }
}