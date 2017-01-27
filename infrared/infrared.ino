float Vcc = 5.0;
float dist;

void setup()
{
  Serial.begin(9600);
}
void loop()
{
  dist = Vcc*analogRead(A0)/1023;
  dist = 26.549 * pow(dist,-1.2091);
  Serial.print("dist = ");
  Serial.print(dist);
  Serial.println("cm");
  delay(300);
}

