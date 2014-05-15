
#include <Wire.h>

// Define Constants
byte zaddr = 0x78;
float Ap = 0.05; 
float At = 0.10; 
float Bp = 0.95;
float Bt = 0.90;
int Pmax = 5;
int Pmin = 0;
int Tmax = 125;
int Tmin = -40;

// Define Variables
unsigned int Pdec;
unsigned int Tdec;
float Pres;
float Temp;


void setup()
{
  Wire.begin();          // join i2c bus (address optional for master)
  Serial.begin(9600);    // start serial for output
  delay(2000);
  Serial.println("Setup");
}

void loop()
{
  Pres = -1;
  Temp = -1;

  Wire.requestFrom(zaddr, byte(4));    // request 4 bytes from slave device
  while(Wire.available())    // slave may send less than requested
  {
    byte b = Wire.read();               //MSB pressure
    byte c = Wire.read();               //LSB pressure
    byte d = Wire.read();               //MSB temperature
    byte e = Wire.read();               //LSB temperature

    Pdec = b;
    Tdec = d;
    Pdec = Pdec<<8;
    Tdec = Tdec<<8;
    Pdec |= c;
    Tdec |= e;
  }

  Pres = ((float)Pdec/32767)*((Pmax-Pmin)/(Bp-Ap))+(Pmin-((Pmax-Pmin)/(Bp-Ap))*Ap);
  Temp = ((float)Tdec/32767)*((Tmax-Tmin)/(Bt-At))+(Tmin-((Tmax-Tmin)/(Bt-At))*At);


  Serial.print("Pressure = ");
  Serial.print(Pres);
  Serial.println(" bar");
  Serial.print("Temperature = ");
  Serial.print(Temp);
  Serial.println(" C");
  Serial.println();

  delay(1000);

}


