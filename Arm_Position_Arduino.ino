//Adriano Trindade 
//e-mail  adtrin@hotmail.com
//El proyecto consiste, en enviar la inclinacion del brazo + posicion, para un display LCD de 2 lineas 16 columnas y tambem
//por el puerto serial, siendo ese ultimo, para la computadora que va graficar eso en una pantalla las coordinadas XY + Inclinacion.


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2); //Para Display I2C
float coordinadaX,coordinadaY; //tiempo que demora en llegar el eco
float t,distancia ;

struct UltrasonidoXY
{
   uint8_t fim   ;
   float  X      ;
   float  Y      ;
   
   
};


struct Inclinacion 
{
   uint8_t fim    ;
   float   aX     ;
   float   aY     ;
     
  
};

void enviaDatos(byte *XY, int ancho)
{
 if (Serial.available()>0){

  Serial.write(XY, ancho);
 }
}
//enviaDatos((byte*)&XY, sizeof(XY));



float XY (const int Trigger, const int Echo)
{
  digitalWrite(Trigger, LOW);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  distancia =  t *0.034/2 ;
  return distancia;  
}
 
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0 // least significant bit per g) bit //(Segun datasheet)
// bit menos significativo/ g siendo g = 9.81m/s2 
#define G_R 131.0 //(Segun datasheet)
 
//Conversion de radianes a grados 180/P
#define RAD_A_DEG  57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores sin refinar
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[2];
float Angle[2];
float AZ;

void setup()
{
  pinMode(2, OUTPUT);///Trigger
  pinMode(3, INPUT);//Echo
  pinMode(8, OUTPUT);///Trigger
  pinMode(9, INPUT);//Echo  
  digitalWrite(2, LOW);
  digitalWrite(8, LOW);
  
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);

Serial.begin(9600);
lcd.init();              // initialize the lcd 
lcd.backlight();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////CALCULOS DE ANGULOS CONFORME COORDINADAS Y ENVIO DE LAS COORDINADAS DE LOS SENSORES DE DISTANCIA //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  struct UltrasonidoXY ultraXY; 
 //Cordinadas venidas del medidor de altura
 coordinadaX  =    XY(2,3)     ;
 coordinadaY  =    XY(8,9)     ;
 ultraXY.X    =    coordinadaX ;
 ultraXY.Y    =    coordinadaY ;
 ultraXY.fim =    'a'          ; //Fin de la altura
 
 
 

 
   //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
    //A partir de los valores del acelerometro, se calculan los angulos Y, X
    //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
   GyX=Wire.read()<<8|Wire.read();
   GyY=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   AZ = (AcZ/A_R);
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*0.010) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*0.010) + 0.02*Acc[1];
 
//------------------------------------------------------------------------------------------------------------------------------------------------//   
  struct Inclinacion inclinacion;
  //ESTRUTURA DE PARA ENVIO DE LOS ANGULOS PARA EL PUERTO SERIAL  
//------------------------------------------------------------------------------------------------------------------------------------------------//   
  
  //inclinacion.fim = 'F';
  inclinacion.aX  = Angle[0]  ;
  inclinacion.aY  = Angle[1]  ;
  inclinacion.fim =      'b'  ;//Fin de la inclinacion 
  //------------------------------------------------------------------------------------------------------------------------------------------------//   
   enviaDatos((byte*)&inclinacion, sizeof(inclinacion));
  delay(10);
  enviaDatos((byte*)&ultraXY, sizeof(ultraXY));
  delay(10);
//------------------------------------------------------------------------------------------------------------------------------------------------//   
 
   //MUESTRO LOS VALORES EN EL DISPLAY COMO REFERENCIA DE PRUEBA
   lcd.setCursor(0, 0);
   lcd.print("Z");  
   //lcd.print(Angle[0],2);
   lcd.print(AZ,2);
   lcd.setCursor(0, 1);
   lcd.print("Y");
   lcd.print(Angle[1],2);
   lcd.setCursor(8,0);
   lcd.print("Xc");
   lcd.print(coordinadaX,1);
   lcd.setCursor(8,1);
   lcd.print("Yc");
   lcd.print(coordinadaY,1);   
//Adriano Trindade 
//e-mail  adtrin@hotmail.com
//El proyecto consiste, en enviar la inclinacion del brazo + posicion, para un display LCD de 2 lineas 16 columnas y tambem
//por el puerto serial, siendo ese ultimo, para la computadora que va graficar eso en una pantalla las coordinadas XY + Inclinacion.


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2); //Para Display I2C
float coordinadaX,coordinadaY; //tiempo que demora en llegar el eco
float t,distancia ;

struct UltrasonidoXY
{
   uint8_t fim   ;
   float  X      ;
   float  Y      ;
   
   
};


struct Inclinacion 
{
   uint8_t fim    ;
   float   aX     ;
   float   aY     ;
     
  
};

void enviaDatos(byte *XY, int ancho)
{
 if (Serial.available()>0){

  Serial.write(XY, ancho);
 }
}
//enviaDatos((byte*)&XY, sizeof(XY));



float XY (const int Trigger, const int Echo)
{
  digitalWrite(Trigger, LOW);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  distancia =  t *0.034/2 ;
  return distancia;  
}
 
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0 // least significant bit per g) bit //(Segun datasheet)
// bit menos significativo/ g siendo g = 9.81m/s2 
#define G_R 131.0 //(Segun datasheet)
 
//Conversion de radianes a grados 180/P
#define RAD_A_DEG  57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores sin refinar
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[2];
float Angle[2];
float AZ;

void setup()
{
  pinMode(2, OUTPUT);///Trigger
  pinMode(3, INPUT);//Echo
  pinMode(8, OUTPUT);///Trigger
  pinMode(9, INPUT);//Echo  
  digitalWrite(2, LOW);
  digitalWrite(8, LOW);
  
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);

Serial.begin(9600);
lcd.init();              // initialize the lcd 
lcd.backlight();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////CALCULOS DE ANGULOS CONFORME COORDINADAS Y ENVIO DE LAS COORDINADAS DE LOS SENSORES DE DISTANCIA //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  struct UltrasonidoXY ultraXY; 
 //Cordinadas venidas del medidor de altura
 coordinadaX  =    XY(2,3)     ;
 coordinadaY  =    XY(8,9)     ;
 ultraXY.X    =    coordinadaX ;
 ultraXY.Y    =    coordinadaY ;
 ultraXY.fim =    'a'          ; //Fin de la altura
 
 
 

 
   //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
    //A partir de los valores del acelerometro, se calculan los angulos Y, X
    //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
   GyX=Wire.read()<<8|Wire.read();
   GyY=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   AZ = (AcZ/A_R);
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*0.010) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*0.010) + 0.02*Acc[1];
 
//------------------------------------------------------------------------------------------------------------------------------------------------//   
  struct Inclinacion inclinacion;
  //ESTRUTURA DE PARA ENVIO DE LOS ANGULOS PARA EL PUERTO SERIAL  
//------------------------------------------------------------------------------------------------------------------------------------------------//   
  
  //inclinacion.fim = 'F';
  inclinacion.aX  = Angle[0]  ;
  inclinacion.aY  = Angle[1]  ;
  inclinacion.fim =      'b'  ;//Fin de la inclinacion 
  //------------------------------------------------------------------------------------------------------------------------------------------------//   
   enviaDatos((byte*)&inclinacion, sizeof(inclinacion));
  delay(10);
  enviaDatos((byte*)&ultraXY, sizeof(ultraXY));
  delay(10);
//------------------------------------------------------------------------------------------------------------------------------------------------//   
 
   //MUESTRO LOS VALORES EN EL DISPLAY COMO REFERENCIA DE PRUEBA
   lcd.setCursor(0, 0);
   lcd.print("Z");  
   //lcd.print(Angle[0],2);
   lcd.print(AZ,2);
   lcd.setCursor(0, 1);
   lcd.print("Y");
   lcd.print(Angle[1],2);
   lcd.setCursor(8,0);
   lcd.print("Xc");
   lcd.print(coordinadaX,1);
   lcd.setCursor(8,1);
   lcd.print("Yc");
   lcd.print(coordinadaY,1);   
   delay(250); 
   delay(250); 
}
