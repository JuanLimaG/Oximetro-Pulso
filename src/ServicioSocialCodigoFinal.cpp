#include "arduinoFFT.h"       //Libreria para FFT
#include <Wire.h>             //Libreria para comunicacion i2c
#include "MAX30105.h"         //Libreria Sparkfun
#include "CircularBufferLib.h"   //Libreria para FFT
#include "spo2_algorithm.h"      //Libreria para Spo2
#include "Algoritmo_ritmo.h"     //Libreria para Ritmo Cardiaco
#include <Arduino.h>
MAX30105 particleSensor;         //Declaracion de la Clase MAX30105

#define MUESTRAS 512            //Vector de 512 muestras
#define FS 100                     
double vReal[MUESTRAS];
double vImag[MUESTRAS];

int b1=0;
double HR=0;               //Variable que guardara el ritmo cardiaco
uint32_t irBuffer[800];   //datos del sensor de infrarrojos LED
uint32_t redBuffer[800];  //datos del sensor LED rojo


//Variables donde se van a guardar los valores promediados
uint32_t Vprom4IR[100];
uint32_t Vprom4RED[100];
uint32_t Vprom8IR[800];
uint32_t Vprom8RED[800];
///////////////////////////////////////////////
//Variables auxiliares a utilizar
int32_t bufferLength;//longitud de datos
int32_t bufferlength2 = 100;
int n; 
////////////////////////////////////
//Variables donde se almacenan los resultados
int32_t spo22; // Valor de SPO2
int8_t validSPO22; //Indicador para mostrar si el cálculo de SPO2 es válido
int32_t heartRate2; //Valor de frecuencia cardíaca
int8_t validHeartRate2; //Indicador para mostrar si el cálculo de la frecuencia cardíaca es válido

int32_t spo2; //Valor de SPO2
int8_t validSPO2; //Indicador para mostrar si el cálculo de SPO2 es válido
int32_t heartRate; //Valor de frecuencia cardíaca
int8_t validHeartRate;

arduinoFFT FFT = arduinoFFT(vReal, vImag,MUESTRAS,FS); 
CircularBuffer<double> CBIRFFT(MUESTRAS); //Buffer circular para las muestras IR

void datos();
void fft();

//Codigo que se ejecuta cada que se es encendido el sensor 
void setup() {                              
Serial.begin(115200);  //Inicio de comunicacion serial a 115200 baudios
if (particleSensor.begin() == false)  //si no se detecta el sensor entonces se muestra 
  {
    Serial.println("MAX30102 no disponible");  //el mensaje 
    while (1);   //se queda indefinidamente hasta que el sensor sea detectado
  }        //si se detecta el sensor entonces se setea con los parametros ya establecidos para nuestros fines
 byte ledBrightness = 60; //Options: 0=Off to 255=50mA
 byte sampleAverage =1; //Options: 1, 2, 4, 8, 16, 32
 byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
 int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
 int pulseWidth = 411; //Options: 69, 118, 215, 411
 int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
 particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //se usa la funcion setup de la libreria sparkfun            
}

//Funcion que se ejecuta indefinidamente en el sensor
void loop() {  
datos();   //llamado a la funcion de datos 
}

//funcion de toma de datos y calculo de algoritmos
void datos(){
  bufferLength =800; //la longitud del búfer de 100 almacena 4 segundos de muestras que se ejecutan a 25 sps
  n = 0;
  //leer las primeras 100 muestras y determinar el rango de la señal
  for (int i = 0 ; i < bufferLength ; i++)
  {  
     //obtencion de los primero 800 datos
    while (particleSensor.available() == false) //tenemos nuevos datos?
    particleSensor.check(); //Verifique el sensor en busca de nuevos datos
   
    irBuffer[i] = particleSensor.getRed(); //Invertir los buffers
    CBIRFFT.Add(irBuffer[i]);
    b1=b1+1;
    redBuffer[i] = particleSensor.getIR(); 
     
    particleSensor.nextSample(); //Se ha terminado con esta muestra, así que pase a la siguiente.
    n = n +1;
    fft();  //Llamado y ejecucion de la funcion fft()
  }
  
 //promedio movil de 400 datos, de 4 valores 
  int k;
  int j = 0;
  
  for(k=0; k< 100; k++)
    {
    Vprom4IR[k]=( irBuffer[j]+irBuffer[j+1]+ irBuffer[j+2]+ irBuffer[j+3])/(int)4;
    Vprom4RED[k]=( redBuffer[j]+redBuffer[j+1]+ redBuffer[j+2]+ redBuffer[j+3])/(int)4;   
    j = j +4;  
    }
/////////////////////////////////////
  j=0;
  //Promedio movil de 800 datos, de 24 valores
  for(k=0; k< 800; k++)
  {
  Vprom8IR[k]=( irBuffer[k]+irBuffer[k+1]+ irBuffer[k+2]+ irBuffer[k+3] +irBuffer[k+4] +irBuffer[k+5]+ irBuffer[k+6] + irBuffer[k+7] + irBuffer[k+8] + irBuffer[k+9] + irBuffer[k+10] + irBuffer[k+11] + irBuffer[k+12] + irBuffer[k+13] + irBuffer[k+14] + irBuffer[k+15]+  irBuffer[k+16] + irBuffer[k+17] + irBuffer[k+18] + irBuffer[k+19] + +  irBuffer[k+20] + irBuffer[k+21] + irBuffer[k+22] + irBuffer[k+23])/(int)24;
  Vprom8RED[k]=( redBuffer[k]+redBuffer[k+1]+ redBuffer[k+2]+ redBuffer[k+3] +redBuffer[k+4] +redBuffer[k+5]+ redBuffer[k+6] + redBuffer[k+7] + redBuffer[k+8] + redBuffer[k+9] + redBuffer[k+10] + redBuffer[k+11] + redBuffer[k+12] + redBuffer[k+13] + redBuffer[k+14] + redBuffer[k+15] + redBuffer[k+16] + redBuffer[k+17] + redBuffer[k+18] + redBuffer[k+19] + redBuffer[k+20] + redBuffer[k+21] + redBuffer[k+22] + redBuffer[k+23])/(int)24;   
  }
     
  

  //calcular la frecuencia cardíaca y la SpO2 después de las primeras 100 muestras (primeros 4 segundos de las muestras)
  //Llamado de la función, la primera calcula el valor del SpO2 y la otra del HR, los valores se guardan en variables diferentes
  maxim_heart_rate_and_oxygen_saturation(Vprom4IR, bufferlength2, Vprom4RED, &spo2, &validSPO2, &heartRate, &validHeartRate);
  CalcularRitmo(Vprom8IR, bufferLength, Vprom8RED, &spo22, &validSPO22, &heartRate2, &validHeartRate2);    

      Serial.print(F("SPO2="));
      Serial.println(spo2, DEC);

      Serial.print(F("SPO2Valido="));
      Serial.println(validSPO2, DEC);  

      Serial.print(F("RITMO="));
      Serial.print(heartRate2, DEC);
}

//Funcion FFT
void fft(){
  if(b1==512){
 
CBIRFFT.CopyTo(vReal);
  for(int i=0; i<MUESTRAS;i++){
    vImag[i] = 0.0; 
  }

 
 FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
 FFT.Compute(FFT_FORWARD); 

 FFT.ComplexToMagnitude(); 
HR = FFT.MajorPeak()*60;
Serial.print(" , RITMO-FFT=  ");
Serial.println(HR, 8);
b1=0;
  }
}
