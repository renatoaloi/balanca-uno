#include <ModbusMaster.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "HX711.h"
#include <SoftwareSerial.h>

#define MAX485_DE      3
#define MAX485_RE_NEG  4

#define DOUT  A0
#define CLK   A1

#define QTDE_TEMPO 5000
#define QTDE_TEMPO_CICLO 60000
#define PESO_MINIMO 0.01
#define PESOS_SIZE 20
#define PORCENTO_CORTE 0.8

#define ALTITUDE 530.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters

// --- Constantes ---
const float pi = 3.14159265;     //Número de pi
int period = 5000;               //Tempo de medida(miliseconds)
int delaytime = 15000;            //Invervalo entre as amostras (miliseconds)
unsigned long startTime;
unsigned long delayanemom;
bool startedInterrupt = false;
int radius = 147;                //Raio do anemometro(mm)

// --- Variáveis Globais ---
unsigned int Sample  = 0;        //Armazena o número de amostras
volatile unsigned int counter = 0;        //Contador para o sensor  
unsigned int RPM = 0;            //Rotações por minuto
float speedwind = 0;             //Velocidade do vento (m/s)
float windspeed = 0;             //Velocidade do vento (km/h)
unsigned int contaAnemom = 0;

unsigned long contaTempo = 0;
unsigned long contaTempoCiclo = 0;
float units = 0.0;
int contaPesos = 0;
float pesos[PESOS_SIZE];
float muitoDiferenteValor = 0.0; // variável de referencia do cálculo dos 80%

float calibration_factor = -279560.00; //-7050 trabalhei para minha configuração de escala máxima de 440lb

double T,P,p0,a;

uint8_t deviceID = 1;
uint8_t resultMain;
float temp485, umid485;
int lux485;


SFE_BMP180 pressure;
HX711 scale;
SoftwareSerial swSerial(10, 11); // RX, TX
// Instancia Objeto ModbusMaster
ModbusMaster node;
// Instancia Objeto SoftwareSerial
SoftwareSerial mySerial(12, 13); // RX, TX

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
 
void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void reiniciaPesos() {
  for (int i = 0; i < PESOS_SIZE; i++) {
    pesos[i] = 0.0;
  }
}

void getTempUmiLux() {
  // Modbus ID 1 - Controlador de temperatura
  node.begin(deviceID, mySerial);
  delay(20);
  resultMain = node.readHoldingRegisters(0x0001 - 1, 3);
  delay(20);

  if (resultMain == node.ku8MBSuccess) {
    
    Serial.print(F("Temperatura =  "));
    Serial.println(node.getResponseBuffer(0x00));
    temp485 = (float)(node.getResponseBuffer(0x00) / 10.0);
    Serial.println(temp485, 2);
    Serial.print(F("Umidade  = "));
    Serial.println(node.getResponseBuffer(0x01));
    umid485 = (float)(node.getResponseBuffer(0x01) / 10.0);
    Serial.println(umid485, 2);
    Serial.print(F("Lux = "));
    lux485 = node.getResponseBuffer(0x03);
    Serial.println(lux485);
    } else {
    Serial.print(F("ID2MSGN"));
    Serial.println(F("Falha de comunicacao"));
  }
}

void checkBMP180() {
  char status;

  Serial.println();
  Serial.print(F("provided altitude: "));
  Serial.print(ALTITUDE,0);
  Serial.print(F(" meters, "));
  Serial.print(ALTITUDE*3.28084,0);
  Serial.println(F(" feet"));
  
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print(F("temperature: "));
      Serial.print(T,2);
      Serial.print(F(" deg C, "));
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(F(" deg F"));
      
      // Start a pressure measurement:
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print(F("absolute pressure: "));
          Serial.print(P,2);
          Serial.print(F(" hPa, "));
          Serial.print(P*0.0295333727,2);
          Serial.println(F(" inHg"));

          // The pressure sensor returns abolute pressure, which varies with altitude.
          p0 = pressure.sealevel (P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print(F("relative (sea-level) pressure: "));
          Serial.print(p0,2);
          Serial.print(F(" mb, "));
          Serial.print(p0*0.0295333727,2);
          Serial.println(F(" inHg"));

          // On the other hand, if you want to determine your altitude from the pressure reading,
          a = pressure.altitude(P,p0);
          Serial.print(F("computed altitude: "));
          Serial.print(a,0);
          Serial.print(F(" meters, "));
          Serial.print(a*3.28084,0);
          Serial.println(F(" feet"));
        }
        else Serial.println(F("error retrieving pressure measurement\n"));
      }
      else Serial.println(F("error starting pressure measurement\n"));
    }
    else Serial.println(F("error retrieving temperature measurement\n"));
  }
  else Serial.println(F("error starting temperature measurement\n"));
}

void calcAnemom() {
  if (delayanemom < millis()) {
    
    if (Sample > 0) {
      contaAnemom = counter;
      Serial.println(F("   finished."));
      Serial.print(F("Counter: "));
      Serial.print(counter);
      Serial.print(F(";  RPM: "));
      RPMcalc();
      Serial.print(RPM);
      Serial.print(F(";  Wind speed: "));
      WindSpeed();
      Serial.print(windspeed);
      Serial.print(F(" [m/s] "));
      SpeedWind();
      Serial.print(speedwind);
      Serial.print(F(" [km/h] "));  
      Serial.println();
    }

    Sample++;
    Serial.print(Sample);
    Serial.print(F(": Start measurement..."));
    windvelocity();
    
    delayanemom = millis() + delaytime;
  }
}

//Função para medir velocidade do vento
void windvelocity()
{
  if (!startedInterrupt) {
    startedInterrupt = true;
    speedwind = 0;
    windspeed = 0;
    counter = 0;  
    attachInterrupt(0, addcount, RISING);     
    startTime = millis() + period;
    //while(millis() < startTime + period) {}
    // detachInterrupt(0);
  }
}


//Função para calcular o RPM
void RPMcalc()
{
  RPM=((counter)*60)/(period/1000);  // Calculate revolutions per minute (RPM)
}


//Velocidade do vento em m/s
void WindSpeed()
{
  windspeed = ((4 * pi * radius * RPM)/60) / 1000;  //Calcula a velocidade do vento em m/s
 
} //end WindSpeed


//Velocidade do vento em km/h
void SpeedWind()
{
  speedwind = (((4 * pi * radius * RPM)/60) / 1000)*3.6;  //Calcula velocidade do vento em km/h
 
} //end SpeedWind


//Incrementa contador
void addcount()
{
  counter++;
} 

void setup() {
  swSerial.begin(9600);
  Serial.begin(9600);
  Serial.println(F("Esboço de calibração HX711"));
  Serial.println(F("Remova todo o peso da escala"));
  Serial.println(F("Após as leituras começarem, coloque o peso conhecido em escala"));
  Serial.println(F("Pressione + ou a para aumentar o fator de calibração"));
  Serial.println(F("Pressione - ou z para diminuir o fator de calibração"));

  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare(); //Repor a escala para 0
  delay (500);

  long zero_factor = scale.read_average(); //Obter uma leitura de linha de base
  Serial.print(F("Zero factor: ")); //Isso pode ser usado para eliminar a necessidade de tarar a balança. Útil em projetos de escala permanente.
  Serial.println(zero_factor);

  reiniciaPesos();

  if (pressure.begin())
    Serial.println(F("BMP180 init success"));
  else
  {
   Serial.println(F("BMP180 init fail\n\n"));
    while(1); // Pause forever.
  }

  pinMode(2, INPUT);        //configura o digital 2 como entrada
  digitalWrite(2, HIGH);    //internall pull-up active
  delayanemom = millis() + delaytime;

  // Atribui pinos como saída
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
 
  // inicializa modo de recebimento
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
 
  // Atribui velocidade de comunicação (Baud Rate)
  mySerial.begin(9600);
 
  // Callbacks - Permite configurar o transeiver RS485 corretamente
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {

  // verificando se é pra desligar o interrupt do anemometro
  if(startTime < millis() && startedInterrupt) {
    startedInterrupt = false;
    detachInterrupt(0);
  }

  scale.set_scale(calibration_factor); //Ajuste para este fator de calibração
  units = scale.get_units();
  delay (500);

  Serial.print(F("Leituras: "));
  Serial.print(units, 3);
  Serial.print(F(" Kg")); //Altere isso para kg e reajuste o fator de calibração se você seguir as unidades 
  Serial.print(F(" calibration_factor: "));
  Serial.print(calibration_factor);
  Serial.println();

  if(Serial.available())
  {
    char temp = Serial.read();
    if(temp == '+' || temp == 'a')
      calibration_factor += 10;
    else if(temp == '-' || temp == 'z')
      calibration_factor -= 10;
  }

  // Registra os pesos a cada 5 segundos
  if (contaTempo < millis()) {

    // calcula peso
    if (units > PESO_MINIMO) {
      // se peso for maior que peso mínimo, então registra
      if (contaPesos < PESOS_SIZE - 1) {
        pesos[contaPesos++] = units;
      } else Serial.print(F("Estouro de array!"));
    }

    // verifica sensor BMP180
    checkBMP180();

    // calcula Anemometro
    calcAnemom();

    // pega temperatura, umidade e lux do sensor RS485
    getTempUmiLux();
    
    contaTempo = millis() + QTDE_TEMPO;
  }

  // Executa a média a cada minuto
  if (contaTempoCiclo < millis()) {

    // desconsiderando o menor valor
    float menorValor = 9999.0;
    int idxMenorValor = -1;
    for (int i = 0; i < PESOS_SIZE; i++) {
      if (pesos[i] < menorValor) {
        menorValor = pesos[i];
        idxMenorValor = i;
      }
    }

    // desconsiderando o maior valor
    float maiorValor = 0.0;
    int idxMaiorValor = -1;
    for (int i = 0; i < PESOS_SIZE; i++) {
      if (pesos[i] > maiorValor) {
        maiorValor = pesos[i];
        idxMaiorValor = i;
      }
    }

    // descartando valores maiores que 80% que o valor de referencia
    int idxMuitoDiferenteValor[PESOS_SIZE];
    int contaMuitoDiferenteValor = 0;
    memset(idxMuitoDiferenteValor, -1, PESOS_SIZE);
    for (int i = 0; i < PESOS_SIZE; i++) {
      if (muitoDiferenteValor == 0.0 && i != menorValor && i != maiorValor && pesos[i] > PESO_MINIMO) {
         muitoDiferenteValor = pesos[i];
      }
      else if ((muitoDiferenteValor * (1 + PORCENTO_CORTE)) < pesos[i]) {
        if (contaMuitoDiferenteValor < (PESOS_SIZE - 1)) {
          idxMuitoDiferenteValor[contaMuitoDiferenteValor++] = i;
        }
      }
    }

    // Computando a media
    float soma = 0.0;
    int qtde = 0;
    for (int i = 0; i < PESOS_SIZE; i++) {

      // Verificando por valores muito diferentes
      bool mustContinue = false;
      for (int j = 0; j < contaMuitoDiferenteValor; j++) {
        if (idxMuitoDiferenteValor[j] == i) {
          mustContinue = true;
          break;
        }
      }
      if (mustContinue) continue;
      
      if (i != menorValor && i != maiorValor && pesos[i] > PESO_MINIMO) {
         soma += pesos[i];
         qtde++;
      }
    }

    // valores usados para computar a média
    Serial.print(F("Valores utilizados para computar a média: "));
    for (int i = 0; i < PESOS_SIZE; i++) {
      if (i != menorValor && i != maiorValor && pesos[i] > PESO_MINIMO) {
         Serial.print(pesos[i], 3);
         Serial.print(F(", "));
      }
    }
    Serial.println();

    // Media final
    float media = 0.0;

    if (qtde > 0) {
      media = (float)(soma / qtde);
    }

    // Monta mensagem serial
    String  sSerial = "pe=";
            sSerial += String(media, 3);
            sSerial += ",an=";
            sSerial += String(contaAnemom);
            sSerial += ",tp=";
            sSerial += String(temp485, 2);
            sSerial += ",um=";
            sSerial += String(umid485, 2);
            sSerial += ",lu=";
            sSerial += String(lux485);
            sSerial += ",pr=";
            sSerial += String(P, 2);

    Serial.print(F("Media de 1 minuto: "));
    Serial.println(sSerial);

    // Envia para o ESP07
    swSerial.println(sSerial);
    
    contaPesos = 0;
    reiniciaPesos();
    contaTempoCiclo = millis() + QTDE_TEMPO_CICLO;
    contaTempo = millis() + QTDE_TEMPO;
  }
  else {
    // se não passou 1 minuto faz outras tarefas
    
  }
}
