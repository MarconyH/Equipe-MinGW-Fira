#include <Arduino.h>
#include <PID_v1.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <HardwareSerial.h>

int rotacao_RPM_sensor2();
int rotacao_RPM_sensor1();

#define PINO_CH1 3
#define PINO_CH2 2
#define PINO_CH3 10
#define PINO_CH4 11

#define ENA 5 // ENA PWM Motor Esquerdo
#define ENB 6 // ENB PWM Motor Direito

#define IN1 7 // DIR Motor Esquerdo
#define IN2 7 // DIR Motor Esquerdo

#define IN3 8 // DIR Motor Direito
#define IN4 8 // DIR Motor Direito

#define xshutPinsE 12
#define xshutPinsC 9
#define xshutPinsD 4

// SoftwareSerial BTSerial(A0, A1); // RX, TX

VL53L1X sensorE;
VL53L1X sensorC;
VL53L1X sensorD;

// Variáveis Globais
float tamanho_carrinho = 13.5;
float tamanho_pista;

float distanciaE;
double distanciaC;
float distanciaD;

// Velocidade em rpm da roda esquerda e direita
int velocidadeD = 0;
int velocidadeE = 0;

unsigned long time;
/*Parâmetros para ajustar*/

// Robocore - fonte
// Declaracao das variaveis auxiliares para a verificacao do sentido
int estado_sensor1;
int ultimo_estado_sensor1;
boolean sentido_sensor1;

int estado_sensor2;
int ultimo_estado_sensor2;
boolean sentido_sensor2;

// Declaracao das variaveis auxiliares para o calculo da velocidade
unsigned long contador1_sensor1;
unsigned long contador2_sensor1;

unsigned long contador1_sensor2;
unsigned long contador2_sensor2;

const int NUMERO_CONTADORES = 2;
const int NUMERO_LEITURAS = 2;
// Variavel de numero de dentes do disco de leitura
const int NUMERO_DENTES = 10; // Altere se necessario

// Declaracao das variaveis auxiliares para a temporizacao de um minuto
unsigned long tempo_antes_sensor1 = 0;
unsigned long tempo_antes_sensor2 = 0;
const long MINUTO = 1000;
// Robocore - fonte

// Declaracao das variaveis auxiliares para a temporizacao de um minuto
unsigned long tempo_antes_direita_sensor1 = 0;
unsigned long tempo_antes_esquerda_sensor1 = 0;

unsigned long tempo_antes_direita_sensor2 = 0;
unsigned long tempo_antes_esquerda_sensor2 = 0;

const long tempo_atual = 1000;

//-----------------------------------------------

// Valor máximo 255 para potência total
// float VEL_MAX = 90;

/* Ajuste de alinhamento em reta */

float MAX_DELTA = 40;

float MAX_VOLTAGE = 92; // em voltagem
float MIN_PERCENT = 35; // em porcentagem

double DIS_MAX = 7.1;
double DIS_MIN = 0.1;

// PID PARA O SENSOR CENTRAL ((DES)ACELERAÇÃO)
double OutputC;
double SetpointCentral = 15;
double kpCentral = 5.0, kiCentral = 3.5, kdCentral = 2.0;
PID PIDc(&distanciaC, &OutputC, &SetpointCentral, kpCentral, kiCentral,
         kdCentral, REVERSE);

void ler_sensores()
{
  distanciaE = sensorE.read() - 20;
  sensorE.timeoutOccurred() ? distanciaE = 4000 : distanciaE = distanciaE;

  distanciaD = sensorD.read() - 20;
  sensorD.timeoutOccurred() ? distanciaD = 4000 : distanciaD = distanciaD;

  distanciaC = sensorC.read();
  sensorC.timeoutOccurred() ? distanciaC = 4000 : distanciaC = distanciaC;

  velocidadeE = rotacao_RPM_sensor2();
  velocidadeD = rotacao_RPM_sensor1();
}

// Robocore - Fonte

// Funcao de interrupcao
void contador_pulso2_sensor1()
{

  // Incrementa o contador
  contador2_sensor1++;

  // Verifica o sentido de rotacao do motor
  estado_sensor1 = digitalRead(PINO_CH2);
  if (ultimo_estado_sensor1 == LOW && estado_sensor1 == HIGH)
  {
    if (digitalRead(PINO_CH1) == LOW)
    {
      sentido_sensor1 = true;
    }
    else
    {
      sentido_sensor1 = false;
    }
  }
  ultimo_estado_sensor1 = estado_sensor1;
}

// Funcao de interrupcao
void contador_pulso1_sensor1()
{

  // Incrementa o contador
  contador1_sensor1++;
}

void contador_pulso2_sensor2()
{

  // Incrementa o contador
  contador2_sensor2++;

  // Verifica o sentido de rotacao do motor
  estado_sensor1 = digitalRead(PINO_CH4);
  if (ultimo_estado_sensor2 == LOW && estado_sensor2 == HIGH)
  {
    if (digitalRead(PINO_CH3) == LOW)
    {
      sentido_sensor2 = true;
    }
    else
    {
      sentido_sensor2 = false;
    }
  }
  ultimo_estado_sensor2 = estado_sensor2;
}

// Funcao de interrupcao
void contador_pulso1_sensor2()
{

  // Incrementa o contador
  contador1_sensor2++;
}

// Robocore - Fonte

void imprimeDistancias()
{
  Serial.print("Dis Esq: ");
  Serial.print(distanciaE);
  Serial.print(" mm  /  ");
  Serial.print("Dis Cen: ");
  Serial.print(distanciaC);
  Serial.print(" mm   /  ");
  Serial.print("Dis Dir: ");
  Serial.print(distanciaD);
  Serial.println(" mm");
}

float tratamento(float vel)
{
  vel = min(vel, 100);
  vel = max(vel, MIN_PERCENT);
  vel = (vel)*MAX_VOLTAGE / 100;
  return vel;
}

int ajuste_rpm(int velocidade_melhor_real = velocidadeD, int velocidade_pior_real = velocidadeE);

void acelera(float vel_esquerda, float vel_direita, int ajustar_rpm_var = 0)
{
  // int vel_direita_int = ceil(tratamento((vel_direita* OutputC / MAX_VOLTAGE), MIN_VOLTAGE_DIR));
  // int vel_esquerda_int = ceil(tratamento((vel_esquerda* OutputC / MAX_VOLTAGE), MIN_VOLTAGE_ESQ));

  int vel_direita_int = round(tratamento((vel_direita)));
  int vel_esquerda_int = round(tratamento((vel_esquerda)));

  if (ajustar_rpm_var)
  {
    int ajuste = ajuste_rpm();
    if (ajuste > 0)
    {
      vel_esquerda_int = ajuste;
    }
  }

  analogWrite(ENA, vel_esquerda_int);

  analogWrite(ENB, vel_direita_int);
}

void back()
{
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, MAX_VOLTAGE);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, MAX_VOLTAGE);
}

// unsigned long time_here_left = 0;
// unsigned long time_here_right = 0;
// unsigned long last_time = 0;

double output_func_math(int choice, float MIN_PERCENT, double referencia,
                        double referencia_2 = 0)
{
  double c;
  double a;
  switch (choice)
  {
  // cosseno
  case 0:
    return cos(referencia * acos(MIN_PERCENT / MAX_VOLTAGE) / DIS_MAX) *
           MAX_VOLTAGE;

  // quadratica
  case 1:
    c = MAX_VOLTAGE;
    a = (MIN_PERCENT - MAX_VOLTAGE) / DIS_MAX * DIS_MAX;
    return referencia * referencia * a + c;

  // linear
  case 2:
    // valores iniciais a = -50.0 e c = 15.5
    a = -70;
    c = 16;
    return ceil((a / c) * referencia + 100.0);

  // 3 variaveis, varia em relacao ao tempo
  case 3:
    c = MAX_VOLTAGE;
    a = (MIN_PERCENT - MAX_VOLTAGE) / DIS_MAX * DIS_MAX;
    if (a < 1)
    {
      return referencia * referencia * a * referencia_2 + c;
    }
    else
    {
      return referencia * referencia * a / referencia_2 + c;
    }

  // erro
  default:
    return -1;
  }
}

void ajuste(float delta)
{
  // variável que calcula o quanto uma roda deverá diminuir para ajustar o
  // carrinho
  double valor_acelera;

  // CONTROLE PID

  // float valor_acelera =
  //  mais a direita
  if (delta > 0)
  {
    valor_acelera = output_func_math(0, MIN_PERCENT, abs(delta));
    valor_acelera = min(valor_acelera, 100);
    valor_acelera = max(valor_acelera, 0);

    // time_here_right += (millis() - last_time);
    // time_here_left = 0;
    //  digitalWrite(led_amarelo_esquerda, HIGH);
    //  digitalWrite(led_azul_direita, LOW);
    acelera(valor_acelera, 100);
  }

  // mais a esquerda
  else if (delta < 0)
  {
    valor_acelera = output_func_math(0, MIN_PERCENT, (delta));
    valor_acelera = min(valor_acelera, 100);
    valor_acelera = max(valor_acelera, 0);
    // time_here_left += (millis() - last_time);
    // time_here_right = 0;
    //  digitalWrite(led_amarelo_esquerda, LOW);
    //  digitalWrite(led_azul_direita, HIGH);
    acelera(100, valor_acelera * 0.20);
  }
  else
  {
    // time_here_left = 0;
    // time_here_right = 0;
    //  digitalWrite(led_amarelo_esquerda, LOW);
    //  digitalWrite(led_azul_direita, LOW);
    acelera(100, 100);
  }
  // last_time = millis();
}

int rotacao_RPM_sensor1()
{
  // Verifica a contagem de tempo e exibe as informacoes coletadas do motor
  // if ((millis() - tempo_antes) > MINUTO)
  // { // A cada minuto

  // Verifica a variavel "sentido"
  //  if (sentido) { //Se ela for verdadeira ("true")
  //    Serial.print("Sentido: Horario");
  //    Serial.print("       |  ");
  //  } else { //Se ela for falsa ("false")
  //    Serial.print("Sentido: Anti-Horario");
  //    Serial.print("  |  ");
  //  }

  // Calcula a velocidade e exibe no monitor
  float media = (contador1_sensor1 + contador2_sensor1) / (NUMERO_CONTADORES); // Calcula a media dos contadores
  float velocidade = media / (NUMERO_DENTES * NUMERO_LEITURAS);                // Calcula a velocidade de acordo com o numero de dentes do disco
  velocidade *= 60;
  // Serial.print("Velocidade: ");
  // Serial.print(velocidade);
  // Serial.println(" RPM");

  // Zera os contadores e reinicia a contagem de tempo.
  contador1_sensor1 = 0;
  contador2_sensor1 = 0;

  return round(velocidade);
  // tempo_antes = millis();
  // }
}

int rotacao_RPM_sensor2()
{
  // Verifica a contagem de tempo e exibe as informacoes coletadas do motor
  // if ((millis() - tempo_antes) > MINUTO)
  // { // A cada minuto

  // Verifica a variavel "sentido"
  //  if (sentido) { //Se ela for verdadeira ("true")
  //    Serial.print("Sentido: Horario");
  //    Serial.print("       |  ");
  //  } else { //Se ela for falsa ("false")
  //    Serial.print("Sentido: Anti-Horario");
  //    Serial.print("  |  ");
  //  }

  // Calcula a velocidade e exibe no monitor
  float media = (contador1_sensor2 + contador2_sensor2) / (NUMERO_CONTADORES); // Calcula a media dos contadores
  float velocidade = media / (NUMERO_DENTES * NUMERO_LEITURAS);                // Calcula a velocidade de acordo com o numero de dentes do disco
  velocidade *= 60;
  // Serial.print("Velocidade: ");
  // Serial.print(velocidade);
  // Serial.println(" RPM");

  // Zera os contadores e reinicia a contagem de tempo.
  contador1_sensor2 = 0;
  contador2_sensor2 = 0;

  return round(velocidade);
  // tempo_antes = millis();
  // }
}

// posição 0 esquerda, 1 centro, 2 direita
int *livre()
{
  int *vector = new int[3];
  // 0 não está livre
  //  1 está livre
  distanciaE/10 < tamanho_pista * 1.5 ? vector[0] = 0 : vector[0] = 1;

  distanciaC/10 <= (tamanho_pista - tamanho_carrinho) / 2 ? vector[1] = 0 : vector[1] = 1;

  distanciaD/10 < tamanho_pista * 1.5 ? vector[2] = 0 : vector[2] = 1;

  return vector;
}

// melhor roda
double rpm_direta(float x)
{
  return 5.2822e-9 * pow(x, 6) -
         8.30698e-7 * pow(x, 5) -
         3.02739e-5 * pow(x, 4) +
         0.009833676 * pow(x, 3) -
         0.372287095 * pow(x, 2) +
         3.496110355 * x;
}

double rpm_esquerdo(float x)
{
  return -6.39077e-9 * pow(x, 6) +
         2.67143e-6 * pow(x, 5) -
         0.000410143 * pow(x, 4) +
         0.027383752 * pow(x, 3) -
         0.6778225 * pow(x, 2) +
         5.256018147 * x -
         3.550308947;
}

int ajuste_rpm(int velocidade_melhor_real, int velocidade_pior_real)
{

  int diferenca_real = abs(velocidade_pior_real - velocidade_melhor_real);
  int menor_diferenca = 0;

  for (int i = 0; i <= 100; i++)
  {
    if (abs(rpm_esquerdo(menor_diferenca) - velocidade_melhor_real) > abs(rpm_esquerdo(i) - velocidade_melhor_real))
    {
      menor_diferenca = i;
    }
  }

  return abs(rpm_esquerdo(menor_diferenca) - velocidade_melhor_real) < diferenca_real ? menor_diferenca : -1;
}

void frente(int *vector)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
  acelera(80, 80);
}

void virar_direita(int *vector)
{
  acelera(0,0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
  acelera(80, 80);
  delay(550);
  if (vector[1])
  {
    frente(vector);
    delay(100);
  }
}

void virar_esquerda(int *vector)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);
  acelera(80, 80);
  delay(550);
  if (vector[1])
  {
    frente(vector);
    delay(100);
  }
}

void setup()
{
  while (!Serial)
  {
  }

  Serial.begin(9600); // Comunicação Serial com o Computador
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); // definição dos pinos entradas e saidas
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); // OUTPUT = Saída
  pinMode(IN4, OUTPUT); // INPUT = Entrada

  Wire.begin();
  Wire.setClock(400000);

  attachInterrupt(digitalPinToInterrupt(PINO_CH1), contador_pulso1_sensor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_CH2), contador_pulso2_sensor1, CHANGE);

  attachPCINT(digitalPinToPCINT(PINO_CH3), contador_pulso1_sensor2, CHANGE);
  attachPCINT(digitalPinToPCINT(PINO_CH4), contador_pulso2_sensor2, CHANGE);

  pinMode(xshutPinsD, OUTPUT);
  digitalWrite(xshutPinsD, LOW);

  pinMode(xshutPinsC, OUTPUT);
  digitalWrite(xshutPinsC, LOW);

  pinMode(xshutPinsE, OUTPUT);
  digitalWrite(xshutPinsE, LOW);
  pinMode(A3, OUTPUT);
  pinMode(A2, INPUT);
  digitalWrite(A3, HIGH);

  PIDc.SetSampleTime(10);
  PIDc.SetMode(AUTOMATIC);
  PIDc.SetTunings(kpCentral, kiCentral, kdCentral);
  // if (MIN_VOLTAGE_DIR < MIN_VOLTAGE_ESQ)
  // {
  //   PIDc.SetOutputLimits(MIN_VOLTAGE_DIR, MAX_VOLTAGE);
  // }
  // else
  // {
  //   PIDc.SetOutputLimits(MIN_VOLTAGE_ESQ, MAX_VOLTAGE);
  // }
  PIDc.SetOutputLimits(70, MAX_VOLTAGE);

  pinMode(xshutPinsE, INPUT);
  delay(10);

  sensorE.setTimeout(500);
  if (!sensorE.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("E");
    while (1)
      ;
  }

  sensorE.setAddress(0x2A);

  sensorE.startContinuous(50);

  pinMode(xshutPinsD, INPUT);
  delay(10);

  sensorD.setTimeout(500);
  if (!sensorD.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("D");
    while (1)
      ;
  }

  sensorD.setAddress(0x2A + 1);

  sensorD.startContinuous(50);

  pinMode(xshutPinsC, INPUT);
  delay(10);

  sensorC.setTimeout(500);
  if (!sensorC.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("C");
    while (1)
      ;
  }

  sensorC.setAddress(0x2A + 2);

  sensorC.startContinuous(50);

  delay(2000);
  ler_sensores();
  tamanho_pista = distanciaD / 10 + distanciaE / 10 + tamanho_carrinho;
  tamanho_pista = 30;
  time = millis();
  // last_time = time;
  // acelera(90,90);
  // delay(500);
  // tamanho_pista = distanciaD + distanciaE + tamanho_carrinho;
}

void loop()
{
  ler_sensores();
  float delta = distanciaE - distanciaD;

  int delta_velocidade = velocidadeE - velocidadeD;

  int *vector = livre();
  Serial.print("vector E: ");
  Serial.println(distanciaE);
  Serial.print("vector C: ");
  Serial.println(distanciaC);
  Serial.print("vector D: ");
  Serial.println(distanciaD);
  if (distanciaD > 150){
    frente(vector);
    delay(500);
    virar_direita(vector);
    
  }else if(distanciaE < 150 && distanciaC < 100 && distanciaD < 150) {
    virar_direita(vector);
  }else if(distanciaC > 100){
      if(distanciaD > distanciaE){
      acelera(60, 80);
    }else if(distanciaE>distanciaD){
      acelera(80, 60);
    }else{
      frente(vector);
    }
  }else if(distanciaE > 150){
    virar_esquerda(vector); 
  }
  // if(vector[1])
  // {
  //   frente(vector);
  //   if (!vector[2] && !vector[0])
  //   {
  //     ajuste(delta);
  //   }

  // }
  // else
  // {
  //   if(vector[0])
  //   {
  //     virar_esquerda(vector);
  //   }
  //   else
  //   {
  //     virar_direita(vector);
  //   }
  // }

  // int vector[3] = {0, 0, 1};
  // virar_esquerda(vector);
  // while (true)
  // {
  //   acelera(0,0);
  // }

  // capturar valores das rodas
  //  while (true)
  //  {
  //    unsigned long tempo_limite = 1000 * 32;

  //   for (int i = 0; i <= 100; i += 5)
  //   {

  //     acelera(100, 100);
  //     delay(100);

  //     acelera(i,i);
  //     delay(100);

  //     unsigned long tempo_atual = millis();
  //     while (millis() - tempo_atual < tempo_limite)
  //     {
  //       /* code */

  //       ler_sensores();
  //       Serial.print("i: ");
  //       Serial.println(i);
  //       Serial.print("Velocidade Esquerda: ");
  //       Serial.println(rotacao_RPM_sensor2());
  //       Serial.print("Velocidade Direita: ");
  //       Serial.println(rotacao_RPM_sensor1());
  //     }
  //     //  delay(1000);
  //   }

  //   while (true)
  //   {
  //     acelera(0, 0);
  //   }
  // }
}
