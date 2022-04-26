//------------------LAB INTEGRADOR LAVRAS MG-------------------------------
//Carrega a biblioteca do sensor ultrassonico
#include <Ultrasonic.h>

//Define the pins to configuration
#define pin_trigger    2      // the number of the pin of trigger
#define pin_echo       3      // the number of the pin of echo
#define pin_pot       A14     // the number of the pin of input to potentiometer
#define pin_PWM        7      // the number of the pin of output to driver
//#define pin_controler  8      // the number of the pin to inicialize controller


  
//Inicializa o sensor nos pinos definidos acima
Ultrasonic ultrasonic(pin_trigger, pin_echo);

double valor_pwm;
double valor_pwm_pc;
int t_delay = 100; //Delay to syncronism
double duty = 0; //duty-cycle MV(Manipulated Variable)
float potValue;


void setup()//Here your code execute once
{
  // set the digital pin as output for PWM:
  pinMode(pin_PWM, OUTPUT);

  //Setpoint
  pinMode(pin_pot, INPUT);


  Serial.begin(9600);// start serial communication with 9600 baudrate
  //It is important to see if the HC-05 Module is configured at this rate , if is not, change it configuration in AT mode,it has files on the project folder
  //with an detailed explanation
  delay(500);

  inicio();

}


void loop()//Here the codes runs continously
{
  //--------------Start PWM-------------------------
  potValue = analogRead(pin_pot);
  valor_pwm = map(analogRead(pin_pot), 0,1023, 0,256);
  valor_pwm_pc = map(valor_pwm, 0,256, 0,100);
  

  //---------------End PWM--------------------------

  //-----------Impressão Serial---------------------
  Serial.print("DATA,TIME,"); // inicia impressão de dados, sempre iniciando
  Serial.print(potValue); // valor setpoint
  Serial.print(",");
  Serial.print(distancia());
  Serial.print(",");
  Serial.println(valor_pwm_pc);
  //------------------End------------------  
  //if(distancia() > 150){ // Condição de segurança para o nível não ultrapassar o máximo
  //    valor_pwm = 0;
  //  }
  analogWrite(pin_PWM, valor_pwm);
    
  delay(t_delay);

   /*      
      Serial.print(distancia());//distancia PV (Process Variable)
      Serial.print("\t");
      Serial.print(sensorValue);
      Serial.print("\t");
      Serial.print(valor_pwm_pc);
      Serial.print("\n");
      delay(t_delay);
  */
}

float distancia(){
  //Le as informacoes do sensor
  float zero = 25.5;
  float cmMsec;
  float altura = 0;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  for(int i = 0; i<50; i++){
    altura = altura + cmMsec;
  }
  altura = abs((zero - (altura/50))*10);
  return altura;
}

void inicio(){
  int LABEL = 1;
  Serial.println("CLEARDATA"); // RESETAR COMUNICAÇÃO SERIAL
  Serial.println("LABEL,Hora,POTENCIOMETRO,DISTANCIA [mm],PWM"); // NOMEAR COLUNAS
}
