//------------------ LAB INTEGRADOR LAVRAS MG 2022 -------------------------------

class PID{
  public:
    
    double error;
    double sample;
    double lastSample;
    double kP, kI, kD;      
    double P, I, D;
    double pid;
    
    double setPoint;
    long lastProcess;
    
    PID(double _kP, double _kI, double _kD){
      kP = _kP;
      kI = _kI;
      kD = _kD;
    }
    
    void addNewSample(double _sample){
      sample = _sample;
    }
    
    void setSetPoint(double _setPoint){
      setPoint = _setPoint;
    }
    
    double process(){
      // Implementação P ID
      error = setPoint - sample;
      float deltaTime = (millis() - lastProcess) / 1000.0;
      lastProcess = millis();
      
      //P
      P = error * kP;
      
      //I
      I = I + (error * kI) * deltaTime;
      
      //D
      D = (lastSample - sample) * kD / deltaTime;
      lastSample = sample;
      
      // Soma tudo
      pid = P + I + D;
      
      return pid;
    }
  };
  
//Carrega a biblioteca do sensor ultrassonico
#include <Ultrasonic.h>

//Define the pins to configuration
#define pin_trigger    2      // the number of the pin of trigger white
#define pin_echo       3      // the number of the pin of echo
#define pin_pot       A14     // the number of the pin of input to potentiometer
#define pin_PWM        7      // the number of the pin of output to driver

int altura_desejada = 0;
int setpoint = 0;
double valor_pwm_pc;
float h;
int duty;
int t_delay = 100; //Delay to syncronism


//Inicializa o sensor nos pinos definidos acima
Ultrasonic ultrasonic(pin_trigger, pin_echo);

// tf(h1) 0.2359, 0.0014333, 0
// tf(h2) 0.97414, 0.00052098, 0
// tf(h3) 0.79765, 0.017427, 0

PID meuPid(0.79765, 0.017427, 0); //(double _kP, double _kI, double _kD)

void setup()//Here your code execute once
{
  // set the digital pin as output for PWM:
  pinMode(pin_PWM, OUTPUT);

  Serial.begin(9600);// start serial communication with 9600 baudrate
  //It is important to see if the HC-05 Module is configured at this rate , if is not, change it configuration in AT mode,it has files on the project folder
  //with an detailed explanation
  delay(500);
  Serial.flush();
}

void loop() {
  if (Serial.available())   // Verifica se a porta serial está aberta
  {
    altura_desejada = Serial.parseInt();  // A altura desejada recebe o valor da porta serial
  }
  if(altura_desejada != 0){
    setpoint = altura_desejada;
  }

    // Manda pro objeto PID!
  h = distancia();
  meuPid.addNewSample(h); // Leitura da altura de nivel
  meuPid.setSetPoint(setpoint); // Informa o setpoit para o controlador

  float controlePwm;
  // Converte para controle
  controlePwm = (meuPid.process() + 0); // Controle aplicado
  // Saída do controle
  valor_pwm_pc = map(controlePwm, 0, 155, 0, 255);
  if (valor_pwm_pc >= 0){
    analogWrite(pin_PWM, valor_pwm_pc);
    duty = map(valor_pwm_pc, 0, 255,0, 100);
  }
  

  //Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print("\t");
  //Serial.print("Nível: ");
  Serial.print(h); // altura
  Serial.print("\t");
  //Serial.print("Duty cycle: ");
  Serial.println(duty);
 // Serial.println("%");
  //delay(t_delay);
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
 
