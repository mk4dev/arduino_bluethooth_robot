/*
 * Title:        Bluetooth Remote Control

 * Orignal Author:       Karl Kangur <karl.kangur@gmail.com>
 
 * Expanded by Mohamed Hassani for Comp Robot Ibdaa for intership project
 */

#define _DISABLE_ARDUINO_TIMER0_INTERRUPT_HANDLER_

#include <wiring.c>

#include <ServoTimer2.h> 


#define LED 13

char serialData[32];
byte com = 0, error = 0, timerCounter = 0;
boolean connected;

ServoTimer2 armServo;
ServoTimer2 penceServo;




void setup()
{

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  
  pinMode(LED, OUTPUT);
  
  // the bluetooth dongle communicates at 9600 baud only
  Serial.begin(9600);
  // set up timeout timer on timer2 in case of lost connection
  // initialize counter
  TCNT0 = 0;
  // set prescaler to 256
  TCCR0B = 0x04;
  // enable overflow interrupt
  TIMSK0 |= (1 << TOIE2);
  // enable global interrupts
  asm("SEI");

  //Arm setup
  
  armServo.attach(3);
  penceServo.attach(11);
  
}

void loop()
{
  if(Serial.available() > 0)
  {
    digitalWrite(LED, HIGH);
    connected = true;
    // clear timeout
    com = timerCounter;

    Serial.readBytesUntil('\n', serialData, 31);
    switch(serialData[0])
    {
    case 0:
      Serial.println(0);
      break;
    case 'a':
      // use as a small and slow oscilloscope
      int pin;
      if(parseCommand(serialData, &pin, 1) && pin >= 0 && pin <= 7)
      {
        // stop loop by sending something to the robot
        while(!Serial.available() && connected)
        {
          Serial.println(analogRead(pin));
        }
      }
      else
      {
        Serial.println("Error while setting ADC pin");
      }
      break;
      case 'i':
        Serial.println("robot_ibdaa");
        break;
     case 't':
        Serial.println("connected");
        break;
    case 's':
      // set left and right motor speeds
      int speed[2];
      if(parseCommand(serialData, speed, 2))
      {
        setSpeed(speed[0], speed[1]);
        Serial.println("New speed set");
      }
      else
      {
        Serial.println("Error while setting new speed");
      }
      break;
    case 'c':
      // set arm control
      int control[2];
      if(parseCommand(serialData, control, 2))
      {
        setArmClt(control[0], control[1]);
        Serial.println("New arm control set");
      }
      else
      {
        Serial.println("Error while setting new arm control");
      }
      break;
    
    case 'r':
      // quickly stop
      reset();
      Serial.println("Robot reset");
      break;
    default:
      // inform user of non existing command
      Serial.println("Command not recognised");
    }

    // clear serialData array
    memset(serialData, 0, sizeof(serialData));
  }
}

void reset()
{
  connected = false;
  setSpeed(0, 0);
}

/**
 * This function makes ints out of the received serial data, the 2 first
 * characters are not counted as they consist of the command character and
 * a comma separating the first variable.
 *
 * @params command The whole serial data received as an address
 * @params returnValues The array where the return values go as an address
 * @params returnNumber The number of values to set inside the returnValues variable
 */
boolean parseCommand(char* command, int* returnValues, byte returnNumber)
{
  // parsing state machine
  byte i = 1, j = 0, sign = 0, ch = 0, number;
  int temp = 0;
  while(i++)
  {
    switch(*(command + i))
    {
    case '\0':
    case ',':
      // set return value
      if(ch != 0)
      {
        returnValues[j++] = sign?-temp:temp;
        sign = 0;
        temp = 0;
        ch = 0;
      }
      else
      {
        return false;
      }
      break;
    case '-':
      sign = 1;
      break;
    default:
      // convert string to int
      number = *(command + i) - '0';
      if(number < 0 || number > 9)
      {
        return false;
      }
      temp = temp * 10 + number;
      ch++;
    }

    // enough return values have been set
    if(j == returnNumber)
    {
      return true;
    }
    // end of command reached
    else if(*(command + i) == '\0')
    {
      return false;
    }
  }
}

ISR(TIMER0_OVF_vect)
{
  // 8e6Hz / 256 / 256 / 122 = 1Hz
  if(timerCounter++ < 122)
  {
    return;
  }

  // verify communication status
  if(com == 0)
  {
    // no data has been passed since last time
    // interpret as communication failure
    digitalWrite(LED, LOW);
    reset();
  }
  com = 0;
  timerCounter = 0;
}

// set motor speed for both wheels, uses timer1
void setSpeed(int left, int right)
{
  // PD4-7 as output
  DDRD |= 0xf0;
  // set Waveform Generation Mode
  TCCR1A = 0x03; // (1<<WGM11)|(1<<WGM10): PWM, phase correct, 10bits
  TCCR1B = 0x01; // (1<<CS10): set prescaling of 1 (8MHz/1 = 8MHz)


  
  if(left == 0)
  {
    // normal port operation, OC1A disconnected
    TCCR1A &= ~(3 << COM1A0);
    PORTD &= ~(1 << PD4);
    PORTD &= ~(1 << PD5);
  }
  else
  {
    // clear OC1A on Compare Match (set output to low level)
    TCCR1A |= (1 << COM1A1);

    // set left motor direction (PD7)
    if(left < 0)
    {
      left = -left;
      PORTD |=  (1 << PD4);
      PORTD &= ~(1 << PD5);
    }
    else
    {
      PORTD |=  (1 << PD5);
      PORTD &= ~(1 << PD4);
    }

    // limit to maximum speed
    if(left > 100)
    {
      left = 716;
    }
    else
    {
      left = (long) map(left, 0, 100, 511, 716); // 1023*left/100;	
    }
  }

  if(right == 0)
  {
    // normal port operation, OC1B disconnected
    TCCR1A &= ~(3 << COM1B0);
    PORTD &= ~(1 << PD6);
    PORTD &= ~(1 << PD7);
  }
  else
  {
    // clear OC1B on Compare Match (set output to low level)
    TCCR1A |= (1 << COM1B1);

    // set right motor direction (PD6)
    if(right < 0)
    {
      right = -right;
      PORTD |=  (1 << PD6);
      PORTD &= ~(1 << PD7);
    }
    else
    {
      PORTD |=  (1 << PD7);
      PORTD &= ~(1 << PD6);
    }

    // limit to maximum speed
    if(right > 100)
    {
      right = 661;
    }
    else
    {
      right = map(right, 0, 100, 472, 661) ;//(long) 950*right/100;
    }
  }

  // first write high byte (important, see datasheet)
  OCR1AH = left >> 8;
  OCR1AL = left & 0xff;
  OCR1BH = right >> 8;
  OCR1BL = right & 0xff;

 

  
  
}


// set arm movements
void setArmClt(int armBody, int armPince){

  int armDeg = map(armBody, 0, 100, 0, 3000);
  int armPenceDeg = map(armPince, 0, 100, 0, 3000);
  armServo.write(armDeg);
  penceServo.write(armPenceDeg);
  
  
  
}

