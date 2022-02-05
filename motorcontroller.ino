/////////////////////////////////////////////////////////////////////////////
//Arduino Motor Shield Driver. 
//Code developed by Gustavo Viera Lopez
//Code developed by Silvio Delgado
//Ask gvieralopez@gmail.com for the hardware schematics


//////////////////////////////////////////////////////////////////////////
//Defining Arduino's Pins
/////////////////////////////////////////////////////////////////////////////

#define IN1                 7
#define IN2                 6
#define IN3                 12
#define IN4                 13
#define PWM1                9
#define PWM2                10
#define STBY                8

#define Encoder1            2
#define Encoder2            4
#define SetpointThreshold   0.01

/////////////////////////////////////////////////////////////////////////////
//Serial port protocol commands
/////////////////////////////////////////////////////////////////////////////
#define COMMAND_SETPIDPARAM             0xA6
#define COMMAND_SETPOINT                0xA7
#define COMMAND_GETSTATE                0xA8
#define COMMAND_GETSTATE_RESPONSE       0xA9
#define COMMAND_ENCODER_RESET           0xAA
#define COMMAND_START_SAMPLING_SPEEDS   0xAB
#define COMMAND_STOP_SAMPLING_SPEEDS    0xAC
/////////////////////////////////////////////////////////////////////////////
//Defining useful constants
/////////////////////////////////////////////////////////////////////////////
#define CLOCKWISE           0
#define COUNTERCLOCKWISE    1
#define REVOLUTION_STEPS    270.9
#define SAMPLE_TIME         0.025
/////////////////////////////////////////////////////////////////////////////
//Defining variables
/////////////////////////////////////////////////////////////////////////////
volatile long pulses1;
volatile long pulses2;
volatile long lastpulses1;
volatile long lastpulses2;
volatile char sense1;
volatile char sense2;
volatile float pulses_factor;
char buffer[4];
uint8_t *ptr;
char buff[10];
byte reload = 0x9C; 


/////////////////////////////////////////////////////////////////////////////
//Defining PID variables
/////////////////////////////////////////////////////////////////////////////
volatile float constant_kc, constant_ki, constant_kd;
volatile float e1k1,e1k2,u1k1;
volatile float e2k1,e2k2,u2k1;
volatile float setpoint1,setpoint2;
volatile float speed1, speed2;
volatile char battery_status;
/////////////////////////////////////////////////////////////////////////////
//PID tunning variables
/////////////////////////////////////////////////////////////////////////////
volatile char samplingSpeeds; // Indicates if the sampling function is active
volatile char samplingFlag; // Raises when is time to send a sample
/////////////////////////////////////////////////////////////////////////////
//Defining funtions
/////////////////////////////////////////////////////////////////////////////



void catchPulses1()
{
    if(sense1==CLOCKWISE)
    {
        pulses1--;
    }
    else
    {
        pulses1++;
    }
}
/////////////////////////////////////////////////////////////////////////////
void catchPulses2()
{
    if(sense2==CLOCKWISE)
    {
        pulses2--;
    }
    else
    {
        pulses2++;
    }
}
/////////////////////////////////////////////////////////////////////////////

void setup() 
{
    Serial.begin(9600); 
    Serial.println("Setup");

    InitMotors();
    //InitPWM();
    InitSampling();
    TCCR0B = 0; 
    OCR2A = reload;
    TCCR2A = 1<<WGM21;
    TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);
    TIMSK2 = (1<<OCIE2A);

    interrupts();
    setpoint1 = 0.5f;
    setpoint2 = -0.5f;

} 
///////////////////////////////////////////////////////////////////////////// 
void loop() 
{
    if (samplingFlag)
    {       
        ptr = ((uint8_t*)&speed1);
        Serial.write(ptr,4);
        ptr = ((uint8_t*)&speed2);  
        Serial.write(ptr,4);
        samplingFlag = 0;
    }
    if(Serial.available() > 0)
    {
        char x = Serial.read();

        switch (x)
        {
        case COMMAND_SETPIDPARAM:
            while(Serial.available() < 4);
            Serial.readBytes(buffer, 4);
            constant_kc = *(float*)buffer;

            while(Serial.available() < 4);
            Serial.readBytes(buffer, 4);
            constant_ki = *(float*)buffer;

            while(Serial.available() < 4);
            Serial.readBytes(buffer, 4);
            constant_kd = *(float*)buffer;
            break;
        case COMMAND_SETPOINT:
            // while(Serial.available() < 4);
            // Serial.readBytes(buffer, 4);
            // setpoint1 = *(float*)buffer;

            // while(Serial.available() < 4);
            // Serial.readBytes(buffer, 4);
            // setpoint2 = *(float*)buffer;
            break;
        case COMMAND_GETSTATE:
            if(samplingSpeeds == 0)
            {
                Serial.write(COMMAND_GETSTATE_RESPONSE);
                ptr = (uint8_t*)&pulses1;
                Serial.write(ptr,4);
                ptr = (uint8_t*)&pulses2;
                Serial.write(ptr,4);
                Serial.write(battery_status);
            }
            break;
        case COMMAND_ENCODER_RESET:
            pulses1 = 0;
            pulses2 = 0;
            lastpulses1 = 0;
            lastpulses2 = 0;
            break;
        case COMMAND_START_SAMPLING_SPEEDS:
            samplingSpeeds = 1;
            break;
        case COMMAND_STOP_SAMPLING_SPEEDS:
            samplingSpeeds = 0;
            break;
        default:
            break;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////
void driveMotor(char motor, float speed)
{
    char direction = ((speed >= 0) ?  COUNTERCLOCKWISE : CLOCKWISE);
    speed = fabs(speed);

    if (motor==1)
    {
        if(!speed)
        {
            digitalWrite(IN1,LOW);
            digitalWrite(IN2,LOW);
        }
        else
        {
            if(direction==CLOCKWISE)
            {
                digitalWrite(IN1,HIGH); 
                digitalWrite(IN2,LOW); 
            }
            else
            {
                digitalWrite(IN1,LOW); 
                digitalWrite(IN2,HIGH);           
            }
            analogWrite(PWM1, speed); 
            //setPWM(motor, (char)speed); 
        }
    }
    else
    {
        if(!speed)
        {
            digitalWrite(IN3,LOW); 
            digitalWrite(IN4,LOW); 
        }
        else
        {
            if(direction==CLOCKWISE)
            {
                digitalWrite(IN3,LOW); 
                digitalWrite(IN4,HIGH); 
            }
            else
            {
                digitalWrite(IN3,HIGH);
                digitalWrite(IN4,LOW);
            }
            
            analogWrite(PWM2, speed); 

            //setPWM(motor, (char)speed); 
        }
    }   
}
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
void InitMotors()
{

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT); 
    pinMode(STBY, OUTPUT);  
    pinMode(Encoder1, INPUT); 
    pinMode(Encoder2, INPUT);

    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
    digitalWrite(STBY, 1);
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);    
    
    digitalWrite(STBY, 1);

    attachInterrupt(0,catchPulses1,FALLING);
    attachInterrupt(1,catchPulses2,FALLING);    

}
/////////////////////////////////////////////////////////////////////////////
void InitSampling()
{
    constant_kc = 10.0f; // parameters for PID
    constant_ki = 5.0f;
    constant_kd = 5.0f;

    e1k1 = 0.0f;
    e1k2 = 0.0f;
    u1k1 = 0.0f;
    
    e2k1 = 0.0f;
    e2k2 = 0.0f;
    u2k1 = 0.0f;
    
    setpoint1 = 0.1f;
    setpoint2 = -0.1f;
    speed1 = 0.0f;
    speed2 = 0.0f;

    samplingSpeeds = 0; // No speed sampling by default, it is only for Tunning PID system
    samplingFlag = 0;

}

ISR(TIMER2_COMPA_vect)
{
    OCR2A = reload;
    long temp_pulses1 = pulses1;
    long temp_pulses2 = pulses2;
    long delta_pulses1 = temp_pulses1 - lastpulses1;
    long delta_pulses2 = temp_pulses2 - lastpulses2;

    lastpulses1 = temp_pulses1;
    lastpulses2 = temp_pulses2;

    //float speed1 = delta_pulses1 * 2.0f * PI / REVOLUTION_STEPS / SAMPLE_TIME;
    //float speed2 = delta_pulses2 * 2.0f * PI / REVOLUTION_STEPS / SAMPLE_TIME;

    speed1 = delta_pulses1 * pulses_factor;
    speed2 = delta_pulses2 * pulses_factor;

    if (samplingSpeeds == 1)
    {
        samplingFlag = 1;
    }

    float e1k = setpoint1 - speed1;
    float e2k = setpoint2 - speed2;


    float u1k;
    if (setpoint1 > SetpointThreshold || setpoint1 < -SetpointThreshold )
    {
        u1k = constant_kc * (e1k - e1k1) + constant_ki * e1k + u1k1 + constant_kd * (
                    e1k - 2 * e1k1 + e1k2);
        if (u1k > 255.0f)
        {
            u1k = 255.0f;
        }

        if (u1k < -255.0f)
        {
            u1k = -255.0f;
        }
    }
    else    
    {
        u1k = 0;
    }

    float u2k;
    if (setpoint2 > SetpointThreshold || setpoint2 < -SetpointThreshold)
    {
        u2k = constant_kc * (e2k - e2k1) + constant_ki * e2k + u2k1 + constant_kd * (
                    e2k - 2 * e2k1 + e2k2);
        if (u2k > 255.0f)
        {
            u2k = 255.0f;
        }

        if(u2k < -255.0f)
        {
            u2k = -255.0f;
        }
    }
    else 
    {
        u2k = 0;
    }
    driveMotor(1,u1k);
    driveMotor(2,u2k);

    e1k2 = e1k1;
    e1k1 = e1k;
    u1k1 = u1k;

    e2k2 = e2k1;
    e2k1 = e2k;
    u2k1 = u2k;
}
