using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.NetduinoPlus;
using System.Diagnostics;
using Toolbox.NETMF.Hardware;

namespace RobiMow
{
    // ---------- PID -----------------------------------
     struct Pid_t
    {
        public Pid_t(double ta, Int16 w, Int16 x, Int16 esum, Int16 eold, Int16 y, Int16 y_min, Int16 y_max, Int16 max_pwm, double kp, double ki, double kd)
        {
            this.Ta = ta;
            this.w = w;
            this.x = x;
            this.esum = esum;
            this.eold = eold;
            this.y = y;
            this.y_min = y_min;
            this.y_max = y_max;
            this.max_pwm = max_pwm;
            this.Kp = kp;
            this.Ki = ki;
            this.Kd = kd;
        }
        private readonly double Ta;
        private readonly Int16 w;
        private readonly Int16 x;
        private readonly Int16 esum;
        private readonly Int16 eold;
        private readonly Int16 y;
        private readonly Int16 y_min;
        private readonly Int16 y_max;
        private readonly Int16 max_pwm;  // 1023
        private readonly double Kp;//8.0  // Verstaerkungsfaktor Regelabweichung 		
        private readonly double Ki;//0.0 	// Faktor fuer Integral Regelabweichung	
        private readonly double Kd;//0.01 // Faktor fuer Ableitung Regelabweichung		
    };

    public class Program
    {
                // ------ configuration --------------------------------------------------
        // ------ features 
        const bool DriveBiDir = false;      // drive bidirectional (1) (forward/backward) or make turns only (0)? 
        const bool UseBumper = true;       // has bumpers? 
        const bool UseCompass = false;      // use compass sensor?
        const bool UseAccel = false;        // use acceleration sensor?
        const bool UseSonar = false;        // use ultra sonic sensor?
        const bool UseTilt = false;         // has tilt sensor? (required for TC-G158 board)
        const bool UseButton = false;       // has digital ON/OFF button?
        const bool UsePerimeter = true;    // use perimeter?
        // ------ pins
       // Microsoft.SPOT.Hardware.Cpu.PWMChannel pinMotorLeftPWM = Cpu.PWMChannel.PWM_1;          // left motor pins
       // Microsoft.SPOT.Hardware.Cpu.Pin pinMotorLeftDir = Cpu.Pin.GPIO_Pin1;
       // Microsoft.SPOT.Hardware.Cpu.AnalogChannel pinMotorLeftSense = Cpu.AnalogChannel.ANALOG_1;
       // Microsoft.SPOT.Hardware.Cpu.PWMChannel pinMotorRightPWM = Cpu.PWMChannel.PWM_3;      // right motor pins
       // Microsoft.SPOT.Hardware.Cpu.Pin pinMotorRightDir = Cpu.Pin.GPIO_Pin3;
       // Microsoft.SPOT.Hardware.Cpu.AnalogChannel pinMotorRightSense = Cpu.AnalogChannel.ANALOG_2;
        Microsoft.SPOT.Hardware.Cpu.Pin pinMotorMow = Cpu.Pin.GPIO_Pin3;              // mower motor pins
        Microsoft.SPOT.Hardware.Cpu.AnalogChannel pinMotorMowSense = Cpu.AnalogChannel.ANALOG_3;
        Microsoft.SPOT.Hardware.Cpu.Pin pinMotorMowRpm          = Cpu.Pin.GPIO_Pin11;    
        Microsoft.SPOT.Hardware.Cpu.Pin pinBumperLeft           = Cpu.Pin.GPIO_Pin12;           // bumper pins
        Microsoft.SPOT.Hardware.Cpu.Pin pinBumperRight          = Cpu.Pin.GPIO_Pin13;
        Microsoft.SPOT.Hardware.Cpu.Pin pinSonarCenterTrigger   = Cpu.Pin.GPIO_Pin14;         // ultrasonic sensor pins
        Microsoft.SPOT.Hardware.Cpu.Pin pinSonarCenterEcho      = Cpu.Pin.GPIO_Pin15;
        //#define pinSonarRightTrigger 30         
        //#define pinSonarRightEcho 32
        //#define pinSonarLeftTrigger 34         
        //#define pinSonarLeftEcho 36
        Microsoft.SPOT.Hardware.Cpu.AnalogChannel pinPerimeterRight = Cpu.AnalogChannel.ANALOG_4;// perimeter
        Microsoft.SPOT.Hardware.Cpu.AnalogChannel pinPerimeterLeft = Cpu.AnalogChannel.ANALOG_5;
        //#define pinPerimeterMuxZ A6           // perimeter mux Z (only TC-G158 board)
       // #define pinLED 13                  // LED
       // #define pinBuzzer 53               // Buzzer
       // #define pinTilt 35                 // Tilt sensor (required for TC-G158 board)
       // #define pinButton 51               // digital ON/OFF button
        Microsoft.SPOT.Hardware.Cpu.AnalogChannel PinBatteryVoltage = Cpu.AnalogChannel.ANALOG_6; // battery voltage sensor
        Microsoft.SPOT.Hardware.Cpu.AnalogChannel PinChargeCurrent =  Cpu.AnalogChannel.ANALOG_7; // charge current sensor
        //#define pinMuxS0 28              // mux S0 (only TC-G158 board)
        //#define pinMuxS1 26              // mux S1 (only TC-G158 board)
        //#define pinMuxZ A7                 // mux Z (only TC-G158 board)

        //Set Motordriver
        HBridge MotorDriver1 = new HBridge(PWMChannels.PWM_PIN_D6, Pins.GPIO_PIN_D7, PWMChannels.PWM_PIN_D3, Pins.GPIO_PIN_D4);


        // compass (I2C):   SCL A5 ,  SDA A4
        // ------- other
        const int  BAUDRATE = 19200;            // serial output baud rate
        const int  MAX_MOTOR_SPEED = 255;       // motor wheel max PWM  (8-bit PWM=255, 10-bit PWM=1023)
        const int  MAX_MOTOR_CURRENT = 30;      // motor wheel max current (8-bit ADC:0..255 , 10-bit ADC:0..1023)
        const int  MAX_MOW_SPEED   = 255;       // motor mower max PWM
        const int  MAX_MOW_CURRENT = 60;        // motor mower max current
        const int  motorRightSenseZero = 511;   // motor right sense zero point 
        const int  motorLeftSenseZero = 511;    // motor left sense zero point
        const int  motorMowSenseZero = 511;     // motor mower sense zero point
        const int  chargeSenseZero = 511 ;      // charge current sense zero point
        const double BATTERY_FAC = 4.7;       // battery conversion factor
        const int  MAX_ECHO_TIME = 5000;        // ultrasonic sensor max echo time
        const int  MAX_PERIMETER = 150;         // perimeter threshold 
        // ------ configuration end -------------------------------------------

        static State _stateNext = State.STATE_OFF;
        static State _stateCurr = State.STATE_OFF; 
        static State _stateLast = State.STATE_OFF;
        private static long _stateStartTime;
        private static long _stateEndTime;
        static bool _driveHome = false;
        bool _motorMowEnable = false;

        // --------- motor state ----------------------------
        // motor speed (-255..0 backward, 0..255 forward)
       static int _motorLeftSpeed = 0; // set speed
       static int _motorRightSpeed = 0;
       static int _motorMowSpeed = 0;
       static double _motorLeftPWM = 0; // current speed
       static double _motorRightPWM = 0;
       static double _motorMowPWM = 0;
        // motor current
       static double _motorLeftSense = 0;
       static double _motorRightSense = 0;
       static double _motorMowSense = 0;
        // motor current counter
       static int _motorLeftSenseCounter = 0;
       static int _motorRightSenseCounter = 0;
       static int _motorMowSenseCounter = 0;
        // --------- bumper state ---------------------------
        static int _bumperLeftCounter = 0;
        static bool _bumperLeft;
        static int _bumperRightCounter = 0;
        static bool _bumperRight;
        // --------- compass state --------------------------
        // acceleration sensor data
        double _accX,_accY,_accZ;
        double _accPitch;
        double _accRoll;
        // gyro sensor data
        double _gyroX,_gyroY,_gyroZ;
        // compass sensor data
        double _comX,_comY,_comZ;
        int _accelCounter = 0;
        // compass heading (degree)
        double _comHeading;
        // battery voltage
        int _batVoltage;
        // charge current
        int _chgCurrent;
        // perimeter state
        static int _perimeterLeft = 0;
        static int _perimeterRight = 0;
        bool _perimeterActive = false;
        int _perimeterCounter = 0;
        // --------- other ----------------------------------
        static int _loopsPerSec = 0;
        byte _buttonCounter = 0;
        byte _ledState = 0;
        int _verbose = 0;  // verbose debug output mode
        static long _nextTimeInfo = 0;
        long _nextTimeMotorSense = 0;
        long _nextTimeCompass = 0;
        long _nextTimeBumper = 0;
        long _nextTimeSonar = 0;
        long _nextTimeBattery = 0;
        long _nextTimePerimeter = 0;
        long _lastMotorControlTime = 0;
        static Direction _rollDir;
        int _driveHeading = 0;
        int _sonarDistCenter;
        int _sonarDistRight;
        int _sonarDistLeft;
        int _sonarDistCounter;
        long _nextTimeButton = 0;

        static Pid_t pidMotor = new Pid_t( 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0 );
        
        public static void Main()
        {

            // write your code here
            Stopwatch stopWatch = Stopwatch.StartNew();
            while(true){
              ReadSerial(); 
              ReadSensors();
      
              if (stopWatch.ElapsedMilliseconds >= _nextTimeInfo) {        
                _nextTimeInfo = stopWatch.ElapsedMilliseconds + 1000;    
               // ledState = ~ledState;    
               // if (ledState) digitalWrite(pinLED, HIGH);
               //   else digitalWrite(pinLED, LOW);        
                PrintInfo();        
                //if (useAccel) checkAccel();
                _loopsPerSec = 0;
              }
      
              // state machine - things to do *PERMANENTLY* for current state
              switch (_stateCurr) {
                case State.STATE_ERROR:
                  // fatal-error
                  for (int i=0; i < 2; i++){
                    tone(pinBuzzer, 1200);
                    Thread.Sleep(50);
                    noTone(pinBuzzer);
                    Thread.Sleep(50);
                  }
                    Thread.Sleep(100);            
                  break;
                case State.STATE_CHARGE:
                  // waiting until charging completed
                  break;
                case State.STATE_OFF:
                  // robot is turned off
                  break;
                case State.STATE_FORWARD:
                  // driving forward      
                  CheckCurrent();            
                  CheckBumpers();
                  CheckSonar();             
                  CheckPerimeter();
                  break;
                case State.STATE_ROLL:
                  // making a roll (left/right)            
                  if (stopWatch.ElapsedMilliseconds >= _stateEndTime) {
                    if (_driveHome) 
                        setNextState(State.STATE_PERI_FIND, 0);
                      else 
                        setNextState(State.STATE_FORWARD,0);				
                  }
                  break;
                case State.STATE_REVERSE:
                  // driving reverse      
                  if (DriveBiDir){              
                    CheckCurrent();            
                    CheckBumpers();
                    CheckSonar();                     
                  } else {
                    if (stopWatch.ElapsedMilliseconds >= _stateEndTime) 
                        setNextState(State.STATE_ROLL, _rollDir);				         
                  }
                  break;
                case State.STATE_PERI_FIND:
                  // find perimeter
                  _driveHome = true;
                  //checkCurrent();            
                  //checkBumpers();
                  //checkSonar();                   
                  CheckPerimeter();
                  break;
                case State.STATE_PERI_TRACK:
                  // track perimeter
                  _driveHome = true;
                  //checkCurrent();                  
                  //checkBumpers();
                  //checkSonar();                   
                  break;
                case State.STATE_CIRCLE:
                  // driving circles
                  break;
              } // end switch  
  
              long stateTime = stopWatch.ElapsedMilliseconds - _stateStartTime;
              if ((_stateCurr == State.STATE_FORWARD) && (stateTime > 60000)){ 
                // timeout 
                if (_rollDir == Direction.RIGHT) 
                    setNextState(State.STATE_REVERSE, Direction.LEFT); // toggle roll dir
                   else 
                    setNextState(State.STATE_REVERSE, Direction.RIGHT);
              }

              if (UseButton) 
                  CheckButton();  
  
              if (_stateNext != _stateCurr){    
                // state has changed    
                _stateStartTime = stopWatch.ElapsedMilliseconds;
                _stateLast = _stateCurr;
                _stateCurr = _stateNext;    
                PrintInfo();    
              } else {
                // still same state
              }
              MotorControl();
    
              _bumperRight = false;
              _bumperLeft = false;     
  
              Thread.Sleep(5);                              
              _loopsPerSec++;
            }

        }

        private static void PrintInfo()
        {
            throw new NotImplementedException();
        }

        private static void ReadSensors()
        {
            throw new NotImplementedException();
        }

        private static void ReadSerial()
        {
            throw new NotImplementedException();
        }

        // calculate compass heading based on compass and pitch/roll
        void calcHeading()
        {
            double xh = _comX * System.Math.Cos(_accPitch) + _comZ * System.Math.Sin(_accPitch);
            double yh = _comX * System.Math.Sin(_accRoll) * System.Math.Sin(_accPitch) + _comY * System.Math.Cos(_accRoll) - _comZ * System.Math.Sin(_accRoll) * System.Math.Cos(_accPitch);
            double zh = -_comX * System.Math.Cos(_accRoll) * System.Math.Sin(_accPitch) + _comY * System.Math.Sin(_accRoll) + _comZ * System.Math.Cos(_accRoll) * System.Math.Cos(_accPitch);
            double newheading = 180.0 * System.Math.Atan2(yh, xh) / System.Math.PI;
            if (newheading < 0) 
                newheading += 360.0;
            //double newheading = (180*atan2(Yc, Xc)/PI);  // assume pitch, roll are 0    
            if (newheading < 0) 
                newheading += 360.0;
            if ((_comHeading > 270) && (newheading < 90)) 
                newheading += 360;
            else if ((_comHeading < 90) && (newheading > 270)) 
                newheading -= 360;
            _comHeading = (15.0 * _comHeading + newheading) / 16.0;
            if (_comHeading < 0) 
                _comHeading += 360.0;
            if (_comHeading > 360) 
                _comHeading -= 360.0;
        }

      static  void checkBumpers()
        {
            if (DriveBiDir)
            {
                // bidir
                if (_bumperLeft || _bumperRight)
                {
                    if (_stateCurr == State.STATE_FORWARD) 
                        setNextState(State.STATE_REVERSE, 0);
                    else 
                        setNextState(State.STATE_FORWARD, 0);
                }
            }
            else
            {
                // non-bidir
                if (_bumperLeft || _bumperRight)
                {
                    if (_bumperLeft)
                    {
                        setNextState(State.STATE_REVERSE, Direction.RIGHT);
                    }
                    else
                    {
                        setNextState(State.STATE_REVERSE, Direction.LEFT);
                    }
                }
            }
        }
        // called *ONCE* upon a before setting to a *NEW* state
        static void setNextState(State stateNew, Direction dir){
            Stopwatch stopwatch = Stopwatch.StartNew();
           long stateTime = stopwatch.ElapsedMilliseconds - _stateStartTime;
          if (stateNew == _stateCurr) return;
          _stateNext = stateNew;
          if (DriveBiDir){
            // bidir
            if ((stateNew == State.STATE_FORWARD) || (stateNew == State.STATE_REVERSE)){      
              _driveHeading += (new Random().Next() % 90 - 45);            
              int diff;
              if (stateTime / 1000 < 10){
                diff = MAX_MOTOR_SPEED/2;
              } else {
                  diff = new Random().Next() % (MAX_MOTOR_SPEED / 5);
              }
              if (new Random().Next() % 100 > 50)
              {
                _motorLeftSpeed  =  MAX_MOTOR_SPEED -diff;
                _motorRightSpeed =  MAX_MOTOR_SPEED;
              } else {
                _motorLeftSpeed =  MAX_MOTOR_SPEED;
                _motorRightSpeed = MAX_MOTOR_SPEED -diff;        
              }      
              if (stateNew == State.STATE_REVERSE) {
                _motorLeftSpeed *= -1;
                _motorRightSpeed *= -1;
              }      
            }    
          } else {
            // non-bidir    
            _rollDir = dir;
            if (stateNew == State.STATE_FORWARD){
              _motorLeftSpeed = _motorRightSpeed = MAX_MOTOR_SPEED;
            } else if (stateNew == State.STATE_REVERSE){
              _motorLeftSpeed = _motorRightSpeed = -MAX_MOTOR_SPEED;                    
              _stateEndTime = stopwatch.ElapsedMilliseconds + 2500;                     
            } 
            else if (stateNew == State.STATE_ROLL){
              _rollDir = dir;
              /*if (dir == LEFT){
                driveHeading -= rand() % 180;        
              } else {
                driveHeading += rand() % 180;          
              } */     
              _stateEndTime = stopwatch.ElapsedMilliseconds + new Random().Next() % 2000 + 2000;               
              if (dir == Direction.RIGHT){
	        _motorLeftSpeed = MAX_MOTOR_SPEED;
	        _motorRightSpeed = -_motorLeftSpeed;						
              } else {
	        _motorRightSpeed = MAX_MOTOR_SPEED;
	        _motorLeftSpeed = -_motorRightSpeed;	
              }      
            }  
          }  
          if (stateNew == State.STATE_OFF){
            _motorLeftSpeed = _motorRightSpeed = 0;
            _motorMowEnable = false;          
            _driveHome = false;
          }  
          if (stateNew == State.STATE_ERROR){
            _motorLeftSpeed = _motorRightSpeed = 0;
            _motorMowEnable = false;     
            _driveHome = false;
          }
          if (stateNew == State.STATE_PERI_FIND){
            // find perimeter  => drive half speed  
            _motorLeftSpeed = _motorRightSpeed = MAX_MOTOR_SPEED / 2;
          }
          if (stateNew == State.STATE_PERI_TRACK){        
            //beep(6);
          }
          if (_motorMowEnable) {
            if (stateNew == State.STATE_FORWARD) 
                setActuator(Actuator.ACT_MOTOR_MOW, MAX_MOW_SPEED);
          } else {
            setActuator(Actuator.ACT_MOTOR_MOW, 0);      
          }    
        }

        // calculate motor speed
        void MotorControl(Stopwatch stopWatch)
        {

            //double TA = ((double)(millis() - lastMotorControlTime)) / 1000.0;  
            if (_stateCurr == State.STATE_PERI_TRACK)
            {
                // track perimeter 
                pidMotor.Ta = ((double)(stopWatch.ElapsedMilliseconds - lastMotorControlTime)) / 1000.0;
                pidMotor.x = perimeterLeft - perimeterRight;
                pidMotor.w = 0;
                pidMotor.y_min = -MAX_MOTOR_SPEED;
                pidMotor.y_max = MAX_MOTOR_SPEED;
                pidMotor.max_pwm = MAX_MOTOR_SPEED;
                pidMotor.Kp = 50.0;  // Verstaerkungsfaktor Regelabweichung 		
                pidMotor.Ki = 0.0; 	// Faktor fuer Integral Regelabweichung	
                pidMotor.Kd = 0.01; // Faktor fuer Ableitung Regelabweichung						
                PIDControl(&pidMotor);
                motorLeftPWM = min(MAX_MOTOR_SPEED, max(0, MAX_MOTOR_SPEED / 2 + pidMotor.y));
                motorRightPWM = min(MAX_MOTOR_SPEED, max(0, MAX_MOTOR_SPEED / 2 - pidMotor.y));
            }
            else if ((useCompass) && (motorLeftSpeed == motorRightSpeed == MAX_MOTOR_SPEED) && (stopWatch.ElapsedMilliseconds > stateStartTime + 1000))
            {
                // compass tracking
                pidMotor.Ta = ((double)(stopWatch.ElapsedMilliseconds - lastMotorControlTime)) / 1000.0;
                pidMotor.x = comHeading;
                pidMotor.w = driveHeading;
                pidMotor.y_min = -MAX_MOTOR_SPEED;
                pidMotor.y_max = MAX_MOTOR_SPEED;
                pidMotor.max_pwm = MAX_MOTOR_SPEED;
                pidMotor.Kp = 50.0;  // Verstaerkungsfaktor Regelabweichung 		
                pidMotor.Ki = 0.0; 	// Faktor fuer Integral Regelabweichung	
                pidMotor.Kd = 0.01; // Faktor fuer Ableitung Regelabweichung						
                PIDControl(&pidMotor);
                motorLeftPWM = min(MAX_MOTOR_SPEED, max(0, MAX_MOTOR_SPEED / 2 + pidMotor.y));
                motorRightPWM = min(MAX_MOTOR_SPEED, max(0, MAX_MOTOR_SPEED / 2 - pidMotor.y));
            }
            else
            {
                // normal drive  (slow acceleration/brake)    
                int leftPWM = motorLeftSpeed;
                int rightPWM = motorRightSpeed;
                if (stopWatch.ElapsedMilliseconds < stateStartTime + 1000)
                {
                    leftPWM = rightPWM = 0;
                }
                boolean brakeLeft = ((motorLeftPWM > 0) && (leftPWM <= 0)) || ((motorLeftPWM < 0) && (leftPWM >= 0));
                boolean brakeRight = ((motorRightPWM > 0) && (rightPWM <= 0)) || ((motorRightPWM < 0) && (rightPWM >= 0));
                double accel;
                if (brakeLeft || brakeRight)
                {
                    // brake quickly
                    accel = 0.5;
                }
                else
                {
                    // accelerate slowly
                    accel = 0.01;
                }
                motorLeftPWM = motorLeftPWM * (1.0 - accel) + ((double)leftPWM) * accel;
                motorRightPWM = motorRightPWM * (1.0 - accel) + ((double)rightPWM) * accel;
                driveHeading = comHeading;
            }
            _lastMotorControlTime = stopWatch.ElapsedMilliseconds;
            setMotorSpeed(motorLeftPWM, motorRightPWM);
        }


        void setActuator(Actuator type, int value)
        {
            switch (type)
            {
                case Actuator.ACT_MOTOR_MOW: 
        //           PWM.Start(pinMotorMow)    analogWrite(pinMotorMow, value); 
                    break;
                case Actuator.ACT_MOTOR_LEFT: 
                   MotorDriver1.SetState(HBridge.Motors.Motor1, (sbyte)value);// setL298(pinMotorLeftDir, pinMotorLeftPWM, value); 
                    break;
                case Actuator.ACT_MOTOR_RIGHT: 
                   MotorDriver1.SetState(HBridge.Motors.Motor2, (sbyte)value);// setL298(pinMotorLeftDir, pinMotorLeftPWM, value); 
                    break;
                //case Actuator.ACT_BUZZER: if (value == 0) noTone(pinBuzzer); else tone(pinBuzzer, value); break;
                //case Actuator.ACT_LED: digitalWrite(pinLED, value); break;
            }
        }

    }
    public enum Direction { 
        LEFT, 
        RIGHT 
    }

    public enum State {
        STATE_OFF,          // off
        STATE_FORWARD,      // drive forward
        STATE_ROLL,         // drive roll right/left
        STATE_REVERSE,      // drive reverse
        STATE_CIRCLE,       // drive circle
        STATE_CHARGE,       // charge
        STATE_ERROR,        // error
        STATE_PERI_FIND,    // find perimeter 
        STATE_PERI_TRACK,   // track perimeter

    }
    
    public enum Actuator
    {
        ACT_MOTOR_LEFT,
        ACT_MOTOR_RIGHT,
        ACT_MOTOR_MOW,
    }
    public enum Sensor {
      SEN_PERIM_LEFT,
      SEN_PERIM_RIGHT, 
      SEN_PERIM_LEFT_EXTRA, 
      SEN_PERIM_RIGHT_EXTRA,   
      SEN_BAT_VOLTAGE,
      SEN_CHG_CURRENT,  
      SEN_MOTOR_LEFT,
      SEN_MOTOR_RIGHT,
      SEN_MOTOR_MOW,
      SEN_BUMPER_LEFT,
      SEN_BUMPER_RIGHT,
      SEN_SONAR_CENTER,
      SEN_SONAR_LEFT,
      SEN_SONAR_RIGHT,
      SEN_BUTTON,
      SEN_ACCEL,
      SEN_COMPASS,
      SEN_GYRO,
    }

}
