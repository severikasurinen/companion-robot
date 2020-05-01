#include <Stepper.h> // include steooer motor library
#include <Servo.h> // include servo library

const int enablePin = 6, fwdPin = 7, bwdPin = 8, servoPin = 9, trigPin = 10, echoPin = 11, speakerPin = 12, btnPin = 13; // motor pins, servo pin, ultrasonic sensor pins, speaker pin, and button pin
const int revSteps = 2048, motorSpeed = 13, radarPortions = 24, scanNum = 3, startScans = 1; // steps per revolution (set according to datasheet), stepper motor rpm, radar scans per rotation (minimum is 4), and number of radar scans per check
const int scanDelay = 65, acceptedDistRange[] = { 2, 450 }, targetDistRange[] = { 40, 75, 125 }; // minimum time in ms between radar scans (datasheet suggests a minimum of 60), min and max distances accepted in selected units (according to the datasheet, the ultrasonic sensor's range is about 2cm-450cm), and min, ideal and max distances to target
const int detectDist = 200, obstacleDist = 20; // distance in selected units for detecting objects
const int detectScanAngle = 50, frontAngle = 25, servoAngles[] = { 15, 86, 165 }; // approx. angle to scan when object has been detected, angle accepted for keeping target in front, and servo angles
const float startThreshold = 40, trackingThreshold = 5, targetTimeout = 20.0; // minimum distance change for movement, maximum distance change for movement, expected distance change, time of still target to untarget
const float soundInterval = 6.0; // interval of making sounds
const bool metric = true; // true = cm, false = inches

Stepper stepper(revSteps, 2, 4, 3, 5); // revSteps and stepper motor pins
Servo servo;

bool started = false, ended = false, clockWise = true, falseDetections[radarPortions]; // has program started? has program ended? is radar turning clockwise?
int turns = radarPortions/2, dir = -1, scans = 0, targetNum = 0, targetDist = 0; // current radar direction, and direction of detected object (-1 = null)
float dist[radarPortions], distChange[radarPortions]; // distances at each radar direction, and distance changes
unsigned long lastScan = 0, lastMovement = 0, lastSound = 0; // time of last scan and target movement

void setup()
{
  stepper.setSpeed(motorSpeed); // set stepper motor rpm
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(fwdPin, OUTPUT);
  pinMode(bwdPin, OUTPUT);
  pinMode(btnPin, INPUT);
  servo.attach(servoPin);
  
  // default to max distance
  for(int i=0; i<radarPortions; i++)
  {
    dist[i] = acceptedDistRange[1];
    distChange[i] = 0;
    falseDetections[i] = false;
  }
  
  Serial.begin(9600);

  Turn(radarPortions/2);
}

void loop()
{
  if(started && !ended)
  {
    if(dir == -1)
    {
      // mark down index number of current direction
      int o = turns;
      if(o == radarPortions)
      {
        o = 0;
      }
      
      float change = dist[o];
      Scan(o); // request a scan
      if(change < acceptedDistRange[1])
      {
        change = abs(dist[o] - change);
      }
      else
      {
        change = 0;
      }
      if(dist[o] <= detectDist && change >= startThreshold)
      {
        if(scans >= startScans)
        {
          if(targetNum > 0 || (targetNum == 0 && !falseDetections[o]))
          {
            distChange[o] = change;
            
            Serial.print("Movement detected! Change: ");
            Serial.print(change);
            Serial.print(" ");
            if(metric)
            {
              Serial.println("cm");
            }
            else
            {
              Serial.println("inches");
            }
          }
        }
        else
        {
          falseDetections[o] = true;
          
          Serial.print("False detection in [");
          Serial.print(o);
          Serial.println("]");
        }
      }
      else
      {
        distChange[o] = 0;
      }
      
      if(scans >= startScans)
      {
        int movementDirs = 0;
        float minDist = acceptedDistRange[1];
        
        for(int i=0; i<radarPortions; i++)
        {
          if(distChange[i] > 0 && dist[i] <= detectDist)
          {
            movementDirs++;
            if(dist[i] < minDist)
            {
              minDist = dist[i];
              dir = i;
              targetDist = dist[dir];
              lastMovement = millis();
            }
          }
        }
      }
    
      RotateRadar(-1); // rotate radar
    }
    else
    {
      int movementDirs = 0;
      
      float lastDist = dist[dir];
      int lRots = round((detectScanAngle/(360.0/radarPortions)));
      int minChange = acceptedDistRange[1]-acceptedDistRange[0];
      bool lRight = turns > radarPortions/2;
      for(int i=0; i<(lRots*2+1); i++)
      {
        int o = radarPortions/2-lRots + i;
        if(lRight)
        {
          o = radarPortions/2+lRots - i;
        }
        float change = dist[o];
        Scan(o);
        change = abs(dist[o] - change);
        if(change >= trackingThreshold)
        {
          movementDirs++;
        }
        
        if(abs(dist[o]-lastDist) < minChange && dist[o] <= detectDist)
        {
          minChange = abs(dist[o]-lastDist);
          dir = o;
        }
      }
      Turn(dir);
      KeepDistance();
      
      if(movementDirs > 0)
      {
        lastMovement = millis();
      }
    }
  
    if(millis() - lastMovement >= targetTimeout*1000)
    {
      dir = -1;
    }

    if(millis() - lastSound >= soundInterval*1000)
    {
      Sound(random(5)); // play random sound
      lastSound = millis();
    }
    
    if(digitalRead(btnPin) == HIGH)
    {
      // stop program
      ended = true;
      Play(400, 1000);
      RotateRadar(radarPortions/2);
    }
  }
  else if(!ended)
  {
    if(digitalRead(btnPin) == HIGH)
    {
      // start program
      started = true;
      Play(400, 1000);
    }
    else
    {
      delay(100);
    }
  }
  else
  {
    delay(10000); // idle
  }
}

void Play(int freq, int duration)
{
  tone(speakerPin, freq, duration);
  delay(duration);
}

void Drive(bool forward, float driveSpeed, int duration)
{
  analogWrite(enablePin, round(driveSpeed*255));
  if(forward)
  {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(bwdPin, LOW);
  }
  else
  {
    digitalWrite(bwdPin, HIGH);
    digitalWrite(fwdPin, LOW);
  }
  delay(duration);
  digitalWrite(fwdPin, LOW);
  digitalWrite(bwdPin, LOW);
}

void KeepDistance()
{
  bool moved = false;
  Scan(dir); // request a scan
  if(dist[dir] < targetDistRange[0])
  {
    Scan(0);
    if(dist[0] > obstacleDist)
    {
      Drive(false, 0.8, abs(targetDistRange[1]-dist[dir])*22.47);
      moved = true;
    }
  }
  else if(dist[dir] > targetDistRange[2])
  {
    Drive(true, 0.8, abs(targetDistRange[1]-dist[dir])*22.47);
    moved = true;
  }
  
  if(moved)
  {
    for(int i=0; i<radarPortions; i++)
    {
      dist[i] = acceptedDistRange[1];
      distChange[i] = 0;
    }
    int lRots = round((detectScanAngle/(360.0/radarPortions)));
    int minChange = acceptedDistRange[1]-acceptedDistRange[0];
    bool lRight = turns > radarPortions/2;
    for(int i=0; i<(lRots*2+1); i++)
    {
      int o = radarPortions/2-lRots + i;
      if(lRight)
      {
        o = radarPortions/2+lRots - i;
      }
      Scan(o);
      if(abs(dist[o]-targetDistRange[1]) < minChange && dist[o] <= detectDist)
      {
        minChange = abs(dist[o]-targetDistRange[1]);
        dir = o;
      }
    }
    RotateRadar(dir);
  }
}

void Turn(int targetDir)
{
  float angleToTarget = (abs(targetDir-(radarPortions/2))/(float)radarPortions)*360;
  if(angleToTarget > frontAngle)
  {
    Scan(dir); // request a scan
    float startDist = dist[dir];
    
    bool right = targetDir > radarPortions/2;
    if(right)
    {
      servo.write(servoAngles[2]);
    }
    else
    {
      servo.write(servoAngles[0]);
    }
    delay(150);
    Drive(true, 0.5, (angleToTarget/30)*250);
    delay(50);
    servo.write(servoAngles[1]);
    
    int lRots = round((detectScanAngle/(360.0/radarPortions)));
    int minChange = acceptedDistRange[1]-acceptedDistRange[0];
    bool lRight = turns > radarPortions/2;
    for(int i=0; i<(lRots*2+1); i++)
    {
      int o = radarPortions/2-lRots + i;
      if(lRight)
      {
        o = radarPortions/2+lRots - i;
      }
      Scan(o);
      if(abs(dist[o]-startDist) < minChange && dist[o] <= detectDist)
      {
        minChange = abs(dist[o]-startDist);
        dir = o;
      }
    }
    for(int i=0; i<radarPortions; i++)
    {
      dist[i] = acceptedDistRange[1];
      distChange[i] = 0;
    }
    RotateRadar(dir);
  }
  else
  {
    servo.write(servoAngles[1]);
  }
}

void RotateRadar(int rotDir) // rotate radar
{
  if(rotDir == -1)
  {
    if(clockWise) // check if radar is turning clockwise
    {
      stepper.step(revSteps/radarPortions); // turn clockwise to next scan position
      turns++;
      if(turns == radarPortions || (!ended && dir != -1 && turns >= dir + round(detectScanAngle/(360.0/radarPortions)))) // check if reached full rotation or if scanning certain direction
      {
        clockWise = false;
      }
    }
    else
    {
      stepper.step(-revSteps/radarPortions); // turn anticlockwise to next scan position
      turns--;
      if(turns == 0 || (!ended && dir != -1 && turns <= dir - round((detectScanAngle/(360.0/radarPortions))))) // check if reached full rotation or if scanning certain direction
      {
        clockWise = true;
        if(scans < startScans)
        {
          scans++;
        }
      }
    }
  }
  else
  {
    int rotSteps = rotDir-turns;
    if(rotDir == 0 && turns > radarPortions/2)
    {
      rotSteps = radarPortions-turns;
    }
    
    stepper.step(rotSteps*(revSteps/radarPortions));
    turns+= rotSteps;
    if(turns == 0) // check if reached full rotation
    {
      clockWise = true;
    }
    else if(turns == radarPortions) // check if reached full rotation
    {
      clockWise = false;
    }
  }
}

void Scan(int lDist) // scan distance
{
  RotateRadar(lDist);
  
  unsigned int lTime = millis() - lastScan;
  if(lTime < scanDelay) // check if enough time has passed since last scan
  {
    delay(scanDelay - lTime); // if not, wait for minimum time
  }

  float avgDist = 0;
  float sum = 0;
  float dists = 0;
  float lMin = acceptedDistRange[1];
  float lMax = acceptedDistRange[0];
  
  for(int i=0; i<scanNum; i++)
  {
    if(i != 0)
    {
      delay(scanDelay);
    }

    // make sure trigger pin is off
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // send 10μs pulse to ultrasonic sensor
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float lDist = (0.0343*pulseIn(echoPin, HIGH)/2); // read signal travel time and convert to distance in cm
    if(!metric)
    {
      lDist *= 2.54; // convert to inches if selected unit
    }
    
    if(lDist < 0)
    {
      lDist = -lDist; // sometimes value received is the opposite value of what it's supposed to be
    }
    if(lDist >= acceptedDistRange[0] && lDist <= acceptedDistRange[1]) // check if value is in accepter range
    {
      sum += lDist;
      dists++;
      lMin = min(lDist, lMin);
      lMax = max(lDist, lMax);
    }
  }
  
  lastScan = millis(); // mark down the time of the last scan

  if(dists == 0) // check if value is acceptable
  {
    avgDist = acceptedDistRange[1]; // if not, default to max value
  }
  else
  {
    avgDist = sum/dists;
    if(dists >= 3)
    {
      if(abs(avgDist-lMin)/avgDist > 0.25 || abs(avgDist-lMin) > 20)
      {
        sum -= lMin;
        dists--;
      }
      if(abs(avgDist-lMax)/avgDist > 0.25 || abs(avgDist-lMax) > 20)
      {
        sum -= lMax;
        dists--;
      }
    }
    
    avgDist = sum/dists;
  }
  
  dist[lDist] = avgDist;
  
  // print data in serial monitor
  Serial.print("Distance[");
  Serial.print(lDist);
  Serial.print("]: ");
  Serial.print(dist[lDist]);
  Serial.print(" ");
  if(metric)
  {
    Serial.print("cm");
  }
  else
  {
    Serial.print("inches");
  }
  Serial.print(" --- Object direction: ");
  Serial.println(dir);
}

void Sound(int n)
{
  switch(n)
  {
    case 0:
      tone(speakerPin, 622, 41.86);
      delay(46.51);
      delay(11.63);
      tone(speakerPin, 698, 20.93);
      delay(23.26);
      delay(11.63);
      delay(11.63);
      tone(speakerPin, 622, 10.47);
      delay(11.63);
      tone(speakerPin, 3135, 10.47);
      delay(11.63);
      tone(speakerPin, 1567, 41.86);
      delay(46.51);
      tone(speakerPin, 783, 10.47);
      delay(11.63);
      tone(speakerPin, 659, 20.93);
      delay(23.26);
      tone(speakerPin, 1661, 73.26);
      delay(81.4);
      tone(speakerPin, 698, 10.47);
      delay(11.63);
      delay(81.4);
      delay(11.63);
      delay(11.63);
      tone(speakerPin, 1174, 41.86);
      delay(46.51);
      delay(69.77);
      delay(23.26);
      tone(speakerPin, 1318, 41.86);
      delay(46.51);
      tone(speakerPin, 659, 10.47);
      delay(11.63);
      delay(11.63);
      delay(11.63);
      delay(23.26);
      delay(34.88);
      tone(speakerPin, 554, 41.86);
      delay(46.51);
      tone(speakerPin, 1108, 10.47);
      delay(11.63);
      delay(69.77);
      tone(speakerPin, 739, 94.19);
      delay(104.651);
      tone(speakerPin, 2217, 10.47);
      delay(11.63);
      delay(11.63);
      delay(23.26);
      tone(speakerPin, 2489, 10.47);
      delay(11.63);
      tone(speakerPin, 830, 10.47);
      delay(11.63);
      delay(23.26);
      delay(11.63);
      tone(speakerPin, 2959, 10.47);
      delay(11.63);
      tone(speakerPin, 2489, 20.93);
      delay(23.26);
      delay(11.63);
      delay(11.63);
      tone(speakerPin, 2637, 10.47);
      delay(11.63);
      tone(speakerPin, 1975, 10.47);
      delay(11.63);
      tone(speakerPin, 3135, 20.93);
      delay(23.26);
      delay(69.77);
      delay(46.51);
      delay(11.63);
      tone(speakerPin, 2637, 31.4);
      delay(34.88);
      tone(speakerPin, 830, 10.47);
      delay(11.63);
      break;
    case 1:
      tone(speakerPin, 466, 41.04);
      delay(45.6);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 554, 10.26);
      delay(11.4);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 2093, 30.78);
      delay(34.2);
      tone(speakerPin, 830, 10.26);
      delay(11.4);
      tone(speakerPin, 1661, 10.26);
      delay(11.4);
      delay(22.8);
      delay(22.8);
      delay(11.4);
      delay(34.2);
      tone(speakerPin, 246, 30.78);
      delay(34.2);
      tone(speakerPin, 493, 10.26);
      delay(11.4);
      tone(speakerPin, 739, 10.26);
      delay(11.4);
      delay(22.8);
      delay(11.4);
      tone(speakerPin, 622, 20.52);
      delay(22.8);
      delay(11.4);
      tone(speakerPin, 880, 10.26);
      delay(11.4);
      tone(speakerPin, 1318, 10.26);
      delay(11.4);
      tone(speakerPin, 1108, 20.52);
      delay(22.8);
      tone(speakerPin, 1396, 10.26);
      delay(11.4);
      delay(34.2);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 587, 30.78);
      delay(34.2);
      tone(speakerPin, 1975, 10.26);
      delay(11.4);
      delay(68.4);
      delay(22.8);
      tone(speakerPin, 739, 41.04);
      delay(45.6);
      tone(speakerPin, 369, 10.26);
      delay(11.4);
      tone(speakerPin, 783, 10.26);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 830, 10.26);
      delay(11.4);
      tone(speakerPin, 987, 10.26);
      delay(11.4);
      tone(speakerPin, 1318, 10.26);
      delay(11.4);
      tone(speakerPin, 2093, 10.26);
      delay(11.4);
      tone(speakerPin, 2217, 10.26);
      delay(11.4);
      tone(speakerPin, 1760, 20.52);
      delay(22.8);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 698, 41.04);
      delay(45.6);
      break;
    case 2:
      tone(speakerPin, 1479, 10.47);
      delay(11.63);
      tone(speakerPin, 932, 10.47);
      delay(11.63);
      tone(speakerPin, 622, 10.47);
      delay(11.63);
      delay(34.88);
      delay(11.63);
      delay(11.63);
      tone(speakerPin, 329, 41.86);
      delay(46.51);
      delay(34.88);
      delay(11.63);
      delay(23.26);
      tone(speakerPin, 1046, 10.47);
      delay(11.63);
      tone(speakerPin, 2093, 10.47);
      delay(11.63);
      tone(speakerPin, 2217, 10.47);
      delay(11.63);
      tone(speakerPin, 1864, 10.47);
      delay(11.63);
      delay(69.77);
      delay(11.63);
      delay(23.26);
      delay(23.26);
      delay(11.63);
      delay(11.63);
      tone(speakerPin, 1864, 10.47);
      delay(11.63);
      tone(speakerPin, 1174, 10.47);
      delay(11.63);
      tone(speakerPin, 783, 10.47);
      delay(11.63);
      delay(11.63);
      tone(speakerPin, 1975, 10.47);
      delay(11.63);
      tone(speakerPin, 2349, 10.47);
      delay(11.63);
      tone(speakerPin, 2637, 31.4);
      delay(34.88);
      tone(speakerPin, 2959, 10.47);
      delay(11.63);
      tone(speakerPin, 3135, 20.93);
      delay(23.26);
      break;
    case 3:
      tone(speakerPin, 880, 20.52);
      delay(22.8);
      tone(speakerPin, 932, 20.52);
      delay(22.8);
      delay(11.4);
      tone(speakerPin, 1479, 20.52);
      delay(22.8);
      delay(45.6);
      delay(11.4);
      tone(speakerPin, 1864, 41.04);
      delay(45.6);
      delay(79.8);
      delay(22.8);
      tone(speakerPin, 783, 30.78);
      delay(34.2);
      tone(speakerPin, 587, 10.26);
      delay(11.4);
      tone(speakerPin, 830, 10.26);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 1567, 20.52);
      delay(22.8);
      tone(speakerPin, 1975, 10.26);
      delay(11.4);
      delay(57);
      tone(speakerPin, 1479, 30.78);
      delay(34.2);
      tone(speakerPin, 739, 10.26);
      delay(11.4);
      delay(34.2);
      delay(11.4);
      tone(speakerPin, 587, 61.56);
      delay(68.4);
      tone(speakerPin, 880, 10.26);
      delay(11.4);
      delay(22.8);
      delay(11.4);
      delay(45.6);
      tone(speakerPin, 830, 10.26);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 1318, 10.26);
      delay(11.4);
      tone(speakerPin, 1567, 10.26);
      delay(11.4);
      delay(11.4);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 1567, 10.26);
      delay(11.4);
      tone(speakerPin, 783, 10.26);
      delay(11.4);
      tone(speakerPin, 2959, 10.26);
      delay(11.4);
      delay(91.2);
      tone(speakerPin, 830, 30.78);
      delay(34.2);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 1318, 10.26);
      delay(11.4);
      tone(speakerPin, 1661, 10.26);
      delay(11.4);
      tone(speakerPin, 2489, 10.26);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 1760, 10.26);
      delay(11.4);
      tone(speakerPin, 2217, 10.26);
      delay(11.4);
      break;
    case 4:
      tone(speakerPin, 466, 30.78);
      delay(34.2);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 880, 10.26);
      delay(11.4);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 2093, 30.78);
      delay(34.2);
      tone(speakerPin, 830, 10.26);
      delay(11.4);
      delay(57);
      delay(22.8);
      tone(speakerPin, 659, 30.78);
      delay(34.2);
      delay(11.4);
      tone(speakerPin, 739, 10.26);
      delay(11.4);
      tone(speakerPin, 622, 20.52);
      delay(22.8);
      delay(22.8);
      tone(speakerPin, 622, 30.78);
      delay(34.2);
      tone(speakerPin, 880, 10.26);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 1396, 30.78);
      delay(34.2);
      delay(45.6);
      delay(11.4);
      tone(speakerPin, 1479, 30.78);
      delay(34.2);
      tone(speakerPin, 1760, 10.26);
      delay(11.4);
      delay(91.2);
      delay(22.8);
      tone(speakerPin, 369, 30.78);
      delay(34.2);
      tone(speakerPin, 783, 20.52);
      delay(22.8);
      delay(11.4);
      delay(11.4);
      tone(speakerPin, 1479, 10.26);
      delay(11.4);
      tone(speakerPin, 1864, 10.26);
      delay(11.4);
      tone(speakerPin, 2217, 10.26);
      delay(11.4);
      delay(45.6);
      tone(speakerPin, 698, 41.04);
      delay(45.6);
      break;
  }
}
