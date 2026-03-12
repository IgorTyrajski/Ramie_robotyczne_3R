#include <Servo.h>
#include <math.h>

#define START 400
#define END 2400

struct angle{
  float degrees;
  int microseconds;

  angle(const float valDeg){
    degrees = valDeg;
    microseconds = map(valDeg,0 , 180, START, END );
  }
};

struct roboticArm{
  Servo base;
  Servo shoulder;
  Servo elbow;

  angle theta1 = 0.0f; //base angle
  angle theta2 = 0.0f; //shoulder angle
  angle theta3 = 0.0f; //elbow angle

  float xPos = 0.0f;
  float yPos = 0.0f;
  float zPos = 0.0f;

  // DH parameters
  const float D1 = 0.165;      
  const float LEN_A2 = 0.153;  
  const float LEN_A3 = 0.175;  
  const float D_OFFSET = 0.05 + 0.013; // 0.063 m

  void init(const int basePin = 8, const int shoulderPin = 9, const int elbowPin = 10){
    pinMode(basePin, OUTPUT);
    pinMode(shoulderPin, OUTPUT);
    pinMode(elbowPin, OUTPUT);

    base.attach(basePin, START, END);
    shoulder.attach(shoulderPin, START, END);
    elbow.attach(elbowPin, START, END);
  }

 bool calculateIK(float x, float y, float z, float &calcTheta1, float &calcTheta2, float &calcTheta3) {
    
    float r_xy = sqrt(x * x + y * y);
    if (r_xy < D_OFFSET) return false; 
    
    float gamma = atan2(y, x);
    float beta = asin(D_OFFSET / r_xy);
    calcTheta1 = gamma + beta; 
    
    float R = sqrt(r_xy * r_xy - D_OFFSET * D_OFFSET); 
    float Z = z - D1; 
    float L = sqrt(R * R + Z * Z);

    if (L > (LEN_A2 + LEN_A3) || L < abs(LEN_A2 - LEN_A3)) return false; 

    float cos_theta3 = (L * L - LEN_A2 * LEN_A2 - LEN_A3 * LEN_A3) / (2.0 * LEN_A2 * LEN_A3);
    cos_theta3 = constrain(cos_theta3, -1.0, 1.0); 
    calcTheta3 = acos(cos_theta3); 

    float alpha = atan2(Z, R); 
    float cos_phi = (L * L + LEN_A2 * LEN_A2 - LEN_A3 * LEN_A3) / (2.0 * L * LEN_A2);
    cos_phi = constrain(cos_phi, -1.0, 1.0);
    float phi = acos(cos_phi); 
    
    calcTheta2 = alpha - phi; 

    calcTheta1 = calcTheta1 * 180.0 / PI;
    calcTheta2 = calcTheta2 * 180.0 / PI;
    calcTheta3 = calcTheta3 * 180.0 / PI;

    if (calcTheta1 < -90 || calcTheta1 > 90 ||
        calcTheta2 < -90 || calcTheta2 > 90 ||
        calcTheta3 < -90 || calcTheta3 > 90) {
        return false; 
    }

    return true; 
  }
  bool moveTo(float x, float y, float z) {
    float tempTheta1, tempTheta2, tempTheta3; 
    
    if (calculateIK(x, y, z, tempTheta1, tempTheta2, tempTheta3)) {
      xPos = x;
      yPos = y;
      zPos = z;
      Serial.print("Base angle:   "); Serial.print(tempTheta1); Serial.println("degrees (real)");
      Serial.print("Shoulder angle:   "); Serial.print(tempTheta2); Serial.println("degrees (real)");
      Serial.print("Elbow angle:   "); Serial.print(tempTheta3); Serial.println("degrees (real)");
      writeAngles(tempTheta1 + 90.0f, tempTheta2 + 90.0f, tempTheta3 + 90.0f);
      return true;
    }
    return false; 
  }

  void writeAngles(float newTheta1, float newTheta2, float newTheta3){
    theta1.degrees = newTheta1;
    theta2.degrees = newTheta2;
    theta3.degrees = newTheta3;
    theta1.microseconds = map(newTheta1,0 , 180, START, END);
    theta2.microseconds = map(newTheta2,0 , 180, START, END);
    theta3.microseconds = map(newTheta3,0 , 180, START, END);
    
    base.writeMicroseconds(theta1.microseconds);
    shoulder.writeMicroseconds(theta2.microseconds);
    elbow.writeMicroseconds(theta3.microseconds);

    Serial.print("Base angle:   "); Serial.print(theta1.degrees); Serial.println("degrees (on servo)");
    Serial.print("Shoulder angle:   "); Serial.print(theta2.degrees); Serial.println("degrees (on servo)");
    Serial.print("Elbow angle:   "); Serial.print(theta3.degrees); Serial.println("degrees (on servo)");
  }

  void setHomePos(){
    writeAngles(90.0f, 90.0f, 90.0f);
  }

};

roboticArm Arm3R;

void setup() {
  Serial.begin(9600);
  Arm3R.init();
}

void loop() {

  Serial.println("=========================================");
  Serial.println("Podaj wspolrzedne celu [X Y Z] w metrach.");
  Serial.println("Przyklad: 0.13 0.28 0.16");
  Serial.println("Czekam na dane...");

  while (Serial.available() == 0) {
  }

  float celX = Serial.parseFloat();
  float celY = Serial.parseFloat();
  float celZ = Serial.parseFloat();

  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.print("-> Zrozumialem cel: X="); Serial.print(celX);
  Serial.print(", Y="); Serial.print(celY);
  Serial.print(", Z="); Serial.println(celZ);

  bool sukces = Arm3R.moveTo(celX, celY, celZ);

  
  if (!sukces) {
    Serial.println("!!! BLAD: Punkt jest poza zasiegiem robota lub lamie limity (-90 do 90) !!!\n");
  } else {
    Serial.println("-> SUKCES: Ramie wykonalo ruch.\n");
  }
  delay(1000); 
}
  


