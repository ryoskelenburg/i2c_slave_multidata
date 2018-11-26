/* !!!read me!!!

    neutralVal +- 5 で閾値を設定し，ニュートラル時には全アクチュエータを解放する．
    sleepTime でスリープ時間を設定し，スリープ時には全アクチュエータを解放する．(不要？)
    変換関数
*/

#include <Wire.h>
#define ANALOG_NUM 3
#define TOTAL_ANALOG_NUM ANALOG_NUM * 2
#define Threshold 5

int analogVal[ANALOG_NUM] = {0, 0, 0}; //自身の入力のみ

int pumpSupplyPin[ANALOG_NUM] = {3, 6, 10};
int pumpVacuumPin[ANALOG_NUM] = {5, 9, 11};
int valveSupplyPin[ANALOG_NUM] = {14, 16, 18};
int valveVacuumPin[ANALOG_NUM] = {15, 17, 19};

int neutralVal = 50;
int rate[TOTAL_ANALOG_NUM] = {0, 0, 0, 0, 0, 0}; //0~100 ここにデータが格納される

int PWM[ANALOG_NUM] = {0, 0, 0};
boolean bDeform[ANALOG_NUM] = {false};
boolean bPolarity[ANALOG_NUM] = {false};
boolean bNeutral[ANALOG_NUM] = {false};

#define LED 8
#define SW 7
boolean bLed = false;
boolean bRealtime = false;
int swVal = 0;
int swCount = 0;

int masterStatus = 0;
int sendSwitch = 0;
int receiveSwitch = 0;

int oldBLed = 0;
int booleanDelta = 0;

//PID
int delta[ANALOG_NUM][2] = {{0}, {0}};
int absDelta[ANALOG_NUM] = {0};
float dt = 0;
float integral;
float KP = 4.0; //Pゲイン
float KI = 0.0; //Iゲイン
float KD = 0.1; //Dゲイン
float p = 0.0;
float i = 0.0;
float d = 0.0;

void setup() {
  Wire.begin(8);// Slave ID #8
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);

  pinMode(LED, OUTPUT);
  pinMode(SW, INPUT);
  digitalWrite(LED, LOW);
  digitalWrite(SW, LOW);

  for (int i = 0; i < ANALOG_NUM; i++) {
    pinMode(pumpSupplyPin[i], OUTPUT);
    pinMode(pumpVacuumPin[i], OUTPUT);
    pinMode(valveSupplyPin[i], OUTPUT);
    pinMode(valveVacuumPin[i], OUTPUT);
  }

  for (int i = 0; i < ANALOG_NUM; i++) {
    digitalWrite(pumpSupplyPin[i], LOW);
    digitalWrite(pumpVacuumPin[i], LOW);
    digitalWrite(valveSupplyPin[i], LOW);
    digitalWrite(valveVacuumPin[i], LOW);
  }
}

void loop() {

  for (int i = 0; i < ANALOG_NUM; i++) {
    analogVal[i] = analogRead(i) / 4;
  }

  /*----*/

  printMinMax();

  switchPlay();//スイッチ
  workRealtime();//

  if (receiveSwitch == 3) {
    bLed = !bLed;
    bRealtime = !bRealtime;
  }

  booleanDelta = oldBLed - (int)bLed;
  oldBLed = (int)bLed;

  delay(100 / 3);
}

void receiveEvent() {
  if ( Wire.available() > 7 ) {
    for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
      rate[i] = Wire.read();
    }
    masterStatus = Wire.read(); //masterのステータス
    receiveSwitch = Wire.read();//スイッチの受信
  }
}

void requestEvent() {
  for (int i = 0; i < ANALOG_NUM; i++) {
    Wire.write(analogVal[i]);
  }
  Wire.write(bLed);//自分のステータス
  if (booleanDelta == -1 && masterStatus == 1) { //slaveがon, かつmasterもonの時,masterをoffにする
    sendSwitch = 3;
  } else {
    sendSwitch = 0;
  }
  Wire.write(sendSwitch);//相手のスイッチ
}

void switchPlay() {
  swVal = digitalRead(SW);

  if (swVal == HIGH) {
    swCount += 1;
  } else {
    swCount = 0;
  }

  if (swCount == 10) {
    bLed = !bLed;
    bRealtime = !bRealtime;
  }

  if (bLed) {
    digitalWrite(LED, HIGH);

  } else {
    digitalWrite(LED, LOW);
  }

  delay(1);
}

void workRealtime() {
  if (bRealtime == true) {
    //input
    for (int i = 0; i < ANALOG_NUM; i++) {
      fbJudge(i, i + 3); //teacherが左 0-3, 1-4, 2-5
      fbOutput(i, i + 3);
    }
  } else {
    for (int i = 0; i < ANALOG_NUM; i++) {
      sendDigitalExhaust(i + 3);
    }
  }
}

void fbJudge(int teacher, int child) { //目標値，センサー値
  dt = 100 / 3;

  delta[teacher][0] = delta[teacher][1]; //過去の偏差を格納

  delta[teacher][1] = rate[teacher] - rate[child]; //**偏差の更新**
  absDelta[teacher] = abs(delta[teacher][1]); //偏差の絶対値
  integral += (delta[teacher][1] + delta[teacher][0]) / 2.0 * dt;

  int dd = delta[teacher][1] - delta[teacher][0];//偏差の変化量

  p = KP * delta[teacher][1]; //定数*偏差(0~100) 255 =  gain * 100
  i = KI * integral;
  d = KD * dd / dt;

  setPWM_PID(p, 0, 0, child);

  if (absDelta[teacher] >= Threshold) {
    bDeform[teacher] = true;
  } else {
    bDeform[teacher] = false;
  }

  if (delta[teacher][1] > 0) {
    bPolarity[teacher] = true;
  }
  else if (delta[teacher][1] < 0) {
    bPolarity[teacher] = false;
  }

  if (neutralVal - Threshold < rate[teacher] && rate[teacher] < neutralVal + Threshold) { //45~ 55のとき
    bNeutral[teacher] = true;
  } else {
    bNeutral[teacher] = false;
  }
}

int setPWM_PID(int p, int i, int d, int number) {
  //pwmに変換
  PWM[number - 3] = abs(p + i + d);
  if (PWM[number - 3] < 50) {
    PWM[number - 3] = 0;
  } else if (PWM[number - 3] >= 255) {
    PWM[number - 3] = 255;
  }
  return PWM[number - 3];
}

void fbOutput(int teacher, int child) {
  if (bNeutral[teacher] == true) { //ニュートラルかどうか
    sendDigitalExhaust(child);
  } else {
    if (bDeform[teacher] == true) { // 偏差があるかどうか
      if (bPolarity[teacher] == true) { //正負の判定
        sendDigitalSupply(child, PWM[child - 3]);
      } else {
        sendDigitalVacuum(child, PWM[child - 3]);
      }
    } else {
      sendDigitalClose(child);
    }
  }
}

//--------------------------------------

void sendDigitalSupply(int number, int PWM) {
  digitalWrite(valveSupplyPin[number - 3], HIGH);
  digitalWrite(valveVacuumPin[number - 3], LOW);
  analogWrite(pumpSupplyPin[number - 3], PWM);
  analogWrite(pumpVacuumPin[number - 3], 0);
}

void sendDigitalVacuum(int number, int PWM) {
  digitalWrite(valveSupplyPin[number - 3], LOW);
  digitalWrite(valveVacuumPin[number - 3], HIGH);
  analogWrite(pumpSupplyPin[number - 3], 0);
  analogWrite(pumpVacuumPin[number - 3], PWM);
}

void sendDigitalClose(int number) {
  digitalWrite(valveSupplyPin[number - 3], HIGH);
  digitalWrite(valveVacuumPin[number - 3], LOW);
  analogWrite(pumpSupplyPin[number - 3], 0);
  analogWrite(pumpVacuumPin[number - 3], 0);
}

void sendDigitalExhaust(int number) {
  digitalWrite(valveSupplyPin[number - 3], LOW);
  digitalWrite(valveVacuumPin[number - 3], LOW);
  analogWrite(pumpSupplyPin[number - 3], 0);
  analogWrite(pumpVacuumPin[number - 3], 0);
}

void printMinMax() {
  Serial.print("[");
  Serial.print(rate[0]);
//  Serial.print(":");
//  Serial.print(filteredVal[0][1]);
//  Serial.print("]");
//  Serial.print(minVal[0]);
//  Serial.print(" - ");
//  Serial.print(maxVal[0]);

  Serial.print(", [");
  Serial.print(rate[1]);
//  Serial.print(":");
//  Serial.print(filteredVal[1][1]);
//  Serial.print("]");
//  Serial.print(minVal[1]);
//  Serial.print(" - ");
//  Serial.print(maxVal[1]);

  Serial.print(", [");
  Serial.print(rate[2]);
//  Serial.print(":");
//  Serial.print(filteredVal[2][1]);
//  Serial.print("]");
//  Serial.print(minVal[2]);
//  Serial.print(" - ");
//  Serial.print(maxVal[2]);

  Serial.print(", [");
  Serial.print(rate[3]);
//  Serial.print(":");
//  Serial.print(filteredVal[3][1]);
//  Serial.print("]");
//  Serial.print(minVal[3]);
//  Serial.print(" - ");
//  Serial.print(maxVal[3]);

  Serial.print(", [");
  Serial.print(rate[4]);
//  Serial.print(":");
//  Serial.print(filteredVal[4][1]);
//  Serial.print("]");
//  Serial.print(minVal[4]);
//  Serial.print(" - ");
//  Serial.print(maxVal[4]);

  Serial.print(", [");
  Serial.print(rate[5]);
//  Serial.print(":");
//  Serial.print(filteredVal[5][1]);
//  Serial.print("]");
//  Serial.print(minVal[5]);
//  Serial.print(" - ");
//  Serial.println(maxVal[5]);

}
