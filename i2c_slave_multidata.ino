#include <Wire.h>
#define ANALOG_NUM 3
#define TOTAL_ANALOG_NUM ANALOG_NUM * 2

//能動入力が0~2, 受動入力が3~5
int masterVal[3];
int slaveVal[3];
int analogVal[TOTAL_ANALOG_NUM];

int pumpSupplyPin[ANALOG_NUM] = {3, 6, 10};
int pumpVacuumPin[ANALOG_NUM] = {5, 9, 11};
int valveSupplyPin[ANALOG_NUM] = {14, 16, 18};
int valveVacuumPin[ANALOG_NUM] = {15, 17, 19};

float a = 0.9;
int filteredVal[TOTAL_ANALOG_NUM][2] = {0}; //0~255
int maxVal[TOTAL_ANALOG_NUM] = {0}; //0~255
int minVal[TOTAL_ANALOG_NUM] = {0}; //0~255

#define RESOLUSION 100
int rate[TOTAL_ANALOG_NUM] = {0}; //0~100

int PWM[ANALOG_NUM] = {0};
bool bDeform[TOTAL_ANALOG_NUM] = {false};
bool bPolarity[TOTAL_ANALOG_NUM] = {false};

boolean bRealtime = false;

//PID
int delta[TOTAL_ANALOG_NUM][2] = {{0}, {0}};
int absDelta[TOTAL_ANALOG_NUM] = {0};
float dt = 100 / 3;
float integral;
float KP = 7.0; //Pゲイン
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

  for (int i = 0; i < 3; i++) {
    analogVal[i] = analogRead(i) / 4;
  }

  /*----*/

  for (int i = 0; i < TOTAL_ANALOG_NUM; i++) { //値のフィルタリング
    adjustData(i);
  }

  Serial.print("slave1: ");
  Serial.print(rate[0]);
  Serial.print(" slave2: ");
  Serial.print(rate[1]);
  Serial.print(" slave3: ");
  Serial.print(rate[2]);
  Serial.print(" master1: ");
  Serial.print(rate[3]);
  Serial.print(" master2: ");
  Serial.print(rate[4]);
  Serial.print(" master3: ");
  Serial.println(rate[5]);

  //switchPlay();//スイッチ
  //workRealtime();//

  for (int i = 0; i < TOTAL_ANALOG_NUM; i++) { //値の更新
    filteredVal[i][0] = filteredVal[i][1];
  }

  delay(100 / 3);
}

void receiveEvent() {
  if ( Wire.available() > 2 ) {
    analogVal[3] = Wire.read();
    analogVal[4] = Wire.read();
    analogVal[5] = Wire.read();
  }
}

void requestEvent() {
  for (int i = 0; i < 3; i++) {
    Wire.write(analogVal[i]);
  }
}

void adjustData(int _number) {

  filteredVal[_number][1] = a * filteredVal[_number][0] + (1 - a) * analogVal[_number]; //フィルタリング
  rate[_number] = map(filteredVal[_number][1], minVal[_number], maxVal[_number], 0, RESOLUSION); //マッピング

  if (filteredVal[_number][1] > maxVal[_number]) { //最大値
    maxVal[_number] = filteredVal[_number][1];
  }
  if (filteredVal[_number][1] < minVal[_number]) { //最小値
    minVal[_number] = filteredVal[_number][1];
  }
}

void workRealtime() {
  if (bRealtime == true) {
    //input
    for (int i = 0; i < ANALOG_NUM; i++) {
      fbJudge(i + 3, i); //teacherが左
      fbOutput(i + 3, i);
    }
  } else {
    for (int i = 0; i < ANALOG_NUM; i++) {
      sendDigitalExhaust(i);
    }
  }
}

void fbJudge(int teacher, int child) { //目標値，センサー値
  delta[teacher][0] = delta[teacher][1]; //過去の偏差を格納

  delta[teacher][1] = rate[teacher] - rate[child]; //**偏差の更新**
  absDelta[teacher] = abs(delta[teacher][1]); //偏差の絶対値
  integral += (delta[teacher][1] + delta[teacher][0]) / 2.0 * dt;

  int dd = delta[teacher][1] - delta[teacher][0];//偏差の変化量

  p = KP * delta[teacher][1]; //定数*偏差(0~100) 255 =  gain * 100
  i = KI * integral;
  d = KD * dd / dt;

  setPWM_PID(p, 0, 0, child);

  if (absDelta[teacher] >= 1) {
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
}

int setPWM_PID(int p, int i, int d, int number) {
  //pwmに変換
  PWM[number] = abs(p + i + d);
  return PWM[number];
}

void fbOutput(int teacher, int child) {
  if (bDeform[teacher] == true) { // 偏差があるかどうか
    if (bPolarity[teacher] == true) { //正負の判定
      sendDigitalSupply(child, PWM[child]);
    } else {
      sendDigitalVacuum(child, PWM[child]);
    }
  } else {
    sendDigitalClose(child);
  }
}

//--------------------------------------

void sendDigitalSupply(int number, int PWM) {
  digitalWrite(valveSupplyPin[number], HIGH);
  digitalWrite(valveVacuumPin[number], LOW);
  analogWrite(pumpSupplyPin[number], PWM);
  analogWrite(pumpVacuumPin[number], 0);
}

void sendDigitalVacuum(int number, int PWM) {
  digitalWrite(valveSupplyPin[number], LOW);
  digitalWrite(valveVacuumPin[number], HIGH);
  analogWrite(pumpSupplyPin[number], 0);
  analogWrite(pumpVacuumPin[number], PWM);
}

void sendDigitalClose(int number) {
  digitalWrite(valveSupplyPin[number], HIGH);
  digitalWrite(valveVacuumPin[number], LOW);
  analogWrite(pumpSupplyPin[number], 0);
  analogWrite(pumpVacuumPin[number], 0);
}

void sendDigitalExhaust(int number) {
  digitalWrite(valveSupplyPin[number], LOW);
  digitalWrite(valveVacuumPin[number], LOW);
  analogWrite(pumpSupplyPin[number], 0);
  analogWrite(pumpVacuumPin[number], 0);
}
