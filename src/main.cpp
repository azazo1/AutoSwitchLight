//
// Created by azazo1 on 2024/2/2.
//
#include <Arduino.h>
#include <BLEDevice.h>
// 我的单片机蓝牙地址为 A0:A3:B3:29:D4:92
#define SERVICE_UUID "9f867bb8-d8a6-43b2-b129-fefc96fcf689"
#define CHAR_UUID "7c6473b1-0164-4842-be7f-d1cc945eaae6"
#define DESCRIPTOR_UUID "08fc14c5-2002-4a9d-8672-15517c70697a"
#define MAX_UNAVAILABLE_CNT 1 // 关灯前扫描无果忍耐次数
#define OUTPUT_LED_PIN 23 // 记得LED要串联一个电阻，不然烧坏
#define MOTOR_PIN1 32
#define MOTOR_PIN2 33
#define MOTOR_PIN3 25
#define MOTOR_PIN4 26
#define MOTOR_ROTATE 1000 // 马达旋转系数，越大马达一次转动的角度越大
// sg90 控制舵机角度的值（占PWM周期的40份之n）https://blog.csdn.net/qq_41873236/article/details/116353829
#define SERVO_STATE_C 2 // 关灯状态
#define SERVO_STATE_O 4 // 开灯状态
#define SERVO_STATE_N 3 // 平常状态
#define SERVO_RESET_TIME 700 // 舵机进入开关灯状态的持续时间（毫秒）

const std::vector<std::string> expected = {
        "iphone 4s",
//        "ipad air"
};
BLEScan *pScan = nullptr;
BLECharacteristic *switchCharacteristic = nullptr;
BLEDescriptor *descriptor = nullptr;

hw_timer_t *timer = nullptr;

bool availablePair = false;
int remoteSwitch = 1;
// 多次检测availablePair为false时才关闭，而一次为true则立马开启
unsigned int unavailableCnt = 0;
// 最终的开关灯决断
bool output = false;
unsigned int motorRotationCnt = 0;
unsigned long servoEnableTime = 0; // 舵机进入开关灯状态的初始时间


class MyConnectingCallBack : public BLEServerCallbacks {
public:
    void onConnect(BLEServer *pServer) override {
        Serial.println("\nServer:connected");
    }

    void onDisconnect(BLEServer *pServer) override {
        Serial.println("\nServer:disconnected");
        BLEDevice::startAdvertising();
    }
};

class MyCharacteristicCallBack : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic *pCharacteristic) override { // 手机连接蓝牙改变该值
        std::string val = pCharacteristic->getValue();
        remoteSwitch = (unsigned char) val.c_str()[0];
    }
};

void IRAM_ATTR motorRoutine() {
    static unsigned char index = 0;
    static unsigned char steps[8][4] = {
            {1, 0, 0, 0},
            {1, 1, 0, 0},
            {0, 1, 0, 0},
            {0, 1, 1, 0},
            {0, 0, 1, 0},
            {0, 0, 1, 1},
            {0, 0, 0, 1},
            {1, 0, 0, 1},
    };
    motorRotationCnt++;
    if (motorRotationCnt >= MOTOR_ROTATE) { // 实现马达按需转动
        return;
    }
    index++;
    if (index >= 8) {
        index -= 8;
    }
    unsigned char trueIndex = output ? index : 7 - index; // 正向和反向转动
    digitalWrite(MOTOR_PIN1, steps[trueIndex][0]);
    digitalWrite(MOTOR_PIN2, steps[trueIndex][1]);
    digitalWrite(MOTOR_PIN3, steps[trueIndex][2]);
    digitalWrite(MOTOR_PIN4, steps[trueIndex][3]);
}

void IRAM_ATTR servoRoutine() {
    static unsigned char index = 0;
    static const unsigned char maxIndex = 30;
    index++;
    if (index >= maxIndex) {
        index = 0;
    }
//    if (millis() - servoEnableTime > SERVO_RESET_TIME) { // 一定时间后恢复平常状态
//        if (index >= SERVO_STATE_N) {
//            digitalWrite(MOTOR_PIN1, LOW);
//        } else {
//            digitalWrite(MOTOR_PIN1, HIGH);
//        }
//    } else if (output) {
//        if (index >= SERVO_STATE_O) {
//            digitalWrite(MOTOR_PIN1, LOW);
//        } else {
//            digitalWrite(MOTOR_PIN1, HIGH);
//        }
//    } else {
//        if (index >= SERVO_STATE_C) {
//            digitalWrite(MOTOR_PIN1, LOW);
//        } else {
//            digitalWrite(MOTOR_PIN1, HIGH);
//        }
//    }
    // for test:
    if (index >= remoteSwitch) {
        digitalWrite(MOTOR_PIN1, HIGH);
    } else {
        digitalWrite(MOTOR_PIN1, LOW);
    }
}

void enableServo() {
    servoEnableTime = millis();
}

/**
 * 初始化舵机
 * 注意不能和initMotor同时使用
 * */
void initServo() {
    pinMode(MOTOR_PIN1, OUTPUT);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, servoRoutine, true);
    timerAlarmWrite(timer, 100, true); // 0.1ms一格，3ms一个周期
    timerAlarmEnable(timer);
}

void initMotor() {
    pinMode(MOTOR_PIN1, OUTPUT);
    pinMode(MOTOR_PIN2, OUTPUT);
    pinMode(MOTOR_PIN3, OUTPUT);
    pinMode(MOTOR_PIN4, OUTPUT);

    timer = timerBegin(0, 80, true); // 第一个参数是定时器序号，第二个参数是分频系数（单片机频率除以该参数）（一般设置为80即可），第三个参数为是否向上计数
    timerAttachInterrupt(timer, motorRoutine, true); // 第三个参数是true: 边沿触发, false: 电平触发
    timerAlarmWrite(timer, 2000,
                    true); // 第二个参数是计时时间（单片机频率80MHz经过timerBegin第二个参数80后变成1MHz，也就是一秒即为1000000，这里写2000即为计时2毫秒），第三个参数是是否自动重载，即是否循环
    timerAlarmEnable(timer);
}

void initBLEScan() {
    BLEDevice::init("AutoDetectESP32");
    pScan = BLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->setInterval(150);
    pScan->setWindow(100);
}

void initBLEServer() {
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    switchCharacteristic = new BLECharacteristic(CHAR_UUID,
                                                 BLECharacteristic::PROPERTY_WRITE |
                                                 BLECharacteristic::PROPERTY_READ |
                                                 BLECharacteristic::PROPERTY_INDICATE |
                                                 BLECharacteristic::PROPERTY_NOTIFY);
    descriptor = new BLEDescriptor(DESCRIPTOR_UUID);

    switchCharacteristic->addDescriptor(descriptor);
    switchCharacteristic->setValue(remoteSwitch);
    switchCharacteristic->setCallbacks(new MyCharacteristicCallBack());
    pService->addCharacteristic(switchCharacteristic);
    pService->start();
    BLEAdvertising *ad = pServer->getAdvertising();
    ad->addServiceUUID(SERVICE_UUID);
    ad->setMinPreferred(0x06);  // 有助于解决iPhone连接问题的功能
    ad->setMinPreferred(0x12);
    pServer->setCallbacks(new MyConnectingCallBack());
    BLEDevice::startAdvertising();
    Serial.println("BLE server started");
}

void scan() {
    Serial.print("Scanning...");
    BLEScanResults rst = pScan->start(2);
    for (int i = 0; i < rst.getCount(); ++i) {
        if (rst.getDevice(i).haveServiceData()) {
            std::string data = rst.getDevice(i).getServiceData();
            for (const std::string &one: expected) {
                if (data.find(one) != -1) {
                    availablePair = true;
                    goto available; // 打断多层循环
                }
            }
        }
    }
    availablePair = false;
    available:
    Serial.print("Scan result: ");
    if (availablePair) {
        Serial.println("yes");
    } else {
        Serial.println("no");
    }
}

void startMotor() {
    motorRotationCnt = 0;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Serial start");
    initBLEScan();
    Serial.println("Init scan completed");
    initBLEServer();
    Serial.println("Init BLEServer completed");
//    initMotor();
//    Serial.println("Init motor completed");
    initServo();
    Serial.println("Init servo completed");
    pinMode(OUTPUT_LED_PIN, OUTPUT);
    digitalWrite(OUTPUT_LED_PIN, LOW);
    Serial.println("Init completed");
}


void loop() {
    scan();
    Serial.print("RemoteSwitch: ");
    Serial.println(remoteSwitch);
    bool rawOutput = output; // 用于判断马达是否要启动
    if (!availablePair) {
        unavailableCnt++;
        if (unavailableCnt >= MAX_UNAVAILABLE_CNT) {
            output = false;
            digitalWrite(OUTPUT_LED_PIN, LOW);
            remoteSwitch = remoteSwitch ? remoteSwitch : 1; // 重置远程开关为true
        }
    } else if (remoteSwitch) {
        unavailableCnt = 0;
        output = true;
        digitalWrite(OUTPUT_LED_PIN, HIGH);
    } else {
        unavailableCnt = 0;
        output = false;
        digitalWrite(OUTPUT_LED_PIN, LOW);
    }
    if (output != rawOutput) {
//        startMotor();
        enableServo();
    }
    Serial.print("Output: ");
    Serial.println(output);
}