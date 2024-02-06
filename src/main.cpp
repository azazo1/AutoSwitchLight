//
// Created by azazo1 on 2024/2/2.
//
#include <Arduino.h>
#include <BLEDevice.h>
#include <EEPROM.h>
// 我的单片机蓝牙地址为 A0:A3:B3:29:D4:92
#define SERVICE_UUID "9f867bb8-d8a6-43b2-b129-fefc96fcf689"
#define CHAR_UUID "7c6473b1-0164-4842-be7f-d1cc945eaae6"
#define DESCRIPTOR_UUID "08fc14c5-2002-4a9d-8672-15517c70697a"
#define UNAVAILABLE_ENDURANCE 3000 // 失去期望设备的关灯前忍受时间(ms, 超过该时间后自动关灯)

#define COMMAND_ENTER_FREE_ROTATION 0xef // 命令: 进入自由旋转模式
#define COMMAND_ENTER_NORMAL 0xee // 命令: 进入正常模式
#define COMMAND_MODIFY_ON 0xa0 // 命令: 修改开灯角度系数
#define COMMAND_MODIFY_OFF 0xa1 // 命令: 修改关灯角度系数
#define COMMAND_MODIFY_PLAIN 0xa2 // 命令: 修改待机角度系数
#define COMMAND_SET_REMOTE_SWITCH 0xb0 // 命令: 设置RemoteSwitch的值

#define STATE_FREE_ROTATION 1
#define STATE_NORMAL 0

#define SERVO_RESET_TIME 300 // 舵机开关灯的持续时间（毫秒）
#define OUTPUT_LED_PIN 23 // 开关灯状态输出引脚，记得LED要串联一个电阻，不然烧坏
#define SERVO_SIGNAL_PIN 32 // 舵机信号引脚
// EEPROM中的储存地址
#define ADDRESS_OFF 0
#define ADDRESS_ON 1
#define ADDRESS_PLAIN 2

const std::vector<std::string> expected = { // 期望设备
        "iphone 4s",
//        "ipad air"
};
BLEScan *pScan = nullptr;
BLECharacteristic *commandCharacteristic = nullptr;
BLEDescriptor *remoteSwitchDescriptor = nullptr;

hw_timer_t *servoTimer = nullptr;
hw_timer_t *mainTimer = nullptr;

bool availablePair = false; // 是否扫描到期望的设备
unsigned char remoteSwitch = 1; // 在正常模式下用于手机遥控开关灯, 在自由旋转模式下, 作为舵机旋转系数, 用于旋转舵机
unsigned long firstUnavailableTime = 0; // 第一次失去所有期望设备时间
bool output = false; // 最终的开关灯决断
unsigned long servoEnableTime = 0; // 舵机进入开关灯状态的初始时间
unsigned char chipState = STATE_NORMAL; // 单片机状态: 0: 正常模式, 1: 自由旋转模式

// sg90 控制舵机角度的值（占PWM周期的30份之n）https://blog.csdn.net/weixin_43866583/article/details/129741864
// 以下值在 initServo 时在 EEPROM 中被读取, 默认值将没啥用处
unsigned char servoRotateOff = 0; // 关灯状态旋转系数
unsigned char servoRotateOn = 0; // 开灯状态旋转系数
unsigned char servoRotatePlain = 0; // 待机状态旋转系数
unsigned char servoCurrentRotate = 0; // 当前旋转系数

void enterFreeRotation() {
    chipState = STATE_FREE_ROTATION;
    timerAlarmWrite(mainTimer, 50 * 1000, true);
    Serial.println("Enter state: free rotation");
}

void enterNormal() {
    chipState = STATE_NORMAL;
    timerAlarmWrite(mainTimer, SERVO_RESET_TIME * 1000, true);
    Serial.println("Enter state: normal");
}

class MyConnectingCallBack : public BLEServerCallbacks {
public:
    void onConnect(BLEServer *pServer) override {
        Serial.println("\nServer:connected");
    }

    void onDisconnect(BLEServer *pServer) override {
        Serial.println("\nServer:disconnected");
        BLEDevice::startAdvertising();
        // 断开连接时进入正常模式
        enterNormal();
    }
};


class MyCharacteristicCallBack : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic *pCharacteristic) override { // 手机连接蓝牙改变该值
        std::string val = pCharacteristic->getValue();
        auto code = (unsigned char) val.c_str()[0];
        auto value = (unsigned char) val.c_str()[1];
        switch (code) {
            case COMMAND_ENTER_FREE_ROTATION: {
                enterFreeRotation();
                break;
            }
            case COMMAND_ENTER_NORMAL: {
                enterNormal();
                break;
            }
            case COMMAND_MODIFY_OFF: {
                servoRotateOff = value;
                EEPROM.writeUChar(ADDRESS_OFF, value);
                delay(1);
                EEPROM.commit();
                Serial.printf("Saved: off=%d\n", value);
                break;
            }
            case COMMAND_MODIFY_ON: {
                servoRotateOn = value;
                EEPROM.writeUChar(ADDRESS_ON, value);
                delay(1);
                EEPROM.commit();
                Serial.printf("Saved: on=%d\n", value);
                break;
            }
            case COMMAND_MODIFY_PLAIN: {
                servoRotatePlain = value;
                EEPROM.writeUChar(ADDRESS_PLAIN, value);
                delay(1);
                EEPROM.commit();
                Serial.printf("Saved: plain=%d\n", value);
                break;
            }
            case COMMAND_SET_REMOTE_SWITCH: {
                remoteSwitch = value;
                Serial.printf("Set: remoteSwitch=%d\n", value);
                break;
            }
            default: {
                Serial.printf("Received unknown command code: %d\n", code);
            }
        }
    }
};


void IRAM_ATTR servoRoutine() {
    static unsigned char index = 0;
    static const unsigned char maxIndex = 25;
    index++;
    if (index >= maxIndex) {
        index = 0;
    }
    if (chipState == STATE_NORMAL) {
        if (millis() - servoEnableTime > SERVO_RESET_TIME) { // 一定时间后恢复平常状态
            servoCurrentRotate = servoRotatePlain;
        }
    }
    if (index >= servoCurrentRotate) {
        digitalWrite(SERVO_SIGNAL_PIN, LOW);
    } else {
        digitalWrite(SERVO_SIGNAL_PIN, HIGH);
    }
}

void enableServo() {
    servoEnableTime = millis();
}

void turnOffLight() {
    servoCurrentRotate = servoRotateOff;
}

void turnOnLight() {
    servoCurrentRotate = servoRotateOn;
}

/**
 * 初始化舵机
 * */
void initServo() {
    pinMode(SERVO_SIGNAL_PIN, OUTPUT);

    EEPROM.begin(4);
    servoRotateOff = EEPROM.readUChar(ADDRESS_OFF);
    servoRotateOn = EEPROM.readUChar(ADDRESS_ON);
    servoRotatePlain = EEPROM.readUChar(ADDRESS_PLAIN);
    servoCurrentRotate = servoRotatePlain;
    Serial.printf("Read: off=%d, on=%d, plain=%d\n", servoRotateOff, servoRotateOn, servoRotatePlain);

    servoTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(servoTimer, servoRoutine, true);
    timerAlarmWrite(servoTimer, 100, true); // 0.1ms一格，2.5ms一个周期
    timerAlarmEnable(servoTimer);
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
    commandCharacteristic = new BLECharacteristic(CHAR_UUID,
                                                  BLECharacteristic::PROPERTY_WRITE |
                                                  BLECharacteristic::PROPERTY_READ |
                                                  BLECharacteristic::PROPERTY_INDICATE |
                                                  BLECharacteristic::PROPERTY_NOTIFY);
    remoteSwitchDescriptor = new BLEDescriptor(DESCRIPTOR_UUID);

    commandCharacteristic->addDescriptor(remoteSwitchDescriptor);
    remoteSwitchDescriptor->setValue(&remoteSwitch, 1);
    commandCharacteristic->setCallbacks(new MyCharacteristicCallBack());
    pService->addCharacteristic(commandCharacteristic);
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
    bool rawPair = availablePair;
    BLEScanResults rst = pScan->start(2);
    for (int i = 0; i < rst.getCount(); ++i) {
        if (rst.getDevice(i).haveServiceData()) {
            std::string data = rst.getDevice(i).getServiceData();
            for (const std::string &one: expected) {
                if (rst.getDevice(i).getName().find(one) != -1 || data.find(one) != -1) {
                    availablePair = true;
                    goto available; // 打断多层循环
                }
            }
        }
    }
    availablePair = false;
    available:
    if (availablePair != rawPair) {
        if (availablePair) {
            Serial.println("Scan: yes");
        } else {
            Serial.println("Scan: no");
        }
    }
}

void myRoutine() {
    if (chipState == STATE_NORMAL) {
        bool rawOutput = output; // 用于判断舵机是否要启动
        if (!availablePair) { // 没有可用设备
            if (millis() - firstUnavailableTime > UNAVAILABLE_ENDURANCE) {
                output = false;
                digitalWrite(OUTPUT_LED_PIN, LOW);
                remoteSwitch = remoteSwitch ? remoteSwitch : 1; // 重置远程开关为true
                commandCharacteristic->notify();
            }
        } else if (remoteSwitch) { // 有期望设备, 并且远程开关设置为true
            firstUnavailableTime = millis();
            output = true;
            digitalWrite(OUTPUT_LED_PIN, HIGH);
        } else { // 有期望设备, 但是远程开关被设置为false
            firstUnavailableTime = millis();
            output = false;
            digitalWrite(OUTPUT_LED_PIN, LOW);
        }
        if (output != rawOutput) {
            if (output) {
                turnOnLight();
            } else {
                turnOffLight();
            }
            enableServo(); // 启动舵机
            Serial.print("Output: ");
            Serial.println(output);
        }
    } else if (chipState == STATE_FREE_ROTATION) {
        servoCurrentRotate = remoteSwitch;
        enableServo();
    }
}

void initMyRoutine() {
    mainTimer = timerBegin(1, 80, true);
    timerAttachInterrupt(mainTimer, myRoutine, true);
    timerAlarmWrite(mainTimer, SERVO_RESET_TIME * 1000, true);
    timerAlarmEnable(mainTimer);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Serial start");
    initBLEScan();
    Serial.println("Init scan completed");
    initBLEServer();
    Serial.println("Init BLEServer completed");
    initServo();
    Serial.println("Init servo completed");
    pinMode(OUTPUT_LED_PIN, OUTPUT);
    digitalWrite(OUTPUT_LED_PIN, LOW);
    Serial.println("Init completed");
    initMyRoutine();
}


void loop() {
    scan();
}