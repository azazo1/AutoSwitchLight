//
// Created by azazo1 on 2024/2/2.
//
#include <Arduino.h>
#include <BLEDevice.h>

#define SERVICE_UUID "0000000000084180"
#define CHAR_UUID "0000000000084181"
#define DESCRIPTOR_UUID "0000000000084182"
#define MAX_UNAVAILABLE_CNT 10 // 关灯前扫描无果忍耐次数

const std::vector<std::string> expected = {
        "iphone 4s",
//        "ipad air"
};
BLEScan *pScan = nullptr;
BLECharacteristic *switchCharacteristic = nullptr;
BLEDescriptor *descriptor = nullptr;
bool availablePair = false;
int remoteSwitch = 1;
// 多次检测availablePair为false时才关闭，而一次为true则立马开启
unsigned int unavailableCnt = 0;
// 最终的开关灯决断
bool output = false;

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

void setup() {
    Serial.begin(115200);
    initBLEScan();
    initBLEServer();
}


void loop() {
    scan();
    Serial.print("RemoteSwitch: ");
    Serial.println(remoteSwitch);
    if (!availablePair) {
        unavailableCnt++;
        if (unavailableCnt >= MAX_UNAVAILABLE_CNT) {
            output = false;
            remoteSwitch = 1; // 重置远程开关
        }
    } else if (remoteSwitch) {
        unavailableCnt = 0;
        output = true;
    } else {
        unavailableCnt = 0;
        output = false;
    }
    Serial.print("Output: ");
    Serial.println(output);
}