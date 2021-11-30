#include <Arduino.h>
#include <BLEDevice.h>

// #define DEBUG


#define MAX_BUFFER_LENGTH   64
#define PREFERED_MTU        MAX_BUFFER_LENGTH+3

#define MAX_IDLE_FLUSH_TIME 40
#define LED_BLINK_PERIOD    400


#ifdef DEBUG
#define dbg_print(x) Serial.print(x)
#define dbg_println(x) Serial.println(x)
#define dbg_printf(fmt,...) Serial.printf(fmt,##__VA_ARGS__)
#else
#define dbg_print(x) 
#define dbg_println(x) 
#define dbg_printf(fmt,...) 
#endif



static const BLEUUID NRF_NUS_SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static const BLEUUID NRF_NUS_RX_CHAR_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static const BLEUUID NRF_NUS_TX_CHAR_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

static uint64_t m_nusAddress;
static esp_ble_addr_type_t m_nusAddressType;
static bool m_do_connect;
static bool m_connected;

static char m_buffer[MAX_BUFFER_LENGTH+1];
static uint16_t m_idx;
static uint16_t m_mtu;
static bool     m_led;
static uint32_t m_last_blink_time;


class MyBLEConnectionCallback : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (m_nusAddress==0 && advertisedDevice.isAdvertisingService(NRF_NUS_SERVICE_UUID))
        {
            dbg_print("NUS Server found: ");
            dbg_print(advertisedDevice.toString().c_str());
            dbg_print(" ");
            dbg_println(advertisedDevice.getAddressType());
            BLEDevice::getScan()->stop();


            BLEAddress addr = advertisedDevice.getAddress();
            m_nusAddress = *(uint64_t *)(addr.getNative()) & 0x0000FFFFFFFFFFFF;
            m_nusAddressType = advertisedDevice.getAddressType();

            m_do_connect = true;
        }
    }
};

class MyBLEClientCallbacks : public BLEClientCallbacks {
public:
	void onConnect(BLEClient *pClient) override
    {
    };
	void onDisconnect(BLEClient *pClient) override
    {
        dbg_println("Disconnected");
        m_connected = false;
        m_led = LOW;
        m_last_blink_time = millis();
        digitalWrite(LED_BUILTIN, m_led);
    };
};


MyBLEConnectionCallback m_connectionCallback;
MyBLEClientCallbacks    m_clientCallbacks;
BLEClient               *m_pClient;
BLERemoteCharacteristic *m_pRXChar;
BLERemoteCharacteristic *m_pTXChar;


static void nusTXnotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify, void *param);


void setup()
{


    Serial.begin(115200);
    dbg_println("Starting NUS Client");

    m_led = LOW;
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, m_led);


    m_do_connect = false;
    m_connected = false;
    m_nusAddress = 1;

    BLEDevice::init("Friesh");
    BLEDevice::setMTU(PREFERED_MTU);


    m_pClient = BLEDevice::createClient();
    m_pClient->setClientCallbacks(&m_clientCallbacks);

    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(&m_connectionCallback);
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);

}


void loop() {
    static uint32_t last_read_time;

    if(m_do_connect)
    {
        m_do_connect = false;
        m_connected = false;
        bool success = false;
        m_led = HIGH;
        digitalWrite(LED_BUILTIN, m_led);
        dbg_printf("connecting to %02X:%02X:%02X:%02X:%02X:%02X\n",
                      (uint8_t)(m_nusAddress >> 40),
                      (uint8_t)(m_nusAddress >> 32),
                      (uint8_t)(m_nusAddress >> 24),
                      (uint8_t)(m_nusAddress >> 16),
                      (uint8_t)(m_nusAddress >> 8),
                      (uint8_t)(m_nusAddress));
        if(m_pClient->connect(BLEAddress((uint8_t*)&m_nusAddress), m_nusAddressType))
        {
            BLERemoteService* pService = m_pClient->getService(NRF_NUS_SERVICE_UUID);
            if(pService==nullptr)
            {
                m_pClient->disconnect();
                // TODO Disconnect process...
            } else {
                m_pRXChar = pService->getCharacteristic(NRF_NUS_RX_CHAR_UUID);
                m_pTXChar = pService->getCharacteristic(NRF_NUS_TX_CHAR_UUID);

                if(m_pTXChar!=nullptr && m_pRXChar != nullptr) {
                    m_pTXChar->registerForNotify(&nusTXnotifyCallback);
                    success = true;
                } else {
                    m_pClient->disconnect();
                }
            }
        }


        if(success) {
            m_connected = true;
            m_pClient->setMTU(PREFERED_MTU);
            m_mtu = m_pClient->getMTU()-3;
            m_idx = 0;
            dbg_printf("Connected [MTU=%d]\n", m_mtu);
        } else {
            m_led = LOW;
            m_last_blink_time = millis();
            digitalWrite(LED_BUILTIN, m_led);
            m_connected = false;
            dbg_println("Connection failed");
        }
    }

    if(m_connected) {
        while(Serial.available()) {
            last_read_time = millis();
            int c = Serial.read();
            m_buffer[m_idx++] = c;
            if(c=='\r' || c=='\n' || m_idx==m_mtu || m_idx==MAX_BUFFER_LENGTH) {
                m_pRXChar->writeValue((uint8_t*)m_buffer, m_idx, false);
                m_idx = 0;
            }
        }

        if(m_idx && millis()-last_read_time>MAX_IDLE_FLUSH_TIME)
        {
            m_pRXChar->writeValue((uint8_t*)m_buffer, m_idx, false);
            m_idx = 0;
        }

    } else {
        if(m_nusAddress) {
            m_nusAddress = 0;
            dbg_println("Start scanning...");
            delay(200);
            BLEDevice::getScan()->start(-1, nullptr, false);
        }

        if(millis() - m_last_blink_time > LED_BLINK_PERIOD) {
            m_last_blink_time = millis();
            m_led= !m_led;
            digitalWrite(LED_BUILTIN, m_led);
        }
    }




}

static void nusTXnotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify, void* param)
{
    while(length--) {
        Serial.print((char)*pData++);        
    }
}


