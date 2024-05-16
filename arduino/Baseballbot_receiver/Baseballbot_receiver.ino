// note our esp board is a esp32 dev kit
#include <esp_now.h>
#include <WiFi.h>
// Red MAC receiver address 1 30:C6:F7:23:94:44
// Green MAC glove Address 2  40:91:51:9F:02:E0

uint8_t glove[] = {0x40, 0x91, 0x51, 0x9F, 0x02, 0xE0};

//Structure example to receive data
//Must match the sender structure
typedef struct payload {
  int x;
} payload;

//Create a struct_message called data
payload data;

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.println(data.x);
  Serial.println();
  if(data.x == 23){
    digitalWrite(23, HIGH);
  }
  else{
    digitalWrite(23, LOW);
  }
}
 
void setup() {
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

}
