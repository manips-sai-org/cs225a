// note our esp board is a esp32 dev kit
#include <esp_now.h>
#include <WiFi.h>
// Red MAC address 1 30:C6:F7:23:94:44
// Green MAC Address 2  40:91:51:9F:02:E0

uint8_t receiver[] = {0x30, 0xC6, 0xF7, 0x23, 0x94, 0x44};

typedef struct payload {
  int x;
} payload;

payload test;

int last_value = -1;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  pinMode(23, INPUT_PULLUP);
  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, receiver, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  int value = digitalRead(23);
  if(value == last_value || last_value == -1){
    delay(1000);
  }
  else{
    delay(100);
  }
  last_value = value;
  if(value == 0){
    test.x = 23;
  }
  else{
    test.x = 45;
  }
  
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(payload));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}
