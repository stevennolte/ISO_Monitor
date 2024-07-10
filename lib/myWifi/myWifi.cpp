#include <WiFi.h>
#include "Arduino.h"
#include "myWifi.h"

MyWifi::MyWifi()
{
    
};

bool MyWifi::connect(){
    // IPAddress local_IP(192, 168, 0, getAddress());
    IPAddress local_IP(address_1, address_2, address_3, address_4);
    IPAddress gateway(address_1, address_2, address_3, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    int n = WiFi.scanNetworks();
    for (int i=0; i<min(n,25); i++){
        for (int si=0; si<3;si++){
          
          if (WiFi.SSID(i) == ssids[si]){
            ssid = ssids[si];
            password = passwords[si];
          }
        }  
    }
    if (ssid == ""){
        Serial.println("No WiFi Found, REBOOTING");
        ESP.restart();
    }
    
    if (!WiFi.config(local_IP, gateway, subnet)) {
              Serial.println("STA Failed to configure");
    }
    WiFi.mode(WIFI_AP);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        delay(1000);
    }
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
}