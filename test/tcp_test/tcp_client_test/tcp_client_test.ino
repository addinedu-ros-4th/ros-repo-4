#include <WiFi.h>

const char* ssid = "AIE_509_2.4G";
const char* password = "addinedu_class1";

const char* hostip = "192.168.0.86";
const int hostport = 3000;

int count = 0;

WiFiClient client;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.connect(hostip, hostport);

}

void loop() {
  // put your main code here, to run repeatedly:
  

  // // READ --------------------------------------------------------------------------------------------
  // if (!client.connected()) {
  //   Serial.println("Disconnected");
  //   client.connect(hostip, hostport);
  // }
  // else
  // { 
  //   if (client.available() > 0)
  //   {
  //     String line = client.readStringUntil('\r');
  //     Serial.println(line);
  //   }
  // }

  //WRITE --------
  if (!client.connected()) {
    Serial.println("Disconnected");
    client.connect(hostip, hostport);
  }
  
  if (client.connected())
  { 
    Serial.println("connected");


    char b[5];

    String str;

    str=String(count);

    str.toCharArray(b,5);

    client.write(b);
    count ++;
  }



  delay(1000);
}
