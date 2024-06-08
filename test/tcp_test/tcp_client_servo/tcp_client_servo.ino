#include <WiFi.h>
#include <ESP32Servo.h>

const char* ssid = "AIE_509_2.4G";
const char* password = "addinedu_class1";

const char* hostip = "192.168.0.47";
const int hostport = 3002;

int count = 0;

WiFiClient client;

Servo servo;


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

  servo.attach(5);
  servo.write(60); //FoodTank default close
}

void loop() {
  // put your main code here, to run repeatedly:
  
  String target, op_cl;

  // READ --------------------------------------------------------------------------------------------
  if (!client.connected()) {
    Serial.println("Disconnected");
    client.connect(hostip, hostport);
  }
  else
  { 
    if (client.available() > 0)
    {
      String data = client.readStringUntil('\r');
      Serial.println(data);

      int startIndex = 0;
      int endIndex = data.indexOf(',');
      
      target = data.substring(startIndex, endIndex);
      Serial.print("1 :");
      Serial.println(target);

      startIndex = endIndex + 1;
      endIndex = data.indexOf(',', startIndex);

      op_cl = data.substring(startIndex, endIndex);

      Serial.print("2 :");
      Serial.println(op_cl);

    }
  }

  // //WRITE --------
  // if (!client.connected()) {
  //   Serial.println("Disconnected");
  //   client.connect(hostip, hostport);
  // }
  
  // if (client.connected())
  // { 
  //   Serial.println("connected");


  //   char b[5];

  //   String str;

  //   str=String(count);

  //   str.toCharArray(b,5);

  //   client.write(b);
  //   count ++;
  // }

 
  // SErvo Move
  if(op_cl == "0") 
  {
    Serial.println("0");
    servo.write(60); // FoodTank close
  }
  else if (op_cl == "1") 
  {
    Serial.println("1");
    servo.write(180); // FoodTank open
  }


  delay(1000);
}
