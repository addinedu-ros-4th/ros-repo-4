#include <DHT_U.h>
#include <DHT.h>


#include <WiFi.h>
#include <WiFiClient.h>

#define DHTPIN 4        // DHT22 데이터 핀
#define DHTTYPE DHT11   // DHT22 타입

const char* ssid     = "AIE_509_2.4G";       // WiFi SSID
const char* password = "addinedu_class1";   // WiFi 패스워드
const char* host = "192.168.0.13";      // 서버 IP 주소
const uint16_t port = 3000;              // 서버 포트 번호

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  delay(10);

  // WiFi 연결
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  dht.begin();
}

void loop() {
  delay(2000); // 2초마다 데이터 전송

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // 센서 오류 체크
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // 서버에 데이터 전송
  Serial.print("Connecting to ");
  Serial.println(host);

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("Connection failed");
    return;
  }

  // 온습도 데이터를 문자열로 변환
  String data = "Temperature: " + String(t) + " *C, Humidity: " + String(h) + " %";

  // 데이터 전송
  client.println(data);
  Serial.println("Data sent to server:");
  Serial.println(data);

  // 연결 종료
  client.stop();
}
