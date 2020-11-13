#include <esp32cam.h>
#include <WebServer.h>
#include <WiFi.h>
#include "sensor.h"
#include "esp_camera.h"

const char* WIFI_SSID = "***********";
const char* WIFI_PASS = "***********";

WebServer server(80);

static auto Res = esp32cam::Resolution::find(640 , 480); // VGA Format

void serveJpg()
{
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("CAPTURE FAIL");
    server.send(503, "", "");
    return;
  }

  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}


void imageHandler()
{
  if (!esp32cam::Camera.changeResolution(Res)) {
    Serial.println("SETTING-RES FAIL");
  }
  serveJpg();
}


void setup()
{
  Serial.begin(115200);
  Serial.println();

  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(Res);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/Img.jpg");
  Serial.println();

  server.on("/Img.jpg", imageHandler);
  server.begin();
}

void loop()
{
  server.handleClient();
}
