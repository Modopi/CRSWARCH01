#include <SoftwareSerial.h>


//zs 040 호환보드(빨간거만 들어오는거)
SoftwareSerial hc06(2, 3);

void setup() {
  Serial.begin(9600);
  hc06.begin(9600);
  Serial.println("Master ready");
}

void loop() {
  // 블루투스로 들어온 한 줄 읽기
  if (hc06.available()) {
    String line = hc06.readStringUntil('\n');
    line.trim();

    // CSV 파싱: ax,ay,az,gx,gy,gz,temp
    float vals[7];
    int idx = 0;
    int start = 0;
    for (int i = 0; i <= (int)line.length() && idx < 7; i++) {
      if (i == (int)line.length() || line.charAt(i) == ',') {
        vals[idx++] = line.substring(start, i).toFloat();
        start = i + 1;
      }
    }

    if (idx == 7) {
      Serial.print("Accel(g): ");
      Serial.print(vals[0], 3); Serial.print(", ");
      Serial.print(vals[1], 3); Serial.print(", ");
      Serial.println(vals[2], 3);
      Serial.print("Gyro(°/s): ");
      Serial.print(vals[3], 2); Serial.print(", ");
      Serial.print(vals[4], 2); Serial.print(", ");
      Serial.println(vals[5], 2);
      Serial.print("Temp: ");
      Serial.print(vals[6], 1);
      Serial.println(" °C");
      Serial.println("---");
    }
  }
}
