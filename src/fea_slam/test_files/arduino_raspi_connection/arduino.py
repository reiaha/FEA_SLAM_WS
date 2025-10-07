void setup() {
  Serial.begin(115200);  // Start USB serial
}

void loop() {
  // If data received from Pi, echo it back
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    Serial.print("Arduino received: ");
    Serial.println(msg);
  }
  
  // Optional: send a heartbeat to Pi
  Serial.println("Arduino is alive");
  delay(1000);
}
