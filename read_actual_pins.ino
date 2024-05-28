void setup() {
      Serial.begin(9600);
    }
    
    void loop() {
      Serial.print("MOSI: ");
      Serial.println(MOSI);
      Serial.print("MISO: ");
      Serial.println(MISO);
      Serial.print(" SCK: ");
      Serial.println(SCK);
      Serial.print("  SS: ");
      Serial.println(SS);
      Serial.println();
      delay(5000);
    }
