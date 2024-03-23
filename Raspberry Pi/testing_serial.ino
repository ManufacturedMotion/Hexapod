const int bufferSize = 64;
char buffer[bufferSize];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial4.begin(115200);
}

void loop() {
  while (Serial4.available() > 0) {
    char receivedChar = Serial4.read();

    if (receivedChar == '\n') {
      // End of word, process the complete word
      buffer[bufferIndex] = '\0';  // Null-terminate the string
      Serial.print("Received from Raspberry Pi: ");
      Serial.println(buffer);

      // Reset bufferIndex for the next word
      bufferIndex = 0;
    } else {
      // Add the character to the buffer
      if (bufferIndex < bufferSize - 1) {
        buffer[bufferIndex++] = receivedChar;
      } else {
        // Buffer overflow, handle error or reset the buffer
        bufferIndex = 0;
      }
    }
  }
}
