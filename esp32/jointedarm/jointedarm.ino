String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);           // match this baud in your Python code / Serial Monitor
  inputString.reserve(64);      // optional: reserve some space
  Serial.println("Ready.");
}

void loop() {
  // Read incoming characters
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    // End of line (from pressing Enter)
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        stringComplete = true;
      }
    } else {
      inputString += inChar;
    }
  }

  // When a full string is received, print it
  if (stringComplete) {
    Serial.print("Received: ");
    Serial.println(inputString);

    // clear for next line
    inputString = "";
    stringComplete = false;
  }
}
