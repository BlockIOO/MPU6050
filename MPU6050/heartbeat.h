String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

bool string_complete() {
  // clear the string:
  if (stringComplete) {
    inputString = "";
    stringComplete = false;
    return true;
  }
  return false;
}

void heartbeat_setup() {
  inputString.reserve(200);
}