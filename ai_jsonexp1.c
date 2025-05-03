#include <Arduino.h>
#include <ArduinoJson.h>

// Your JSON data
const char* jsonInput = R"({
  "0":[0,0,0,0,0,80,1000,5000,500,1,1,1746233357],
  "1":[0,0,0,0,0,80,1000,5000,500,1,1,1746233357],
  "2":[0,0,0,0,0,80,1000,5000,500,1,1,1746233357],
  "3":[0,0,0,0,0,80,1000,5000,500,1,1,1746233357],
  "4":[0,0,0,0,0,80,1000,5000,500,1,1,1746233357],
  "5":[0,0,0,0,0,80,1000,5000,500,1,1,1746233357],
  "6":[0,0,0,0,0,80,1000,5000,500,1,1,1746233357],
  "7":[0,0,0,0,0,80,1000,5000,500,1,1,1746233357]
})";

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect (needed for some boards)

  Serial.println("Starting JSON Parsing...");

  // Allocate the JsonDocument (use https://arduinojson.org/v6/assistant/ for sizing)
  StaticJsonDocument doc; // Increased buffer slightly from recommendation

  // Deserialize the JSON input
  DeserializationError error = deserializeJson(doc, jsonInput);

  // Check for parsing errors
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return; // Don't continue if parsing failed
  }

  Serial.println("JSON Parsing Successful!");
  Serial.println("---");

  // --- Example 1: Access specific data ---
  Serial.println("Accessing specific data (key \"2\"):");
  JsonArray array2 = doc["2"]; // Get the array for key "2"

  if (array2) {
      // Access elements using indices. Use appropriate data types.
      int val_2_0 = array2[0];
      int val_2_5 = array2[5];
      // Timestamps can be large, use unsigned long
      unsigned long timestamp_2 = array2[11];

      Serial.print("  Element 0: "); Serial.println(val_2_0);
      Serial.print("  Element 5: "); Serial.println(val_2_5);
      Serial.print("  Timestamp (Element 11): "); Serial.println(timestamp_2);
  } else {
      Serial.println("  Key \"2\" not found or not an array.");
  }
  Serial.println("---");


  // --- Example 2: Iterate through all keys ---
  Serial.println("Iterating through all keys (\"0\" to \"7\"):");

  for (int i = 0; i <= 7; i++) {
    String currentKey = String(i); // Keys are strings: "0", "1", ...

    // Check if key exists before accessing
    if (doc.containsKey(currentKey)) {
      JsonArray currentArray = doc[currentKey];

      Serial.print("Key \""); Serial.print(currentKey); Serial.println("\":");

      // Access specific elements by index if needed
      int elementAtIndex5 = currentArray[5]; // Should be 80
      unsigned long timestamp = currentArray[11]; // Last element

      Serial.print("  Element at index 5: "); Serial.println(elementAtIndex5);
      Serial.print("  Timestamp (index 11): "); Serial.println(timestamp);

      // Optional: Loop through all values in this specific array
      // Serial.print("  All values: ");
      // for(JsonVariant value : currentArray) {
      //    Serial.print(value.as<long>()); // Cast to expected type
      //    Serial.print(" ");
      // }
      // Serial.println(); // Newline after printing all values

    } else {
       Serial.print("Key \""); Serial.print(currentKey); Serial.println("\" not found.");
    }
  }
  Serial.println("---");
  Serial.println("Finished Processing.");
}

void loop() {
  // Nothing needed here for this example
  delay(5000);
}