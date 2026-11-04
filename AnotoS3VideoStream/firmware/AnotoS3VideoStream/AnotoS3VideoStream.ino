// Anoto MCU v7.2 for ESP32-S3: PC sends prepared grayscale frames over native USB.
// Open this sketch in Arduino IDE; leave build_opt.h and src/ in the same folder.
#include "src/app.hpp"
void setup() { stream_setup(); }
void loop() { stream_loop(); }
