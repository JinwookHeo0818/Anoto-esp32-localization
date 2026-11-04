#include "app.hpp"
#include "session.hpp"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#if !defined(ARDUINO_ARCH_ESP32) || !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select the ESP32S3 Dev Module (or a matching ESP32-S3 board)."
#endif
// Match the ESP32-C6 hardware USB CDC transport used by the Python PC client.
// TinyUSB (USB-OTG) uses a different Serial backend and is not this configuration.
#if !defined(ARDUINO_USB_MODE) || (ARDUINO_USB_MODE != 1)
#error "Select Tools > USB Mode > Hardware CDC and JTAG."
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Enable Tools > USB CDC On Boot for native USB data transfer."
#endif
#if __cplusplus < 201703L
#error "C++17+ required; use Espressif ESP32 3.x board package."
#endif
#if !defined(__cpp_exceptions) && !defined(__EXCEPTIONS)
#error "Keep build_opt.h; C++ exceptions are required."
#endif
SET_LOOP_TASK_STACK_SIZE(24 * 1024);
namespace {
wire::Parser parser;
std::size_t rx_capacity=0;
// HWCDC may drop bytes when its RX queue fills. Hold one complete wire packet.
constexpr std::size_t kUsbRxBytes=48*1024;
static_assert(kUsbRxBytes>wire::MAX_PAYLOAD+wire::HEADER,"RX must hold one full request");
std::uint64_t now_us(){return std::uint64_t(esp_timer_get_time());}
void memory(std::uint32_t& free,std::uint32_t& largest,std::uint32_t& low){auto c=MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT;free=heap_caps_get_free_size(c);largest=heap_caps_get_largest_free_block(c);low=heap_caps_get_minimum_free_size(c);}
bool write_all(const std::uint8_t* p,std::size_t n){auto t=millis();while(n){auto k=Serial.write(p,n);p+=k;n-=k;if(n){if(std::uint32_t(millis()-t)>5000)return false;delay(1);}}return true;}
void send(std::uint8_t type,std::uint32_t seq,const std::uint8_t* data,std::size_t n){std::uint8_t h[wire::HEADER];wire::header(h,type,seq,data,n);if(write_all(h,sizeof(h)))write_all(data,n);}
replay::Session session({now_us,memory,send,"ESP32-S3_NATIVE_USB"});
}
void stream_setup(){
    rx_capacity=Serial.setRxBufferSize(kUsbRxBytes);
    Serial.setTxBufferSize(2048);Serial.begin(115200);
    // No blocking wait for the host and no periodic text inside the binary stream.
    // A HELLO packet works even when this startup line was missed.
    if(Serial)Serial.println("ANOTO MCU v7.2 S3: close Serial Monitor, run START.bat --port COMx");
}
void stream_loop(){
    if(parser.active() && now_us()-parser.last_progress_us>10000000ULL){parser.reset();session.abort();}
    int available=Serial.available();if(available<=0){delay(1);return;}
    std::uint8_t buf[512];auto n=Serial.read(buf,std::min<std::size_t>(sizeof(buf),std::size_t(available)));
    std::size_t offset=0;
    while(offset<n){std::size_t used=0;bool ready=parser.consume(buf+offset,n-offset,used,now_us());offset+=used;
        if(ready){
            if(parser.type()==wire::START && rx_capacity<wire::MAX_PAYLOAD+wire::HEADER){
                const char error[]="USB_RX_BUFFER_ALLOCATION_FAILED: need 48 KiB receive queue";
                send(wire::ERROR,parser.seq(),reinterpret_cast<const std::uint8_t*>(error),sizeof(error)-1);
            }else{
                auto receive_us=now_us()-parser.payload_started_us;session.handle(parser,receive_us);
            }
        }
        if(!used)break;
    }
    // Cooperative yield between packets/chunks, outside the measured engine call.
    yield();
}
