#include "app.hpp"
#include "session.hpp"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <driver/usb_serial_jtag.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#if !defined(ARDUINO_ARCH_ESP32) || !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select ESP32S3 Dev Module (or the matching ESP32-S3 board)."
#endif
#if !defined(ARDUINO_USB_MODE) || (ARDUINO_USB_MODE != 1)
#error "Select Tools > USB Mode > Hardware CDC and JTAG."
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Enable Tools > USB CDC On Boot."
#endif
#if __cplusplus < 201703L
#error "C++17+ required; use Espressif ESP32 3.x board package."
#endif
#if !defined(__cpp_exceptions) && !defined(__EXCEPTIONS)
#error "Keep build_opt.h; C++ exceptions are required."
#endif
// Same native USB connector and CDC device. Do not run two USB RX ISRs.
// Set to 0 only for a compatibility comparison (algorithm/LZ4 stay enabled).
#ifndef ANOTO_USB_IDF
#define ANOTO_USB_IDF 0
#endif
SET_LOOP_TASK_STACK_SIZE(24 * 1024);
namespace {
wire::Parser parser;
bool idf_usb=false;
std::size_t rx_capacity=0;
constexpr std::size_t kUsbRxBytes=48*1024;
static_assert(kUsbRxBytes>wire::MAX_PAYLOAD+wire::HEADER,"RX must hold a full request");
char hardware[64]="ESP32-S3_NATIVE_USB_HWCDC";
std::uint64_t now_us(){return std::uint64_t(esp_timer_get_time());}
void memory(std::uint32_t& free,std::uint32_t& largest,std::uint32_t& low){
    const auto c=MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT;
    free=heap_caps_get_free_size(c);largest=heap_caps_get_largest_free_block(c);low=heap_caps_get_minimum_free_size(c);
}
bool write_all(const std::uint8_t* p,std::size_t n){
    const auto start=now_us();
    while(n){
        const auto size=std::min<std::size_t>(n,1024);
        const int count=idf_usb?usb_serial_jtag_write_bytes(p,size,pdMS_TO_TICKS(20)):
            int(Serial.write(p,size));
        if(count<0||std::size_t(count)>size)return false;
        if(count){p+=count;n-=std::size_t(count);}
        else{if(now_us()-start>=2000000ULL)return false;delay(1);}
    }
    return true;
}
void send(std::uint8_t type,std::uint32_t seq,const std::uint8_t* data,std::size_t n){
    std::uint8_t packet[wire::HEADER+1024];
    wire::header(packet,type,seq,data,n);
    if(n<=1024){if(n)std::memcpy(packet+wire::HEADER,data,n);write_all(packet,wire::HEADER+n);}
    else if(write_all(packet,wire::HEADER))write_all(data,n);
}
replay::Session session({now_us,memory,send,hardware});
void legacy_usb(){
    rx_capacity=Serial.setRxBufferSize(kUsbRxBytes);
    Serial.setTxBufferSize(2048);Serial.begin(115200);
    std::strcpy(hardware,"ESP32-S3_NATIVE_USB_HWCDC_FALLBACK");
}
}
void stream_setup(){
#if ANOTO_USB_IDF
    // Arduino can start HWCDC before setup(). Release it before installing the
    // packet/ring-buffer IDF driver. GPIO19/20 are the S3 native USB D-/D+ pins.
    // This handoff occurs ONCE at boot; there is no disconnect/reset per frame.
    Serial.end();
    gpio_reset_pin(GPIO_NUM_19);gpio_reset_pin(GPIO_NUM_20);
    usb_serial_jtag_driver_config_t config{};
    config.tx_buffer_size=2048;config.rx_buffer_size=kUsbRxBytes;
    const esp_err_t error=usb_serial_jtag_driver_install(&config);
    if(error==ESP_OK){
        idf_usb=true;rx_capacity=kUsbRxBytes;
        std::strcpy(hardware,"ESP32-S3_NATIVE_USB_IDF_RING");
    }else{
        // driver_install cleans up its partial allocation on failure.
        // Do not silently claim the IDF path is active if allocation failed.
        legacy_usb();
    }
#else
    legacy_usb();
#endif
    // No text is injected into a binary stream. HELLO reports firmware/transport.
}
void stream_loop(){
    const auto begin=now_us();
    if(parser.active()&&begin-parser.last_progress_us>10000000ULL){parser.reset();session.abort();}
    std::uint8_t buf[2048];
    int count=0;
    if(idf_usb){
        // Blocks only until bytes arrive, not until buf is full. Idle time is
        // available to FreeRTOS instead of a per-byte queue or a busy wait.
        count=usb_serial_jtag_read_bytes(buf,sizeof(buf),1);
    }else{
        const int available=Serial.available();
        if(available>0)count=int(Serial.read(buf,std::min<std::size_t>(sizeof(buf),std::size_t(available))));
    }
    if(count<=0||std::size_t(count)>sizeof(buf)){if(!idf_usb)delay(1);return;}
    std::size_t offset=0;
    while(offset<std::size_t(count)){
        std::size_t used=0;
        const bool ready=parser.consume(buf+offset,std::size_t(count)-offset,used,now_us());offset+=used;
        if(ready){
            if(parser.type()==wire::START&&rx_capacity<wire::MAX_PAYLOAD+wire::HEADER){
                const char error[]="USB_RX_BUFFER_ALLOCATION_FAILED";
                send(wire::ERROR,parser.seq(),reinterpret_cast<const std::uint8_t*>(error),sizeof(error)-1);
            }else session.handle(parser,now_us()-parser.payload_started_us);
        }
        if(!used)break;
    }
    yield();
}
