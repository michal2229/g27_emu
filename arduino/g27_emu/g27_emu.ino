#include <stdint.h>
#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"
#include <TinyMatrixMath.hpp>

USBCDC USBSerial;

/*
  ## VID/PID
    046d  Logitech, Inc.
    c298  Driving Force Pro
    c299  G25 Racing Wheel
    c29b  G27 Racing Wheel
    c24f  G29 Driving Force Racing Wheel [PS3]
    c260  G29 Driving Force Racing Wheel [PS4]
*/

#define DEV_VID (0x046d)
#define DEV_PID (0xc29b)
#define DEV_PRODUCT_NAME "G27_emu"
#define DEV_MANUFACTURER_NAME "MB"
#define DEV_REPORT_ID (0)
#define DEV_REPORT_SIZE (11)
#define DEV_FFB_REQUEST_SIZE (7)


#define PIN_ADC_WHEEL 7  // potentiometer measurement pin, two others on GND and 3V3
#define PIN_FFB_EN10 35  // motor H-bridge enable pin (pwm speed control)
#define PIN_FFB_IN11 33  // motor H-bridge in1 pin
#define PIN_FFB_IN12 37  // motor H-bridge in2 pin

#define AXIS_WHEEL_ADC_RANGE 0x0500  // about 900deg in this case
#define AXIS_WHEEL_ADC_MIN 0x0500  
#define AXIS_WHEEL_ADC_CENTER (AXIS_WHEEL_ADC_MIN + AXIS_WHEEL_ADC_RANGE >> 1)
#define AXIS_WHEEL_ADC_MAX (AXIS_WHEEL_ADC_MIN + AXIS_WHEEL_ADC_RANGE)


#define KALMAN_DIM 3  // angle, angular velocity, angular acceleration
#define DT_REPORT_MS 2  // HID report time period
#define DT_MEASURE_US 800  // based on loop() iteration time measurements for ESP32-S2 @ 240MHz CPU, about worst case


// ffb commands according to Logitech Force Feedback Protocol V1.6
enum class EnumFfbCmd {
  DOWNLOAD_FORCE = 0x00,
  DOWNLOAD_AND_PLAY_FORCE = 0x01,
  PLAY_FORCE = 0x02,
  STOP_FORCE = 0x03,
  DEFAULT_SPRING_ON = 0x04,
  DEFAULT_SPRING_OFF = 0x05,
  REFRESH_FORCE = 0x0c,
  SET_DEFAULT_SPRING = 0x0e,
};


// ffb force types according to Logitech Force Feedback Protocol V1.6
enum class EnumForceType {
  CONSTANT = 0x00,
  SPRING = 0x01,
  DAMPER = 0x02,
  AUTO_CNT_SPRING = 0x03,
  SAWTOOTH_UP = 0x04,
  SAWTOOTH_DN = 0x05,
  TRAPEZOID = 0x06,
  RECTANGLE = 0x07,
  VARIABLE = 0x08,
  RAMP = 0x09,
  SQUARE_WAVE = 0x0a,
  HI_RES_SPRING = 0x0b,
  HI_RES_DAMPER = 0x0c,
  HI_RES_AUTO_CNT_SPRING = 0x0d,
  FRICTION = 0x0e
};


// HID descriptor compatible with G27
static const uint8_t hid_report_descriptor[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x04,        // Usage (Joystick)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x00,        //   Logical minimum (0)
  0x25, 0x07,        //   Logical maximum (7)
  0x35, 0x00,        //   Physical minimum (0)
  0x46, 0x3B, 0x01,  //   Physical maximum (315)
  0x65, 0x14,        //   Unit (20)
  0x09, 0x39,        //   Usage (Hat switch)
  0x75, 0x04,        //   Report size (4)
  0x95, 0x01,        //   Report count (1)
  0x81, 0x42,        //   Item
  0x65, 0x00,        //   Unit (0)
  0x25, 0x01,        //   Logical maximum (1)
  0x45, 0x01,        //   Physical maximum (1)
  0x05, 0x09,        //   Usage Page (Button)
  0x19, 0x01,        //   Usage Minimum (Button 1)
  0x29, 0x16,        //   Usage Maximum (Button 22)
  0x75, 0x01,        //   Report size (1)
  0x95, 0x16,        //   Report count (22)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x3F,  //   Logical maximum (16383)
  0x46, 0xFF, 0x3F,  //   Physical maximum (16383)
  0x75, 0x0E,        //   Report size (14)
  0x95, 0x01,        //   Report count (1)
  0x05, 0x01,        //   Usage Page (Generic Desktop)
  0x09, 0x30,        //   Usage (X)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x00,  //   Logical maximum (255)
  0x46, 0xFF, 0x00,  //   Physical maximum (255)
  0x75, 0x08,        //   Report size (8)
  0x95, 0x03,        //   Report count (3)
  0x09, 0x32,        //   Usage (Z)
  0x09, 0x35,        //   Usage (Rz)
  0x09, 0x31,        //   Usage (Y)
  0x81, 0x02,        //   Item
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x01,        //   Usage (Vendor-defined {ff00:1))
  0x95, 0x02,        //   Report count (2)
  0x81, 0x02,        //   Item
  0x95, 0x01,        //   Report count (1)
  0x75, 0x01,        //   Report size (1)
  0x25, 0x01,        //   Logical maximum (1)
  0x45, 0x01,        //   Physical maximum (1)
  0x05, 0x09,        //   Usage Page (Button)
  0x09, 0x17,        //   Usage (Button 23)
  0x81, 0x02,        //   Item
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x01,        //   Usage (Vendor-defined {ff00:1))
  0x95, 0x07,        //   Report count (7)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x00,  //   Logical maximum (255)
  0x46, 0xFF, 0x00,  //   Physical maximum (255)
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x02,        //   Usage (Vendor-defined {ff00:2))
  0x95, 0x07,        //   Report count (7)
  0x75, 0x08,        //   Report size (8)
  0x91, 0x02,        //   Item
  0x95, 0x90,        //   Report count (144)
  0x09, 0x03,        //   Usage (Vendor-defined {ff00:3))
  0xB1, 0x02,        //   Item
  0xC0,              // End Collection
};


// union for serialisation/deserialisation
union FfbForceType {
  uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  struct f00_constant_t {
    uint8_t type;
    uint8_t f0;
    uint8_t f1;
    uint8_t f2;
    uint8_t f3;
    uint8_t zero;
  } constant;
  // TODO complete list
};


// data from host - force feedback request
// union for serialisation/deserialisation
union FfbRequest {
  uint8_t bytes[DEV_FFB_REQUEST_SIZE];

  struct cmd00_download_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } download_force;

  struct cmd01_download_and_play_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } download_and_play_force;

  struct cmd02_play_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } play_force;

  struct cmd03_stop_force_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } stop_force;

  struct cmd04_default_spring_on_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } default_spring_on;

  struct cmd05_default_spring_off_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } default_spring_off;

  struct cmd0c_refresh_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } refresh_force;

  struct cmd0e_set_default_spring_t {
    uint8_t cmd;
    uint8_t zero;
    uint8_t k1;
    uint8_t k2;
    uint8_t clip;
    uint8_t zeros[2];
  } set_default_spring;
};


// data to send as HID report to host
// union for serialisation/deserialisation
union WheelStatus {
  uint8_t bytes[11];

  struct Status_t {
    uint8_t buttons_0;
    uint8_t buttons_1;
    uint8_t buttons_2;
    uint8_t axis_wheel_lsb6_and_btns2;
    uint8_t axis_wheel_msb;
    uint8_t axis_throttle;
    uint8_t axis_brake;
    uint8_t axis_clutch;
    uint8_t vendor_specific[3];  // not sure what it is, but probably not important

    // center_zero == true : v range -1 .. 1
    // center_zero == false : v range 0 .. 1
    void set_axis_wheel_float(float v, bool center_zero) {
      if (center_zero) v = (v + 1) / 2.0f;
      if (v < 0.0f) v = 0.0f;
      if (v > 1.0f) v = 1.0f;

      v = v * 0x3fff;

      set_axis_wheel_14bit((uint16_t)v);
    }

    void set_axis_wheel_14bit(uint16_t v) {
      axis_wheel_lsb6_and_btns2 = (v & 0x003f) << 2;
      axis_wheel_msb = (v >> 6) & 0x00ff;
    }

    uint16_t get_axis_wheel_14bit() {
      uint16_t v = 0x0000;
      v = (axis_wheel_lsb6_and_btns2 >> 2) & 0x003f;
      v |= (((uint16_t)axis_wheel_msb) << 6) & 0x3fc0;
      return v;
    }
  } status;
};


// computing forces to apply to the wheel ffb motor
class FfbController {
public:
  FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range)
    : force_current(0x7f),
      ffb_forces_en{ 0, 0, 0, 0 } {
    axis_wheel_cnt = axis_wheel_center;
    axis_wheel_min = axis_wheel_center - axis_wheel_range / 2;
    axis_wheel_max = axis_wheel_center + axis_wheel_range / 2;
  }

  void apply_force(const FfbRequest &req) {
    EnumFfbCmd f_cmd = (EnumFfbCmd)(req.bytes[0] & 0x0f);

    // force slots
    bool f0_en = req.bytes[0] & 0x10;
    bool f1_en = req.bytes[0] & 0x20;
    bool f2_en = req.bytes[0] & 0x40;
    bool f3_en = req.bytes[0] & 0x80;

    if (f_cmd == EnumFfbCmd::DOWNLOAD_FORCE) {
      if (f0_en) {
        ffb_forces[0] = req.download_force.force_type;
        ffb_forces_en[0] = false;
      }
      if (f1_en) {
        ffb_forces[1] = req.download_force.force_type;
        ffb_forces_en[1] = false;
      }
      if (f2_en) {
        ffb_forces[2] = req.download_force.force_type;
        ffb_forces_en[2] = false;
      }
      if (f3_en) {
        ffb_forces[3] = req.download_force.force_type;
        ffb_forces_en[3] = false;
      }
    } else if (f_cmd == EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE) {
      if (f0_en) {
        ffb_forces[0] = req.download_and_play_force.force_type;
        ffb_forces_en[0] = true;
      }
      if (f1_en) {
        ffb_forces[1] = req.download_and_play_force.force_type;
        ffb_forces_en[1] = true;
      }
      if (f2_en) {
        ffb_forces[2] = req.download_and_play_force.force_type;
        ffb_forces_en[2] = true;
      }
      if (f3_en) {
        ffb_forces[3] = req.download_and_play_force.force_type;
        ffb_forces_en[3] = true;
      }
    } else if (f_cmd == EnumFfbCmd::PLAY_FORCE) {
      if (f0_en) { ffb_forces_en[0] = true; }
      if (f1_en) { ffb_forces_en[1] = true; }
      if (f2_en) { ffb_forces_en[2] = true; }
      if (f3_en) { ffb_forces_en[3] = true; }
    } else if (f_cmd == EnumFfbCmd::STOP_FORCE) {
      if (f0_en) { ffb_forces_en[0] = false; }
      if (f1_en) { ffb_forces_en[1] = false; }
      if (f2_en) { ffb_forces_en[2] = false; }
      if (f3_en) { ffb_forces_en[3] = false; }
    } else if (f_cmd == EnumFfbCmd::DEFAULT_SPRING_ON) {
      ffb_default_spring_on = true;
    } else if (f_cmd == EnumFfbCmd::DEFAULT_SPRING_OFF) {
      ffb_default_spring_on = false;
    } else if (f_cmd == EnumFfbCmd::REFRESH_FORCE) {
      if (f0_en) {
        ffb_forces[0] = req.refresh_force.force_type;
        ffb_forces_en[0] = true;
      }
      if (f1_en) {
        ffb_forces[1] = req.refresh_force.force_type;
        ffb_forces_en[1] = true;
      }
      if (f2_en) {
        ffb_forces[2] = req.refresh_force.force_type;
        ffb_forces_en[2] = true;
      }
      if (f3_en) {
        ffb_forces[3] = req.refresh_force.force_type;
        ffb_forces_en[3] = true;
      }
    } else if (f_cmd == EnumFfbCmd::SET_DEFAULT_SPRING) {
      ffb_default_spring_k1 = req.set_default_spring.k1;
      ffb_default_spring_k2 = req.set_default_spring.k2;
      ffb_default_spring_clip = req.set_default_spring.clip;
    }
  }

  // sets force in range 0x00 .. 0xff to force_current
  void update(uint16_t axis_wheel_value) {
    int16_t f = 0;

    if (ffb_default_spring_on)
    {
      int16_t k = (axis_wheel_value > axis_wheel_cnt) ? ffb_default_spring_k1 : -ffb_default_spring_k2;
      k *= ffb_default_spring_clip;
      k /= 7;
      f += k * abs(axis_wheel_value - axis_wheel_cnt) * 2 / (axis_wheel_max - axis_wheel_min);
    }

    for (uint8_t i = 0; i < 4; ++i) {
      bool f_en = ffb_forces_en[i];
      if (!f_en) continue;
      if (i != 0) continue;

      FfbForceType f_entry = ffb_forces[i];
      EnumForceType f_type = (EnumForceType)f_entry.bytes[0];

      if (f_type == EnumForceType::CONSTANT) {
        f += *(&f_entry.constant.f0 + i) - 0x7f;
      } else if (f_type == EnumForceType::SPRING) {
      } else if (f_type == EnumForceType::DAMPER) {
      } else if (f_type == EnumForceType::AUTO_CNT_SPRING) {
      } else if (f_type == EnumForceType::SAWTOOTH_UP) {
      } else if (f_type == EnumForceType::SAWTOOTH_DN) {
      } else if (f_type == EnumForceType::TRAPEZOID) {
      } else if (f_type == EnumForceType::RECTANGLE) {
      } else if (f_type == EnumForceType::VARIABLE) {
      } else if (f_type == EnumForceType::RAMP) {
      } else if (f_type == EnumForceType::SQUARE_WAVE) {
      } else if (f_type == EnumForceType::HI_RES_SPRING) {
      } else if (f_type == EnumForceType::HI_RES_DAMPER) {
      } else if (f_type == EnumForceType::HI_RES_AUTO_CNT_SPRING) {
      } else if (f_type == EnumForceType::FRICTION) {
      }
    }

    f += 0x7f;

    if (f < 0x00) f = 0x00;
    if (f > 0xff) f = 0xff;

    force_current = f;
  }

  uint8_t get_force() {
    return force_current;
  }

  uint16_t axis_wheel_min;
  uint16_t axis_wheel_cnt;
  uint16_t axis_wheel_max;

  uint8_t force_current;

  FfbRequest ffb_request;
  FfbForceType ffb_forces[4];
  bool ffb_forces_en[4];
  bool ffb_default_spring_on;
  uint8_t ffb_default_spring_k1;
  uint8_t ffb_default_spring_k2;
  uint8_t ffb_default_spring_clip;
};


// kalman filter for filtering steering wheel angle got from linear potentiometer (as voltage)
// predicts state of angle, angular velocity, angular acceleration
// outputs filtered angle
// using TinyMatrixMath for matrix comutations, TODO optimize later
class KalmanFilter {
public:
  KalmanFilter(float a1, float a2, float a3, float q1, float q2, float q3, float r) {
    I = 0.0f;
    I[0][0] = 1.0f;
    I[1][1] = 1.0f;
    I[2][2] = 1.0f;

    A[0][0] = a1;
    A[0][1] = a2;
    A[0][2] = a3;
    A[1][0] = 0;
    A[1][1] = a1;
    A[1][2] = a2;
    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = a1;

    H[0][0] = 1;
    H[1][0] = 0;
    H[2][0] = 0;

    Q[0][0] = q1;
    Q[0][1] = 0;
    Q[0][2] = 0;
    Q[1][0] = 0;
    Q[1][1] = q2;
    Q[1][2] = 0;
    Q[2][0] = 0;
    Q[2][1] = 0;
    Q[2][2] = q3;

    R[0][0] = r;

    P = I;

    x = 0.0f;
  }

  void update(float z) {
    y = -H * x + z;
    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (I - K * H) * P;
  }

  float predict() {
    x = A * x;
    P = A * P * A.transpose() + Q;
    return x[0][0];
  }

  tmm::Matrix< KALMAN_DIM, KALMAN_DIM > A;
  tmm::Matrix< 1, KALMAN_DIM > H;
  tmm::Matrix< KALMAN_DIM, KALMAN_DIM > Q;
  tmm::Matrix< 1, 1 > R;
  tmm::Matrix< KALMAN_DIM, KALMAN_DIM > P;
  tmm::Matrix< KALMAN_DIM, 1 > x;

  tmm::Matrix< 1, 1 > y;
  tmm::Matrix< KALMAN_DIM, KALMAN_DIM > I;
  tmm::Matrix< 1, 1 > S;
  tmm::Matrix< KALMAN_DIM, 1 > K;
};


// main wheel class, glues everything together (reading wheel angle, setting ffb force, etc)
// TODO cut out motor handling to another class
// TODO cut out potentiometer reading to another class, make it more modular to support also hall sensor, etc
class WheelController : public USBHIDDevice {
public:
  WheelController()
    : status{},
      ffb_controller(0x1fff, 0x3fff),
      filter(1.0f, DT_MEASURE_US * 1.0e-6f, DT_MEASURE_US * DT_MEASURE_US * 2.0e-6f,
             DT_MEASURE_US * 5.0e-6f, DT_MEASURE_US * 5.0e-3f, DT_MEASURE_US * 5.0e-3f,
             1.0e5f) {
    hid.addDevice(this, sizeof(hid_report_descriptor));

    status.status.buttons_0 = 0x08;
  }

  WheelStatus status;
  FfbController ffb_controller;
  KalmanFilter filter;

  USBHID hid;

  void init() {
    hid.begin();
  }

  void update_ffb() {
    if (analogRead(PIN_ADC_WHEEL) < AXIS_WHEEL_ADC_MIN) {  // apply CW force when wheel angle below min (left)
      analogWrite(PIN_FFB_EN10, 0xf0);
      digitalWrite(PIN_FFB_IN11, 0);
      digitalWrite(PIN_FFB_IN12, 1);
      analogWrite(LED_BUILTIN, 0xf0);
      return;
    }
    if (analogRead(PIN_ADC_WHEEL) > AXIS_WHEEL_ADC_MAX) {  // apply CCW force when wheel angle above max (right)
      analogWrite(PIN_FFB_EN10, 0xf0);
      digitalWrite(PIN_FFB_IN11, 1);
      digitalWrite(PIN_FFB_IN12, 0);
      analogWrite(LED_BUILTIN, 0xf0);
      return;
    }

    ffb_controller.update(status.status.get_axis_wheel_14bit());
    int16_t f = this->ffb_controller.get_force();

    bool dir_ccw = f > 0x7f;

    f = abs(f - 0x7f) << 1;
    if (f > 0xf0) f = 0xf0;

    analogWrite(PIN_FFB_EN10, f);
    analogWrite(LED_BUILTIN, f);
    digitalWrite(PIN_FFB_IN11, dir_ccw);
    digitalWrite(PIN_FFB_IN12, !dir_ccw);
  }

  void update_axes() {
    update_axis_wheel();
  }

  void update_axis_wheel() {
    // potentiometer 10 revolutions for voltage 3.3
    // 330mV per revolution
    // 0.9167mV/deg
    // 12bit adc
    // adc voltage range 0V..2V5
    // 900deg is dV = 825mV
    // 825/2500 = 0.33
    // midV = 1.65 - middle of potentiometer
    // 12bit max value = 0x0fff
    // 1.65V is adc value 2702 (0x0a8e)
    // 1.65V - 450deg = 1.2375V = 2027adc
    // 1.65 + 450deg = 2.0625V = 3378adc


    int32_t v = analogRead(PIN_ADC_WHEEL);

    v -= AXIS_WHEEL_ADC_MIN;
    v *= 12812;
    v /= 1001;

    filter.update(v);  // this is heavy... TODO optimize
    v = filter.predict();

    if (v < 0x0000) v = 0x0000;
    if (v > 0x3fff) v = 0x3fff;

    status.status.set_axis_wheel_14bit(v);
  }

  void sendState() {
    uint8_t ffb_force_req = 0;

    if (hid.ready()) {
      hid.SendReport(DEV_REPORT_ID, status.bytes, sizeof(status.bytes), 0);
    }
  }

  // we set hid report descriptor in this callback, writing it to buffer provided
  uint16_t _onGetDescriptor(uint8_t *buffer) override {
    memcpy(buffer, hid_report_descriptor, sizeof(hid_report_descriptor));
    return sizeof(hid_report_descriptor);
  }

  uint16_t _onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) override {}

  // we get ffb request from host in this callback
  void _onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) override {
    if (len != sizeof(FfbRequest)) return;
    ffb_controller.apply_force(*((const FfbRequest *)buffer));
    // USBSerial.print("[IN]["); USBSerial.print(report_id); USBSerial.print("] ");
    // for (uint16_t i = 0; i < len; ++i)
    // {
    //   if (buffer[i] < 0x10) USBSerial.print(0);
    //   USBSerial.print(buffer[i], HEX);
    //   USBSerial.print(" ");
    // }
    // USBSerial.println();
  }

  void _onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len) override {}
};


WheelController whl;

TaskHandle_t TaskSendHidReportsHandle;

void TaskSendHidReports(void *parameter) {
  while (true) {
    static unsigned long time_last_report = millis();
    if (millis() - time_last_report >= DT_REPORT_MS) {
      time_last_report = millis();
      whl.sendState();
    }
  }
}

void setup() {
  setCpuFrequencyMhz(240);

  analogWriteResolution(LED_BUILTIN, 8);
  analogWriteResolution(PIN_FFB_EN10, 8);

  //analogWriteFrequency(LED_BUILTIN, 10);  // does not work for some reason, TODO investigate
  //analogWriteFrequency(PIN_FFB_EN10, 10);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_FFB_EN10, OUTPUT);
  pinMode(PIN_FFB_IN11, OUTPUT);
  pinMode(PIN_FFB_IN12, OUTPUT);

  analogSetPinAttenuation(PIN_ADC_WHEEL, ADC_11db);
  analogReadResolution(12);  // max 0x0fff

  // NOTE: for below code to work, disable USB CDC on boot in arduino Tools tab
  USB.VID(DEV_VID);
  USB.PID(DEV_PID);
  //USB.usbClass(TUSB_CLASS_HID);
  USB.usbPower(500);
  USB.usbAttributes(TUSB_DESC_CONFIG_ATT_SELF_POWERED);
  //USB.serialNumber("0x428a");
  //USB.firmwareVersion(0x0010);
  USB.usbVersion(0x0200);
  USB.productName(DEV_PRODUCT_NAME);
  USB.manufacturerName(DEV_MANUFACTURER_NAME);
  USB.begin();
  USBSerial.begin();  // use only for debug, it interferes with hid data sent to host when used too often; TODO disable later

  whl.init();

  // HID report update task on core 0 with priority 1 (was stalling when was 0), the rest runs on core 1
  xTaskCreatePinnedToCore(TaskSendHidReports, "TaskSendHidReports", 10000, NULL, 1, &TaskSendHidReportsHandle, 0);
}


void loop() {
  unsigned long start = micros();
  for (uint16_t i = 0; i < 1e4; ++i) {
    whl.update_axes();
    whl.update_ffb();
  }
  USBSerial.print("loop time = ");
  USBSerial.print((micros() - start) / 1e4);
  USBSerial.println("us");
}

