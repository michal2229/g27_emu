# g27_emu

A basic steering wheel controller with force feedback for ESP32-S2.
Currently configured for a 10 turn linear potentiometer and a H-bridge for motor control. 
I might replace potentiometer with Hall effect encoder in the future, since potentiometer readings are quite noisy, with variance of about 800 for 12bit ADC (in-game wheel vibrates a little; mitigated a bit using a Kalman filter).
No buttons/pedals implemented yet, but it's on a TODO list (buttons and pedal axes already mapped to HID report data bits in my notes).

If communication errors occur, try disabling USBCDC serial logging.
Developed on Linux, not tested on Windows/Mac.
