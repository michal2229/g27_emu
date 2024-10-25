# g27_emu

A basic USB steering wheel controller with force feedback for ESP32-S2.
Currently configured for a 10 turn linear potentiometer and a H-bridge for motor control. 
I might replace potentiometer with Hall effect encoder in the future, since potentiometer readings are quite noisy, with variance of about 800 for encoded 14bit wheel angle (in-game wheel vibrates a little; mitigated a bit using a Kalman filter).
No buttons/pedals implemented yet, but it's on a TODO list (buttons and pedal axes already mapped to HID report data bits in my notes).

This project is a base for future work of creating a decent direct-drive wheel. I wanted to learn as much as I can in the process, so I decided to make thing from ground up. 
I also have a regular Logitech G29 wheel, which I might want to mod some time in the future. I might even replace the whole board inside with my stuff to have control over the wheel behavior.

If communication errors occur, try disabling USBCDC serial logging.
Developed on Linux, works with [new-lg4ff](https://github.com/berarma/new-lg4ff) driver. Tested mostly in BeamNG.drive.
Not tested on Windows/Mac or basic Linux driver.

Video demonstration: [2024/10/25](https://youtu.be/wODYtFMs8rI).
