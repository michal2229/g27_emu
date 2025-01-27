# g27_emu

A basic USB steering wheel controller with force feedback for [ESP32-S2](https://www.wemos.cc/en/latest/s2/s2_mini.html), emulates G27 wheel.
Currently configured for a 10 turn linear potentiometer and a H-bridge for motor control. 
I might replace potentiometer with Hall effect encoder in the future, since potentiometer readings are quite noisy, with variance of about 800 for encoded 14bit wheel angle (in-game wheel vibrates a little; mitigated a bit using a Kalman filter).
No buttons/pedals implemented yet, but it's on a TODO list (buttons and pedal axes already mapped to HID report data bits in my notes).

This project is a base for future work of creating a decent direct-drive wheel. I wanted to learn as much as I can in the process, so I decided to make things from ground up. 
I also have a regular Logitech G29 wheel, which I might want to mod some time in the future. I might even replace the whole board inside with my stuff to have control over the wheel behavior.
I know there already are projects like [OpenFFBoard](https://github.com/Ultrawipf/OpenFFBoard), but I guess I just like to reinvent the wheel from time to time.

If communication errors occur, try disabling USBCDC serial logging - there's a [bug in ESP32 libs](https://github.com/espressif/arduino-esp32/issues/10307).
Developed on Linux, works with [new-lg4ff](https://github.com/berarma/new-lg4ff) driver. Tested mostly in BeamNG.drive.
Not tested on Windows/Mac or basic Linux driver.

## schematic
<img src="https://raw.githubusercontent.com/michal2229/g27_emu/refs/heads/main/kicad/g27_emu/g27_emu.png" alt="schematic">

## videos
 * [2024-10-25: steering and force feedback in BeamNG.drive](https://youtu.be/wODYtFMs8rI)
