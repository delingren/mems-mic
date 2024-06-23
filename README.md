# Microphone Library for Pico

I forked this library to interface a stereo microphone with my computer. The said microphone is two mics on one PCB. Each has its own clock and data wires. I modified `usb_microphone` example to sample both mics and mux the signals into one USB stream. If anyone comes upon this repo, note that I have no intention or interest to make this a library. It works for this particular application. The code is super hacky and ugly. See the [original post](https://www.hackster.io/sandeep-mistry/create-a-usb-microphone-with-the-raspberry-pi-pico-cc9bd5) for more information about PDM microphones. I also referenced the `audio_4_channel_mic` example from [tinyusb lib](https://github.com/hathach/tinyusb/tree/master/examples/device/audio_4_channel_mic/src).

Highlights of my changes:
* Define a USB descriptor for a 2-channel mic.
* Modify PIO instructions to read 2 data pins (they need to be consecutive). Each pin for one channel.
* Modify PDM filter code to separate signals from the raw samples collected from the previous step. Bits representing the left and right channels are alternating. I did some bitwise gymnastics to separate them with very few operations.
* Modify PDM filter code to filter both signals. The filter code is stateful. You need to basically create two filters for two channels. You can't call the filter function on two signals.

Original README follows

Capture audio from a microphone on your [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/) or any [RP2040](https://www.raspberrypi.org/products/rp2040/) based board. 🎤


## Hardware

 * RP2040 board
   * [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/)
 * Microphones
   * Analog
     * [Electret Microphone Amplifier - MAX9814 with Auto Gain Control](https://www.adafruit.com/product/1713) 
   * PDM
     * [Adafruit PDM MEMS Microphone Breakout](https://www.adafruit.com/product/3492)

### Default Pinout

#### Analog Microphone

| Raspberry Pi Pico / RP2040 | Analog Microphone |
| -------------------------- | ----------------- |
| 3.3V | VCC |
| GND | GND |
| GPIO 26 | OUT |

#### PDM Microphone

| Raspberry Pi Pico / RP2040 | PDM Microphone |
| -------------------------- | ----------------- |
| 3.3V | VCC |
| GND | GND |
| GND | SEL |
| GPIO 2 | DAT |
| GPIO 3 | CLK |

GPIO pins are configurable in examples or API.

## Examples

See [examples](examples/) folder.


## Cloning

```sh
git clone https://github.com/ArmDeveloperEcosystem/microphone-library-for-pico.git 
```

## Building

1. [Set up the Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
2. Set `PICO_SDK_PATH`
```sh
export PICO_SDK_PATH=/path/to/pico-sdk
```
3. Create `build` dir, run `cmake` and `make`:
```
mkdir build
cd build
cmake .. -DPICO_BOARD=pico
make
```
4. Copy example `.uf2` to Pico when in BOOT mode.

## License

[Apache-2.0 License](LICENSE)

## Acknowledgements

This project was created on behalf of the [Arm Software Developers](https://developer.arm.com/) team, follow them on Twitter: [@ArmSoftwareDev](https://twitter.com/armsoftwaredev) and YouTube: [Arm Software Developers](https://www.youtube.com/channel/UCHUAckhCfRom2EHDGxwhfOg) for more resources!

The [OpenPDM2PCM](https://os.mbed.com/teams/ST/code/X_NUCLEO_CCA02M1//file/53f8b511f2a1/Middlewares/OpenPDM2PCM/) library is used to filter raw PDM data into PCM. The [TinyUSB](https://github.com/hathach/tinyusb) library is used in the `usb_microphone` example.

---

Disclaimer: This is not an official Arm product.
