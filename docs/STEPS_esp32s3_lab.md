# Lab day: move to the ESP32-S3 and bring Pathik up again (every step)

What you need on the laptop: `slam-robot.zip`, Arduino IDE 2, `Pathik-11.apk` on the phone,
the multimeter. Wiring list: `hardware/CONNECTIONS_esp32s3.md` (every wire, both ends).

Do it in order. Every **Check** has to pass before moving on.

---

## 0. Laptop: unzip

```bash
cd ~/Downloads
rm -rf slam-robot && unzip slam-robot.zip      # makes ~/Downloads/slam-robot
cd slam-robot
```

## 1. Flash the ESP32-S3 (on the bench, nothing else connected)

1. Arduino IDE → **Boards Manager** → "esp32 by Espressif" (version 3.0.x or 2.0.17 both work).
2. Open `firmware/robot_esp32s3/robot_esp32s3.ino` (the folder name must stay `robot_esp32s3`).
3. **Tools** menu:
   - Board: **ESP32S3 Dev Module**
   - USB Mode: **Hardware CDC and JTAG**
   - USB CDC On Boot: **Enabled**
   - Upload Mode: **UART0 / Hardware CDC** (default)
   - Port: the one that appears when you plug the **"USB"** socket of the board
4. **Upload.** If the port doesn't appear or the upload fails: hold **BOOT**, tap **RST**, release BOOT, upload again,
   then press **RST** once after it finishes.
5. **Check:** Serial Monitor at **115200**:
   ```
   INFO,READY,robot_esp32 v2.5 ESP32-S3 reset=POWERON
   ```
   The on-board LED **breathes blue**. Dark LED → change `#define RGB_PIN 48` to `38`, upload again.
6. Type `?` → config lines. Type `L,255,0,255,2000` → LED magenta for 2 s.

## 2. Wiring (battery OFF, ESP32 unplugged)

Follow `hardware/CONNECTIONS_esp32s3.md` in this order: power + capacitors (section 1) → motor drivers (2) →
encoders (3) → RC (4) → IMU (5) → ultrasonics + level shifter (6) → e-stop (7) → ESP32 GND + USB (8).

Before power-on, with the multimeter:
- [ ] Buck output = **5.0–5.1 V**
- [ ] No short between 5 V and GND, or between battery + and GND (beep test)
- [ ] Battery divider midpoint (if fitted) **< 2.5 V** with the battery connected
- [ ] Encoder A pin swings 0 → ≤ 3.3 V when you turn the wheel (encoder on 3V3)
- [ ] No ECHO wire goes straight to the ESP32

If you fitted the battery divider: set `#define USE_BAT_ADC 1` and upload again.
If you fitted bumpers: `#define USE_BUMPER 1`.

## 3. First power-up checks (robot ON BLOCKS, wheels in the air, ESP32 on the laptop USB)

Serial Monitor 115200:
1. **Encoders:** turn each wheel forward by hand → `ODM,` values go up (positive) on both.
   Wrong sign → flip `LEFT_ENC_DIR` / `RIGHT_ENC_DIR` (1 ↔ −1), upload.
2. **Motors:** battery ON. Type `P,120,120` → both wheels turn **forward**. `S` stops.
   Wrong way → flip `LEFT_MOTOR_DIR` / `RIGHT_MOTOR_DIR`, upload. LED turns **rainbow** while driving.
3. **Speed loop:** `V,200,200` → both wheels same speed; `S`.
4. **RC:** transmitter ON, sticks drive the wheels, LED **magenta**. Stick forward = forward, stick left = turn left.
5. **E-stop:** press it → wheels can't move, LED **flashes red**. Release the mushroom, type `R`.
6. **IMU:** `?` shows `imu=ok`. Turn the robot left by hand → heading goes up.
7. **Ultrasonics:** `US,` lines; hand 30 cm in front of each sensor ≈ `300`.
8. **Battery:** `INFO,battery sensor: divider on GPIO 1` and `BAT,` mV ≈ multimeter reading (± 0.2 V).

Close the Serial Monitor. Plug the ESP32 into the **Pi** instead of the laptop.

## 4. Connect to the Pi

Laptop (ethernet cable in):
```bash
ip addr | grep 169.254 || ./scripts/setup_laptop_link.sh enx006f0000324d
ssh arpit@169.254.1.2                 # host key error: ssh-keygen -R 169.254.1.2
```
**Check** on the Pi:
```bash
ls /dev/ttyACM* /dev/ttyUSB*          # expect /dev/ttyACM0 (ESP32-S3) and /dev/ttyUSB0 (LiDAR)
vcgencmd get_throttled                # must be throttled=0x0
```

## 5. Copy the new code to the Pi and build

Laptop (new terminal, not inside ssh):
```bash
ssh arpit@169.254.1.2 'rm -rf ~/slam-robot'
scp -r ~/Downloads/slam-robot arpit@169.254.1.2:~/
```
Pi:
```bash
sudo systemctl stop robot
rm -rf ~/ros2_ws/src/lidar_robot ~/ros2_ws/build/lidar_robot ~/ros2_ws/install/lidar_robot
cp -r ~/slam-robot/ros2_ws/src/lidar_robot ~/ros2_ws/src/
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lidar_robot
sudo cp ~/slam-robot/scripts/robot.service /etc/systemd/system/ && sudo systemctl daemon-reload
```

## 6. robot.env (ports + Groq key; this file never goes to GitHub)

Pi:
```bash
eval "$(python3 ~/slam-robot/scripts/find_ports.py)"; echo "ESP=$ESP LIDAR=$LIDAR"
nano ~/robot.env
```
Contents (paste your own key from console.groq.com, keep the quotes off):
```
ROBOT_ARGS=esp32_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0
GROQ_API_KEY=gsk_xxxxxxxxxxxxxxxx
```
```bash
chmod 600 ~/robot.env
bash ~/slam-robot/tools/link_doctor.sh --fix     # Wi-Fi power save off, stop ModemManager/brltty (needs sudo)
sudo systemctl enable --now robot
journalctl -u robot -f                           # look for "Connected to ESP32 on /dev/ttyACM0"; Ctrl+C
```
**Check:** `bash ~/slam-robot/tools/link_doctor.sh` shows the ESP32 on /dev/ttyACM0 and no errors.

## 7. App: install, first account, drive

1. Phone: install **Pathik-11.apk** (allow "install unknown apps").
2. Phone on the same network (robot Wi-Fi, phone hotspot or Tailscale), open the app → it finds the robot
   (or type `169.254.1.2`/the Pi's Wi-Fi IP).
3. First time it shows **"Set up this robot"** → create **your** account: that one is the **admin**.
4. **Check:** Lab → Health: Motor board ≈ **50 Hz**, LiDAR OK, battery shows a %, Link drops = 0.
5. Drive tab: joystick slowly, robot on the floor. STOP button works; hold to release.
6. (Optional) teammates: they tap **Request access**, you approve in **Me → People** with a role.

Wi-Fi for the floor (phone hotspot):
```bash
sudo bash ~/slam-robot/scripts/setup_wifi.sh hotspot "HotspotName" "password"
```

## 8. Drive test for the link drops

Pi (second ssh window):
```bash
bash ~/slam-robot/tools/link_doctor.sh --watch      # records ~/link_log.txt
```
Drive around for 5 minutes: hard starts, reversing, turning on the spot. Then Ctrl+C and:
```bash
cat ~/link_log.txt | tail -40
```
**Goal:** 0 USB drops and 0 resets. App Lab → Health → Link drops shows the same counters.
If drops show up: send the log (resets with a reason = power; usb drops = cable/noise).

## 9. Audio check (sound card + mic + speaker)

```bash
sudo apt install -y espeak-ng alsa-utils
aplay -l ; arecord -l                                      # note the card number of the USB sound card, e.g. card 1
arecord -D plughw:1,0 -f S16_LE -r 16000 -c 1 -d 5 test.wav  # speak for 5 s
aplay -D plughw:1,0 test.wav                               # you should hear yourself
espeak-ng -v en-us "Hello, I am Pathik" --stdout | aplay -D plughw:1,0
```
Make the USB card the default (so the robot voice uses it):
```bash
printf 'defaults.pcm.card 1\ndefaults.ctl.card 1\n' > ~/.asoundrc     # use your card number
alsamixer      # F6 → USB card; raise Speaker and Mic levels; Esc
sudo systemctl restart robot     # robot now speaks events ("Going to ...")
```
Give `test.wav` recordings to Rashi for her voice tests.

## 10. Camera (only connect it today)

Pi **off** → connect the ribbon (see CONNECTIONS K1) → Pi on. The software (libcamera on Ubuntu 22.04) is the
next job; nothing else to do with it today.

## 11. Measure and save (for the software)

- [ ] Footprint from the **LiDAR centre**: front edge __ cm, rear edge __ cm, left __ cm, right __ cm
- [ ] Wheel separation (centre of left tyre to centre of right tyre): __ cm
- [ ] Heights from the floor: camera __ cm, front bumper face __ cm (for the dock contacts)
- [ ] App Lab → tests: **Spin** and **Straight line**: note the errors they report

Send these numbers + `~/link_log.txt` and I'll update `robot_params.yaml`.

## 12. Map + places (if time is left)

App → Map → **Map by itself** (or drive it around with the joystick) → **Save map**.
Then add places (Lab, HOD office, …) and rooms with contents, and try:
"take me to the room with the 3D printer".

---

## If something goes wrong

| Problem | Fix |
|---|---|
| No `/dev/ttyACM0` | cable is power-only, or plugged into the "COM" socket. Try another cable / socket |
| ESP32 port appears, but app says motor board offline | `journalctl -u robot -f`; check `robot.env` ports; `bash ~/slam-robot/tools/link_doctor.sh` |
| LED blue forever when on the Pi | the robot service isn't running or holds the wrong port |
| Upload: "No serial data received" | hold BOOT, tap RST, upload |
| Wheels run but `ODM` stays 0 | encoder VCC/GND, or A/B on wrong pins |
| One wheel faster/opposite | the `*_DIR` flips in section 3 |
| App can't log in / forgot password | Pi: `cd ~/ros2_ws && source install/setup.bash && python3 -m lidar_robot.auth ~/maps/users.json reset-admin arpit 'newpass'` |
| `throttled=0x...` not 0 | Pi power too weak: buck to 5.1 V, thicker/shorter USB-C cable |
