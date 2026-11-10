# M45 Gamma Firmware

![M45 Bitaxe web dashboard](docs/screenshots/dashboard.png)

M45 is experimental firmware for Bitaxe Gamma 602 miners. It gives the Gamma a
local web dashboard, QR-assisted setup, weighted multi-pool mining, and opt-in
overclock controls while booting at stock ASIC settings by default.

[Open the Web Flasher](https://m45-software.github.io/M45-Gamma-Firmware/)
to flash a prepared build from Chrome or Edge using USB serial.

Use it at your own risk. Overclocking or bad cooling can permanently damage
the ASIC, regulator, fan, wiring, or power supply, and can create a fire risk.

## Highlights

- Local dashboard for hashrate, temperatures, power, fan, pool status, best
  diff, and block-found alerts.
- Swarm dashboard that shows the local miner immediately, discovers M45 and
  AxeOS miners on the local `/24`, and caches last-known peers in the browser.
- Browser controls for Wi-Fi, pools, fan mode, ASIC clock/voltage, display
  sleep, screensaver, and ASIC power.
- Single-pool mining with an editable ordered list of backup pools, or weighted
  multi-pool mining with per-pool weights and usernames.
- Friendly device names in setup, the dashboard title, and swarm cards.
- Light and dark dashboard themes with optional browser persistence.
- Experimental auto-clock overclocking with temperature, VIN, current,
  VR-temperature, and watt-cap guardrails. Auto-clock stops increasing and backs
  off when either fresh or averaged TPS VIN reaches VIN Stop Low + 0.20 V
  (or VIN Watch Low, if higher). With default limits this is 5.00 V. Increases
  remain blocked until both readings recover to at least 5.05 V for five seconds;
  the dashboard shows “Overclock limited by voltage” with the reading and thresholds.
  Low-VIN backoff runs at most once per second and never raises core voltage.
- Stock clock and voltage enforcement unless overclocking is explicitly
  enabled.
- Runtime safety-limit controls, including TPS546 PMBus limit updates.
- OLED QR codes for first-boot Wi-Fi setup and the post-setup dashboard URL.
- BOOT-button Wi-Fi recovery and five-second full settings reset with OLED
  instructions.
- Fixed-size, removable hardware fault history.
- Wallet payout detection from coinbase data when the pool exposes enough
  information, block-found alerting, and best-diff reset.
- Native M45 JSON endpoints plus ESP-Miner-compatible JSON routes for existing
  tooling.

See [`docs/json-endpoints.md`](docs/json-endpoints.md) for native M45 and
ESP-Miner-compatible API routes.

## Technical Reference

### Hardware

This firmware targets Bitaxe Gamma 602 hardware:

- ESP32-S3 with 16 MB flash and octal PSRAM.
- BM1370 ASIC, stock default `525 MHz` at `1150 mV`.
- TPS546 PMBus regulator at I2C address `0x24`.
- EMC2101 fan controller on the Bitaxe I2C bus.
- SSD1306 OLED at `0x3c`, `128x32`, on the Bitaxe I2C bus.

Other boards may need source changes before they are safe to run.

### First Boot

If the device has no saved Wi-Fi credentials, it starts a temporary setup
network named `m-XXXX` and shows a Wi-Fi QR code on the OLED. Join that
network, open the displayed setup address, then enter Wi-Fi and pool settings.

After the device joins Wi-Fi, the OLED shows the local dashboard address and an
HTTP QR code.

### Device Name, Backup Pools, And Theme

Enter an optional **Device Name** during first-boot setup or in Settings. It is
shown in the dashboard title and swarm cards. Names can contain spaces and are
stored on the miner (up to 64 UTF-8 bytes). An empty name uses the network
hostname; renaming the device does not reconnect Wi-Fi.

In Settings, choose Single pool mode and use **Add Backup** to build an ordered
list. Each backup has a host, port, and optional verified TLS. Move rows with the
arrow buttons, remove unwanted rows, then click **Save Backup Pools**. This
button saves the list independently of **Save & Apply**. Backups use the primary
pool's username and password. Connection failures cycle through primary, each
backup in order, then primary again. While on a backup, the miner periodically
probes the primary and returns when it responds. Weighted mode retains the
saved backup list but does not use it.

There is no fixed pool-count limit. The list is allocated dynamically and stored
as an NVS blob; available flash storage and a 32 KiB JSON request budget bound its
size. Failed saves keep the active list. Older firmware's legacy backup fields
remain compatibility settings; they do not automatically populate this list.
Factory Reset clears names and backups.

Choose **Dashboard Theme** under Display. Changes apply immediately. Select
**Remember theme in this browser** to restore the choice on later visits,
including Wi-Fi setup. With remembering disabled, later page loads use Dark.
The preference belongs to that browser and device URL and can also be cleared
by clearing browser site data.

### Physical Recovery

Hold BOOT while starting or resetting the device, and keep holding it for the
full five-second OLED countdown, to erase all settings. Releasing BOOT early
cancels the erase and boots normally.

### Updates And OTA

Use the web flasher above for first install or USB recovery.

OTA accepts either:

- `esp-miner.bin`, the raw application image for installing M45 over stock
  AxeOS/ESP-Miner from its OTA page. On its first M45 boot it imports the
  existing Wi-Fi, hostname, and selected Stratum V1 pool settings before
  writing M45 configuration. Stratum V2 and hardware tuning settings are not
  imported.
- `esp-miner-factory-602-*.bin`, the merged single-file image used by the USB
  web flasher and full-flash tools.
- `M45-Firmware.bin`, a compatibility copy of the raw application image in
  local OTA packages.

Canonical releases publish `esp-miner.bin` and the merged factory image directly
to GitHub Pages. Run the **Web Flasher Pages** workflow on the release branch
with a required semantic version such as `v0.1.0`; that version is embedded in
the application and written to `canonical-releases.json`. GitHub Pages is the
only update channel for v0.1.0 and later.

The AxeOS settings importer runs only when no M45 configuration exists. It reads
stock settings from AxeOS's `main` NVS namespace and leaves that namespace
unchanged. If the imported AxeOS board identity is missing or is not `602`, M45
still starts Wi-Fi and its dashboard but keeps the ASIC disabled. The OLED
identifies the unsupported board; pressing BOOT at that screen, or choosing
**Return to AxeOS** in the dashboard, boots a validated retained `esp-miner`
application partition. A fresh full flash with erased settings is treated as an
explicit Gamma 602 installation.

OTA images carry a Gamma 602 board identity and a configuration epoch. Firmware
release numbers may move forward or backward. When installing firmware with an
older configuration epoch, settings are erased before it boots. At startup,
firmware also checks the stored settings schema before reading any setting; a
schema newer than it supports is erased and the device returns to setup mode.

Application OTA never rewrites the device partition table. Existing M45 devices
with 3 MB slots and devices transitioned from AxeOS with 4 MB slots therefore
keep their installed layout. New M45 factory images use the AxeOS/ESP-Miner
16 MB partition layout verbatim, including its 4 MB factory and OTA app slots
and retained `www` partition. Normal M45 application images are checked against
the actual target slot size at upload time.

The Update page also has a separate **Install AxeOS** area. It accepts either a
new merged single-file factory image or the legacy `esp-miner.bin` plus matching
`www.bin` pair. Merged images contribute only their application image; M45 does
not rewrite the bootloader, partition table, or NVS during an AxeOS install.

After an OTA reboot, Gamma 602 hardware initialization runs before mining starts.
Configuration-epoch checks erase incompatible newer settings before older
firmware reads them.

### Stored Credentials

Empty Stratum passwords and the conventional default password `x` are not
written to NVS. Wi-Fi passwords and non-default Stratum passwords remain stored
in ordinary NVS; this firmware does not provision eFuses or enable encrypted
NVS.

The Logs page keeps the eight most recent persistent hardware/safety faults.
Each entry includes the shutdown reason, measured value and configured limit,
and TPS status flags/registers and sampled voltage, current, and temperature
when available. Failed reads identify the PMBus register and ESP error code;
unread values are explicitly unavailable. Date/time is displayed in the
browser's time zone with boot uptime, or a clock-not-synchronized notice for
events before time sync. Each entry has an individual remove button and a
Clear All action. Existing fault history is retained when upgrading.
Updates publish one small history index after staging a new record. Interrupted
updates retain a complete old or new history, including during migration from
older firmware. A failed history/extrema load is retried without overwriting the
unread data; a manual clear can explicitly reset it.

The Logs page also shows the last 64 error-level log entries captured since the
firmware started, even when the page was closed. This RAM-only history resets
on reboot, retains messages up to 2 KiB, and reports omitted entries and
truncated messages. Informational/warning logs remain in the live log view.
Wi-Fi starts ongoing SNTP clock synchronization for both plain and TLS pools;
clock sync does not delay mining on plain connections.

The Logs page shows the lowest and highest input voltage and the highest TPS
and ASIC temperatures observed since the last manual clear. These use raw valid sensor reads,
including fault-bearing and partially successful TPS reads, rather than rolling
averages. They remain available after ASIC shutdown and while the browser is
closed, and are saved across reboots. **Clear Extremes** starts a new recording
period without removing fault history. Recording resumes with the next valid
sensor reading. Changes are saved in the background once per second, and flushed
after a safety shutdown and before a normal firmware reboot. Sudden power loss
can lose up to the most recent second of observations. Missing readings
show `--`. These are observed extremes, not guaranteed hardware trip peaks.

### Network Trust And Exposure

M45 is designed for use on a trusted local network. Its HTTP dashboard and API
do not provide user accounts or access control, so any client that can reach the
device can view its status and change its configuration, hardware settings, or
firmware.

Do not expose the device HTTP port through router port forwarding, UPnP, a
public reverse proxy, or a cloud tunnel. Do not place it on an untrusted or guest
LAN/VLAN unless that network prevents other clients from reaching the device.
Browser cross-origin management requests are blocked, but that is not a
substitute for network isolation.

### Build And Flash

Docker is the easiest build path. The default ESP-IDF 5.5.6 image is pinned
to a verified digest. Use `--idf-version` or `M45_IDF_VERSION` to select another
official SDK image tag or tag with digest.

```sh
scripts/docker-build.sh
```

Firmware artifacts are written to `build/docker/`. Docker builds use repository
defaults and do not embed local Wi-Fi or pool settings.

Flash from Docker on Linux:

```sh
scripts/docker-build.sh --flash /dev/ttyUSB0 --monitor
```

Use `/dev/ttyACM0` instead if that is how your board appears.

Build local OTA upload files:

```sh
scripts/docker-ota.sh
```

Build the static web flasher package:

```sh
scripts/docker-web-flasher.sh
```

If ESP-IDF `v5.5.6` is installed locally:

```sh
scripts/build.sh --build
scripts/build.sh --flash /dev/ttyUSB0
```

`scripts/build.sh --help` lists all build-time configuration options.

### Host Checks

Run `bash scripts/test-host.sh` for the host regression suite. Set
`M45_ESPTOOL_BUNDLE` to a downloaded copy of the pinned esptool-js `bundle.js`
with an `.mjs` extension to include the real-library flasher compatibility test.
CI runs this suite and a full firmware/package build on pushes and pull requests.

With `IDF_PATH` pointing to an ESP-IDF checkout, run
`tests/run-backup-pools.sh` to check backup-list validation, persistence, failure
retention, DNS caching, and endpoint rotation using the bundled cJSON source.
The harness uses AddressSanitizer and UndefinedBehaviorSanitizer. Under a
ptrace-based runner, use `ASAN_OPTIONS=detect_leaks=0` because LeakSanitizer
cannot run there.

Run `tests/run-fault-diagnostics.sh` for sanitized host checks of TPS diagnostic
formatting, retained pre-error telemetry, legacy history migration, timestamp
persistence, and error capture with live logging inactive.

Run `python3 tests/run-telemetry-extrema-store.py` for sanitized checks of
persistence across reboot, manual clear, storage-failure retention/retry, and
skipping unchanged flash writes.

Run `python3 tests/run-asic-temperature.py` for checks of the missing-temperature
timeout, startup/recovery grace, valid-read recovery, and immediate hard faults.

Run `python3 tests/run-hardware-startup.py`,
`python3 tests/run-tps-read-retry.py`, `python3 tests/run-asic-rx-quiesce.py`,
`tests/run-tps-readback.sh`, and `tests/run-chip-detection.sh` for sanitized
checks covering the read grace window, hard-fault exclusions, startup failures,
UART reader quiescence, and bounded chip detection with malformed traffic.

Run `node --test tests/dashboard-faults.test.cjs tests/logging-extrema.test.cjs`
for dashboard checks covering
failed startup banners, manual retry availability, safety shutdown controls,
and dismissal of separate fault occurrences.

### Safety Behavior

The firmware holds ASIC reset low and turns TPS546 output off if a critical
condition is detected. Default shutdown limits include:

- ASIC temperature reaches `69 C`.
- Enabled ASIC VOUT falls below `0.700 V` or reaches `1.400 V`.
- TPS546 temperature reaches `110 C`.
- Input VIN falls below `4.8 V` or reaches `5.5 V`.
- TPS546 output current reaches `33 A`.
- Temperature monitor reads fail while hardware is active.

ASIC detection and TPS546 read failures get a 5-second read-retry window.
The firmware retries communication without rebooting or cycling ASIC power.
Read attempts pause 100 ms between failures; an ASIC chip-ID receive can wait
up to one second for a response within that window.
A successful read clears the transient condition. If reads remain unavailable
for the full window, the firmware shuts down and the dashboard shows the reason.
A read transaction already in progress may finish after the retry deadline.
Confirmed unsafe readings and regulator fault flags still shut down immediately.
ASIC temperature sentinel/no-reading values get a five-second observation
window after startup grace or an active domain-recovery period ends. A valid
temperature clears that window; persistent unavailable readings stop the ASIC.
Fan, memory, writes, and other initialization failures do not use this read grace.
Failed ASIC detection during a runtime startup permits a manual **Turn On** retry.

Transient failures and fault-triggered shutdowns include the clock speed,
voltage settings, and last valid TPS voltages and ASIC/VR temperatures in the
diagnostic log. Sample ages identify older readings, and missing readings are
marked unavailable. Read recovery or exhaustion is logged. Dismissing a
dashboard fault hides that occurrence for the current page visit; a new failure
or page visit shows it again.

TPS fault records distinguish the fault-status **readback** from the **last
validated TPS** operating sample retained before that read. A fault-bearing
read does not replace the retained sample. Hardware protection may trip between
monitor polls, so readback voltage/current may already have fallen or recovered
when the fault flags are read. Neither sample is a captured trip peak; latched
fault flags can therefore report a trip even when readback values are within
the configured limits. The formatted record is captured before software
shutdown clears live telemetry.

These limits are last-resort protections, not a substitute for proper cooling,
power delivery, and monitoring. Unrestricted safety-limit ranges are available
for advanced testing, but they do not make unsafe values safe.

## Licensing

This repository is licensed under GPL-3.0. See `LICENSE`.

Third-party license details and upstream hardware references are tracked in
`THIRD_PARTY_NOTICES.md`. BM1370 hardware and protocol references come from
[`bitaxeorg/esp-miner`](https://github.com/bitaxeorg/esp-miner).
