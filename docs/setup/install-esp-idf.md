# Phase 1 / Step 1 - Install ESP-IDF on Windows

We use the latest stable **ESP-IDF v6.x** (currently v6.0.1), Espressif's
official framework. See the
[Get Started guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html).

Espressif now installs ESP-IDF via the **ESP-IDF Installation Manager (EIM)**,
which has both a GUI and a CLI.

## Install with the ESP-IDF Installation Manager (EIM)

1. Download the **ESP-IDF Installation Manager (EIM)** for Windows from the
   [Get Started guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)
   and run it. Use the **GUI** option and select ESP-IDF **v6.x**; it fetches the
   toolchain, build tools (CMake + Ninja) and ESP-IDF for you.
2. When done, EIM creates an **"ESP-IDF PowerShell"** / **"ESP-IDF CMD"**
   Start-menu shortcut. Always open that shortcut to get a shell with `idf.py`
   on the PATH.

> Note: the old standalone "ESP-IDF Windows Installer" has been superseded by
> EIM. If you previously installed via that route, EIM can manage/uninstall it.

## USB driver

Plug in the ESP32 board. If a new COM port does not appear in Device Manager,
install the USB-UART driver for your board's chip:

- **CP210x** (most DevKitC) - Silicon Labs CP210x VCP driver.
- **CH340/CH9102** (many clones) - WCH driver.

Note the COM port number (e.g. `COM5`); you'll pass it to `flash`/`monitor`.

## Verify with the Hello world project

Open the **ESP-IDF PowerShell** shortcut, then from the `firmware/` folder:

```powershell
cd firmware
idf.py set-target esp32
idf.py build
idf.py -p COM5 flash monitor
```

Expected serial output (exit the monitor with `Ctrl+]`):

```
=== Self-Balancing Vehicle - Hello world ===
I (xxx) balance_bot: running on esp32 with 2 core(s)
I (xxx) balance_bot: ESP-IDF version: v6.x.x
I (xxx) balance_bot: heartbeat 0
I (xxx) balance_bot: heartbeat 1
...
```

If you see the heartbeat counting up, the toolchain, flashing and serial monitor
all work. Check off Steps 1 and 2 in [../ROADMAP.md](../ROADMAP.md).

## Troubleshooting

- **`idf.py` not found** - you're not in the ESP-IDF shell. Open the EIM
  "ESP-IDF PowerShell" / "ESP-IDF CMD" Start-menu shortcut.
- **Flash fails / port busy** - close any other serial monitor; confirm the COM
  port; some boards need you to hold **BOOT** while flashing starts.
- **Garbled serial output** - set the monitor baud to **115200** (the default).
