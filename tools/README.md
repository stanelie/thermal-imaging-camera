# tools

Kept here so they survive between sessions.

- **`picotool`** — built with USB support, which the SDK's bundled copy is not.
  Not committed (platform-specific binary); rebuild with `./build-picotool.sh`.
  Needs `sudo` to reach the device unless you install the udev rules.
- **`monitor.py`** — read the USB serial. `--reboot` resets the board first so
  the boot lines are caught; `--grep` filters.
- **`flash.sh`** — build, flash, and optionally monitor for N seconds.

Note the device's own `picotool` access: the stock credential helper cannot push
to this repo, see the main README.
