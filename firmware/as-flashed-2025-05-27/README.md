# Firmware read back off the device, 2025-09-29

`thermocam-device-readback.uf2` was read out of the RP2040's flash with
`picotool save` before any reflashing in this session. This is the exact
image that was running on the camera.

Its build date is May 27 2025 and its binary is 43940 bytes. The build
left behind in `build/` is dated May 26 2025 and is 43724 bytes, so the
running firmware was a later build whose source was never committed.
This readback is the only copy of it.

Restore with:

    picotool load -x thermocam-device-readback.uf2

or copy it onto the RPI-RP2 drive with the board in BOOTSEL mode.
