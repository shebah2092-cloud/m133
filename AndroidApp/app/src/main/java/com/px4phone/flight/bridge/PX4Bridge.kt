package com.ardophone.px4v17.bridge

object PX4Bridge {
    init { System.loadLibrary("px4phone_native") }

    // Start / Stop
    external fun startPX4(storagePath: String): Boolean
    external fun stopPX4()
    external fun isRunning(): Boolean

    // Native sensor counts (for UI rate display)
    external fun getNativeImuCount(): Long
    external fun getNativeBaroCount(): Long
    external fun getNativeMagCount(): Long
    external fun getNativeGpsCount(): Long

    // Vehicle state from uORB
    external fun getRoll(): Float
    external fun getPitch(): Float
    external fun getYaw(): Float
    external fun getAltitude(): Float
    external fun isArmed(): Boolean
    external fun getEKFStatus(): Int
    external fun getAirframeId(): Int

    // USB Servo output (Phase 11.2)
    external fun setServoUsbFd(fd: Int)

    // USB CAN adapter (Waveshare USB_CAN_A) for xqpower_can SLCAN
    external fun setCanUsbFd(fd: Int)

    // USB GPS UBX (u-blox binary protocol — reads directly in C++)
    external fun setGpsUsbFd(fd: Int)

    // CP210x telemetry radio (PTY bridge for mavlink -d)
    external fun setMavlinkTelemetryUsbFd(fd: Int)
}
