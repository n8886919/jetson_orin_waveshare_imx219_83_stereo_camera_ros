#!/usr/bin/env python3
import time
import math
import statistics
import os

import qwiic_icm20948
import qwiic_i2c.linux_i2c as linux_i2c

IIO_GYRO_DIR = "/sys/bus/iio/devices/iio:device1"


def _read_sysfs_int(path):
    with open(path, "r") as f:
        return int(f.read().strip())


def _open_sysfs(path):
    f = open(path, "r")
    return f


def _read_sysfs_fd(fd):
    fd.seek(0)
    return int(fd.read().strip())


def _run_loop(read_fn, duration_s=10.0, target_hz=200.0):
    period = 1.0 / target_hz
    t_start = time.perf_counter()
    t_next = t_start + period
    timestamps = []
    cpu_start = time.process_time()
    samples = 0
    while True:
        now = time.perf_counter()
        if now - t_start >= duration_s:
            break
        read_fn()
        samples += 1
        timestamps.append(now)
        sleep = t_next - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        t_next += period
    cpu_end = time.process_time()
    elapsed = time.perf_counter() - t_start
    cpu_pct = (cpu_end - cpu_start) / elapsed * 100.0
    dts = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
    stats = {
        "samples": samples,
        "elapsed_s": elapsed,
        "cpu_pct": cpu_pct,
        "dt_mean_ms": statistics.mean(dts) * 1000.0 if dts else float("nan"),
        "dt_std_ms": statistics.pstdev(dts) * 1000.0 if dts else float("nan"),
        "dt_min_ms": min(dts) * 1000.0 if dts else float("nan"),
        "dt_max_ms": max(dts) * 1000.0 if dts else float("nan"),
        "achieved_hz": (samples / elapsed) if elapsed > 0 else 0.0,
    }
    return stats


def benchmark_iio(duration_s=10.0, target_hz=200.0):
    x_fd = _open_sysfs(os.path.join(IIO_GYRO_DIR, "in_anglvel_x_raw"))
    y_fd = _open_sysfs(os.path.join(IIO_GYRO_DIR, "in_anglvel_y_raw"))
    z_fd = _open_sysfs(os.path.join(IIO_GYRO_DIR, "in_anglvel_z_raw"))

    def read_fn():
        _ = _read_sysfs_fd(x_fd)
        _ = _read_sysfs_fd(y_fd)
        _ = _read_sysfs_fd(z_fd)

    stats = _run_loop(read_fn, duration_s, target_hz)
    x_fd.close()
    y_fd.close()
    z_fd.close()
    return stats


def benchmark_userspace(duration_s=10.0, target_hz=200.0):
    i2c = linux_i2c.LinuxI2C(iBus=7)
    imu = qwiic_icm20948.QwiicIcm20948(address=0x68, i2c_driver=i2c)
    if not imu.begin():
        raise RuntimeError("ICM20948 begin() failed")

    def read_fn():
        imu.getAgmt()
        _ = imu.gxRaw
        _ = imu.gyRaw
        _ = imu.gzRaw

    return _run_loop(read_fn, duration_s, target_hz)


def main():
    duration_s = 10.0
    target_hz = 200.0

    print("Benchmark settings: duration_s=%.1f, target_hz=%.1f" % (duration_s, target_hz))

    print("\n[Kernel IIO sysfs gyro]")
    iio_stats = benchmark_iio(duration_s, target_hz)
    for k, v in iio_stats.items():
        print(f"{k}: {v}")

    print("\n[Userspace qwiic_icm20948 gyro]")
    user_stats = benchmark_userspace(duration_s, target_hz)
    for k, v in user_stats.items():
        print(f"{k}: {v}")

    print("\nSummary:")
    print("- Lower dt_std_ms and dt_max_ms is better (less jitter)")
    print("- Lower cpu_pct is better")
    print("- achieved_hz closer to target is better")


if __name__ == "__main__":
    main()
