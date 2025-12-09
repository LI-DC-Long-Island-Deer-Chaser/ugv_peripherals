#!/usr/bin/env python3
import subprocess
import re

def sh(cmd):
    """Run a shell command and return stdout text."""
    return subprocess.check_output(cmd, shell=True, text=True).strip()

def get_default_interface():
    """Extract default route interface from `ip route list`."""
    out = sh("ip route list")
    for line in out.splitlines():
        if line.startswith("default "):
            return line.split()[4]
    return None

def parse_iw_link(text):
    """Parse `iw dev iface link` output."""
    info = {}

    # Connected BSSID
    m = re.search(r"Connected to ([0-9A-Fa-f:]+)", text)
    if m:
        info["bssid"] = m.group(1)

    # ESSID
    m = re.search(r"SSID: (.+)", text)
    if m:
        info["essid"] = m.group(1).strip()

    # Signal level
    m = re.search(r"signal: (-?\d+) dBm", text)
    if m:
        info["signal_dbm"] = int(m.group(1))

    # Bitrate (optional)
    m = re.search(r"tx bitrate: ([0-9.]+) MBit/s", text)
    if m:
        info["tx_bitrate_mbps"] = float(m.group(1))

    return info

def get_wifi_status():
    iface = get_default_interface()
    if not iface:
        raise RuntimeError("No default interface found.")

    raw = sh(f"iw dev {iface} link")
    parsed = parse_iw_link(raw)

    return {
        "interface": iface,
        "connected": parsed != {},
        **parsed
    }

if __name__ == "__main__":
    print(get_wifi_status())

