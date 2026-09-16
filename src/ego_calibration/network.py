"""Choose the running device's IPv4 address without contacting the Internet."""
from __future__ import annotations

import ipaddress
import json
import socket
import subprocess


def _usable(address):
    try:
        value = ipaddress.IPv4Address(address)
        return not (value.is_loopback or value.is_link_local or value.is_unspecified
                    or value.is_multicast or value.is_reserved
                    or value in ipaddress.IPv4Network("198.18.0.0/15"))
    except (ipaddress.AddressValueError, TypeError):
        return False


def _ip_json(*arguments):
    try:
        result = subprocess.run(["ip", "-j", "-4", *arguments], capture_output=True,
                                text=True, timeout=2, check=True)
        value = json.loads(result.stdout)
        return value if isinstance(value, list) else []
    except (OSError, subprocess.SubprocessError, ValueError):
        return []


def device_ip():
    interfaces = _ip_json("address", "show", "scope", "global")
    addresses = {}
    for interface in interfaces:
        name = interface.get("ifname", "")
        if ("UP" not in interface.get("flags", []) or "LOWER_UP" not in interface.get("flags", [])
                or name.startswith(("docker", "veth", "virbr", "br-", "tun", "tap"))):
            continue
        values = [item["local"] for item in interface.get("addr_info", [])
                  if item.get("family") == "inet" and _usable(item.get("local"))]
        if values:
            addresses[name] = values
    # Main-table default routes avoid choosing a VPN's policy-routing address.
    routes = sorted(_ip_json("route", "show", "default"), key=lambda route: route.get("metric", 0))
    for route in routes:
        values = addresses.get(route.get("dev"), [])
        if values:
            preferred = route.get("prefsrc")
            return preferred if preferred in values else values[0]
    if addresses:
        return next(iter(addresses.values()))[0]
    # Platforms without iproute2 can still resolve their local hostname.
    if not interfaces:
        try:
            for address in socket.gethostbyname_ex(socket.gethostname())[2]:
                if _usable(address):
                    return address
        except OSError:
            pass
    return "127.0.0.1"
