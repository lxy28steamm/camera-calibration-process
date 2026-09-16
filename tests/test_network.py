import os
import threading
import unittest
from unittest.mock import patch

from ego_calibration.network import device_ip
from ego_calibration.webapp import CameraWebServer, existing_workbench


def interface(name, address, active=True):
    return {"ifname": name, "flags": ["UP", "LOWER_UP"] if active else ["UP"],
            "addr_info": [{"family": "inet", "local": address}]}


class DeviceAddressTest(unittest.TestCase):
    def test_default_route_uses_running_machine_not_docker_or_vpn(self):
        interfaces = [interface("Mihomo", "198.18.0.1"), interface("docker0", "172.17.0.1"),
                      interface("wlan0", "192.168.124.12"), interface("eth0", "10.0.0.20")]
        routes = [{"dev": "Mihomo", "metric": 0}, {"dev": "eth0", "metric": 100},
                  {"dev": "wlan0", "metric": 200}]
        with patch("ego_calibration.network._ip_json", side_effect=[interfaces, routes]):
            self.assertEqual(device_ip(), "10.0.0.20")
        with patch("ego_calibration.network._ip_json", side_effect=[interfaces, [routes[2]]]):
            self.assertEqual(device_ip(), "192.168.124.12")

    def test_static_lan_without_default_gateway_is_usable(self):
        with patch("ego_calibration.network._ip_json", side_effect=[[interface("eth0", "192.168.5.9")], []]):
            self.assertEqual(device_ip(), "192.168.5.9")

    def test_disconnected_and_link_local_interfaces_do_not_become_entry_points(self):
        interfaces = [interface("eth0", "192.168.1.9", False), interface("eth1", "169.254.1.9")]
        with patch("ego_calibration.network._ip_json", side_effect=[interfaces, []]):
            self.assertEqual(device_ip(), "127.0.0.1")

    def test_missing_iproute2_uses_hostname_or_local_fallback(self):
        with patch("ego_calibration.network._ip_json", return_value=[]), patch("socket.gethostbyname_ex", return_value=("host", [], ["127.0.1.1", "10.1.2.3"])):
            self.assertEqual(device_ip(), "10.1.2.3")
        with patch("ego_calibration.network._ip_json", return_value=[]), patch("socket.gethostbyname_ex", side_effect=OSError):
            self.assertEqual(device_ip(), "127.0.0.1")

    def test_existing_instance_check_uses_selected_address_without_proxy(self):
        server = CameraWebServer(("127.0.0.1", 0), object())
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            url = f"http://127.0.0.1:{server.server_port}"
            with patch.dict(os.environ, {"http_proxy": "http://127.0.0.1:1", "no_proxy": ""}):
                self.assertTrue(existing_workbench(url))
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == "__main__":
    unittest.main()
