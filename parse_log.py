#!/usr/bin/env python3
"""
SignalScout Log Parser for Python

Parses WiFi, BLE, and Zigbee scan data from SignalScout log files
"""

import csv
from typing import Dict, List, Union


class SignalScoutParser:
    """Parser for SignalScout log files"""

    @staticmethod
    def parse_log_file(filename: str) -> Dict[str, List[Dict]]:
        """
        Parse a log file and return structured data

        Args:
            filename: Path to the log file

        Returns:
            Dictionary with 'wifi', 'ble', and 'zigbee' lists
        """
        results = {
            'wifi': [],
            'ble': [],
            'zigbee': []
        }

        try:
            with open(filename, 'r', encoding='utf-8') as file:
                reader = csv.reader(file)

                for row in reader:
                    # Skip header lines, comments, and empty lines
                    if not row or row[0].startswith('Type') or row[0].startswith('#'):
                        continue

                    if len(row) < 9:
                        continue  # Invalid line

                    entry_type = row[0].upper()

                    if entry_type == 'WIFI':
                        results['wifi'].append(SignalScoutParser._parse_wifi_entry(row))
                    elif entry_type == 'BLE':
                        results['ble'].append(SignalScoutParser._parse_ble_entry(row))
                    elif entry_type == 'ZIGBEE':
                        results['zigbee'].append(SignalScoutParser._parse_zigbee_entry(row))

        except FileNotFoundError:
            raise FileNotFoundError(f"Log file not found: {filename}")

        return results

    @staticmethod
    def _parse_wifi_entry(row: List[str]) -> Dict[str, Union[str, int, float]]:
        """Parse WiFi entry"""
        return {
            'type': 'WIFI',
            'fingerprint': row[1],
            'timestamp': row[2],
            'latitude': float(row[3]),
            'longitude': float(row[4]),
            'altitude': float(row[5]),
            'satellites': int(row[6]),
            'hdop': float(row[7]),
            'ssid': row[8],
            'bssid': row[9],
            'rssi': int(row[10]),
            'channel': int(row[11]),
            'band': row[12],
            'encryption': row[13]
        }

    @staticmethod
    def _parse_ble_entry(row: List[str]) -> Dict[str, Union[str, int, float]]:
        """Parse BLE entry"""
        return {
            'type': 'BLE',
            'fingerprint': row[1],
            'timestamp': row[2],
            'latitude': float(row[3]),
            'longitude': float(row[4]),
            'altitude': float(row[5]),
            'satellites': int(row[6]),
            'hdop': float(row[7]),
            'name': row[8],
            'address': row[9],
            'rssi': int(row[10]),
            'manufacturer_data': row[11],
            'service_uuid': row[12]
        }

    @staticmethod
    def _parse_zigbee_entry(row: List[str]) -> Dict[str, Union[str, int, float, bool]]:
        """Parse Zigbee entry"""
        return {
            'type': 'ZIGBEE',
            'fingerprint': row[1],
            'timestamp': row[2],
            'latitude': float(row[3]),
            'longitude': float(row[4]),
            'altitude': float(row[5]),
            'satellites': int(row[6]),
            'hdop': float(row[7]),
            'pan_id': row[8],
            'extended_pan_id': row[9],
            'channel': int(row[10]),
            'permit_join': row[11] == 'Yes',
            'router_capacity': row[12] == 'Yes',
            'end_device_capacity': row[13] == 'Yes'
        }


def main():
    """Example usage"""
    import sys

    # Replace with your actual log file path
    log_file = 'scan_20260121_153045.txt'

    if len(sys.argv) > 1:
        log_file = sys.argv[1]

    try:
        results = SignalScoutParser.parse_log_file(log_file)

        print("Parsed Results:")
        print(f"WiFi networks: {len(results['wifi'])}")
        print(f"BLE devices: {len(results['ble'])}")
        print(f"Zigbee networks: {len(results['zigbee'])}\n")

        # Display first WiFi entry as example
        if results['wifi']:
            print("First WiFi entry:")
            for key, value in results['wifi'][0].items():
                print(f"  {key}: {value}")

        # Display first BLE entry as example
        if results['ble']:
            print("\nFirst BLE entry:")
            for key, value in results['ble'][0].items():
                print(f"  {key}: {value}")

        # Display first Zigbee entry as example
        if results['zigbee']:
            print("\nFirst Zigbee entry:")
            for key, value in results['zigbee'][0].items():
                print(f"  {key}: {value}")

    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
