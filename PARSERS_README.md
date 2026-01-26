# SignalScout Log Parsers

This directory contains parser examples in three languages to read and parse SignalScout log files into structured arrays.

## Available Parsers

- `parse_log.php` - PHP parser
- `parse_log.py` - Python parser
- `parse_log.js` - JavaScript/Node.js parser

## Output Format

All parsers return a structured object/dictionary with three arrays:

```
{
  "wifi": [
    {
      "type": "WIFI",
      "fingerprint": "3C7B6E95",
      "timestamp": "2026-01-24 22:57:04",
      "latitude": 41.342822,
      "longitude": -81.389317,
      "altitude": 327.20,
      "satellites": 8,
      "hdop": 1.34,
      "ssid": "MyHomeNetwork",
      "bssid": "60:B7:6E:6D:99:95",
      "rssi": -45,
      "channel": 6,
      "band": "2.4GHz",
      "encryption": "WPA2-PSK"
    }
  ],
  "ble": [
    {
      "type": "BLE",
      "fingerprint": "FA2FAF58",
      "timestamp": "2026-01-24 22:57:16",
      "latitude": 41.342820,
      "longitude": -81.389308,
      "altitude": 327.90,
      "satellites": 8,
      "hdop": 1.34,
      "name": "Smart Watch",
      "address": "58:D9:FA:AF:2F:FD",
      "rssi": -65,
      "manufacturerData": "4C001005",
      "serviceUuid": "0000180A"
    }
  ],
  "zigbee": [
    {
      "type": "ZIGBEE",
      "fingerprint": "8A3F5C12",
      "timestamp": "2026-01-24 22:57:28",
      "latitude": 41.342818,
      "longitude": -81.389299,
      "altitude": 328.10,
      "satellites": 8,
      "hdop": 1.34,
      "panId": "0x1A2B",
      "extendedPanId": "00:11:22:33:44:55:66:77",
      "channel": 15,
      "permitJoin": true,
      "routerCapacity": true,
      "endDeviceCapacity": false
    }
  ]
}
```

## Usage Examples

### PHP

```bash
# Run directly
php parse_log.php

# Or specify a file
php parse_log.php scan_20260121_153045.txt

# Or use as a class
<?php
require_once 'parse_log.php';

$results = SignalScoutParser::parseLogFile('scan_20260121_153045.txt');

echo "Found " . count($results['wifi']) . " WiFi networks\n";
echo "Found " . count($results['ble']) . " BLE devices\n";
echo "Found " . count($results['zigbee']) . " Zigbee networks\n";

// Access individual entries
foreach ($results['wifi'] as $wifi) {
    echo "SSID: {$wifi['ssid']}, RSSI: {$wifi['rssi']} dBm\n";
}
```

### Python

```bash
# Run directly
python3 parse_log.py

# Or specify a file
python3 parse_log.py scan_20260121_153045.txt

# Or use as a module
from parse_log import SignalScoutParser

results = SignalScoutParser.parse_log_file('scan_20260121_153045.txt')

print(f"Found {len(results['wifi'])} WiFi networks")
print(f"Found {len(results['ble'])} BLE devices")
print(f"Found {len(results['zigbee'])} Zigbee networks")

# Access individual entries
for wifi in results['wifi']:
    print(f"SSID: {wifi['ssid']}, RSSI: {wifi['rssi']} dBm")
```

### JavaScript (Node.js)

```bash
# Run directly
node parse_log.js

# Or specify a file
node parse_log.js scan_20260121_153045.txt

# Or use as a module
const SignalScoutParser = require('./parse_log');

(async () => {
    const results = await SignalScoutParser.parseLogFile('scan_20260121_153045.txt');

    console.log(`Found ${results.wifi.length} WiFi networks`);
    console.log(`Found ${results.ble.length} BLE devices`);
    console.log(`Found ${results.zigbee.length} Zigbee networks`);

    // Access individual entries
    results.wifi.forEach(wifi => {
        console.log(`SSID: ${wifi.ssid}, RSSI: ${wifi.rssi} dBm`);
    });
})();
```

## Features

All parsers include:

- CSV parsing with proper handling of quoted fields
- Automatic type conversion (strings to numbers, booleans)
- Skipping of header lines and comments
- Error handling for missing files
- Support for all three device types (WiFi, BLE, Zigbee)
- Clean, structured output format

## Data Types

### Common Fields (All Types)
- `type`: String - Entry type ("WIFI", "BLE", or "ZIGBEE")
- `fingerprint`: String - 8-character hex identifier
- `timestamp`: String - UTC timestamp
- `latitude`: Float - GPS latitude
- `longitude`: Float - GPS longitude
- `altitude`: Float - Elevation in meters
- `satellites`: Integer - GPS satellite count
- `hdop`: Float - GPS accuracy indicator

### WiFi-Specific Fields
- `ssid`: String - Network name
- `bssid`: String - MAC address
- `rssi`: Integer - Signal strength in dBm
- `channel`: Integer - WiFi channel
- `band`: String - "2.4GHz" or "5GHz"
- `encryption`: String - Security type

### BLE-Specific Fields
- `name`: String - Device name
- `address`: String - Bluetooth MAC address
- `rssi`: Integer - Signal strength in dBm
- `manufacturerData`: String - Hex manufacturer data
- `serviceUuid`: String - Primary service UUID

### Zigbee-Specific Fields
- `panId`: String - 16-bit PAN identifier
- `extendedPanId`: String - 64-bit unique network ID
- `channel`: Integer - Zigbee channel (11-26)
- `permitJoin`: Boolean - Network accepts new devices
- `routerCapacity`: Boolean - Can accept router devices
- `endDeviceCapacity`: Boolean - Can accept end devices

## Requirements

- **PHP**: PHP 7.0 or higher
- **Python**: Python 3.6 or higher
- **JavaScript**: Node.js 10 or higher

No external dependencies required for any parser.
