<?php
/**
 * SignalScout Log Parser for PHP
 *
 * Parses WiFi, BLE, and Zigbee scan data from SignalScout log files
 */

class SignalScoutParser {

    /**
     * Parse a log file and return structured data
     *
     * @param string $filename Path to the log file
     * @return array Associative array with 'wifi', 'ble', and 'zigbee' entries
     * @throws Exception
     */
    public static function parseLogFile($filename) {
        if (!file_exists($filename)) {
            throw new Exception("Log file not found: $filename");
        }

        $results = [
            'wifi' => [],
            'ble' => [],
            'zigbee' => []
        ];

        $lines = file($filename, FILE_IGNORE_NEW_LINES | FILE_SKIP_EMPTY_LINES);

        foreach ($lines as $line) {
            // Skip header lines and comments
            if (str_starts_with($line, 'Type,') || str_starts_with($line, '#') || trim($line) === '') {
                continue;
            }

            $data = str_getcsv($line, ',', '"', '');

            if (count($data) < 9) {
                continue; // Invalid line
            }

            $type = strtoupper($data[0]);

            switch ($type) {
                case 'WIFI':
                    $results['wifi'][] = self::parseWiFiEntry($data);
                    break;
                case 'BLE':
                    $results['ble'][] = self::parseBLEEntry($data);
                    break;
                case 'ZIGBEE':
                    $results['zigbee'][] = self::parseZigbeeEntry($data);
                    break;
            }
        }

        return $results;
    }

    /**
     * Parse WiFi entry
     */
    private static function parseWiFiEntry($data): array
    {
        return [
            'type' => 'WIFI',
            'fingerprint' => $data[1],
            'timestamp' => $data[2],
            'latitude' => floatval($data[3]),
            'longitude' => floatval($data[4]),
            'altitude' => floatval($data[5]),
            'satellites' => intval($data[6]),
            'hdop' => floatval($data[7]),
            'ssid' => $data[8],
            'bssid' => $data[9],
            'rssi' => intval($data[10]),
            'channel' => intval($data[11]),
            'band' => $data[12],
            'encryption' => $data[13]
        ];
    }

    /**
     * Parse BLE entry
     */
    private static function parseBLEEntry($data): array
    {
        return [
            'type' => 'BLE',
            'fingerprint' => $data[1],
            'timestamp' => $data[2],
            'latitude' => floatval($data[3]),
            'longitude' => floatval($data[4]),
            'altitude' => floatval($data[5]),
            'satellites' => intval($data[6]),
            'hdop' => floatval($data[7]),
            'name' => $data[8],
            'address' => $data[9],
            'rssi' => intval($data[10]),
            'manufacturer_data' => $data[11],
            'service_uuid' => $data[12]
        ];
    }

    /**
     * Parse Zigbee entry
     */
    private static function parseZigbeeEntry($data): array
    {
        return [
            'type' => 'ZIGBEE',
            'fingerprint' => $data[1],
            'timestamp' => $data[2],
            'latitude' => floatval($data[3]),
            'longitude' => floatval($data[4]),
            'altitude' => floatval($data[5]),
            'satellites' => intval($data[6]),
            'hdop' => floatval($data[7]),
            'pan_id' => $data[8],
            'extended_pan_id' => $data[9],
            'channel' => intval($data[10]),
            'permit_join' => $data[11] === 'Yes',
            'router_capacity' => $data[12] === 'Yes',
            'end_device_capacity' => $data[13] === 'Yes'
        ];
    }
}

// Example usage
if (basename(__FILE__) == basename($_SERVER['PHP_SELF'])) {
    try {
        // Get filename from the CLI argument or use default
        $logFile = $argc > 1 ? $argv[1] : 'scan_20260121_153045.txt';

        $results = SignalScoutParser::parseLogFile($logFile);

        echo "Parsed Results:\n";
        echo "WiFi networks: " . count($results['wifi']) . "\n";
        echo "BLE devices: " . count($results['ble']) . "\n";
        echo "Zigbee networks: " . count($results['zigbee']) . "\n\n";

        // Display first WiFi entry as example
        if (!empty($results['wifi'])) {
            echo "First WiFi entry:\n";
            print_r($results['wifi'][0]);
        }

        // Display first BLE entry as example
        if (!empty($results['ble'])) {
            echo "\nFirst BLE entry:\n";
            print_r($results['ble'][0]);
        }

        // Display first Zigbee entry as example
        if (!empty($results['zigbee'])) {
            echo "\nFirst Zigbee entry:\n";
            print_r($results['zigbee'][0]);
        }

    } catch (Exception $e) {
        echo "Error: " . $e->getMessage() . "\n";
    }
}
