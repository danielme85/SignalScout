/**
 * SignalScout Log Parser for JavaScript/Node.js
 *
 * Parses WiFi, BLE, and Zigbee scan data from SignalScout log files
 */

const fs = require('fs');
const readline = require('readline');

class SignalScoutParser {
    /**
     * Parse a log file and return structured data
     *
     * @param {string} filename - Path to the log file
     * @returns {Promise<Object>} Object with 'wifi', 'ble', and 'zigbee' arrays
     */
    static async parseLogFile(filename) {
        return new Promise((resolve, reject) => {
            const results = {
                wifi: [],
                ble: [],
                zigbee: []
            };

            const fileStream = fs.createReadStream(filename);
            const rl = readline.createInterface({
                input: fileStream,
                crlfDelay: Infinity
            });

            rl.on('line', (line) => {
                line = line.trim();

                // Skip header lines, comments, and empty lines
                if (!line || line.startsWith('Type,') || line.startsWith('#')) {
                    return;
                }

                const data = this._parseCSVLine(line);

                if (data.length < 9) {
                    return; // Invalid line
                }

                const type = data[0].toUpperCase();

                switch (type) {
                    case 'WIFI':
                        results.wifi.push(this._parseWiFiEntry(data));
                        break;
                    case 'BLE':
                        results.ble.push(this._parseBLEEntry(data));
                        break;
                    case 'ZIGBEE':
                        results.zigbee.push(this._parseZigbeeEntry(data));
                        break;
                }
            });

            rl.on('close', () => {
                resolve(results);
            });

            rl.on('error', (error) => {
                reject(error);
            });
        });
    }

    /**
     * Parse a CSV line (simple implementation)
     */
    static _parseCSVLine(line) {
        const result = [];
        let current = '';
        let inQuotes = false;

        for (let i = 0; i < line.length; i++) {
            const char = line[i];

            if (char === '"') {
                inQuotes = !inQuotes;
            } else if (char === ',' && !inQuotes) {
                result.push(current);
                current = '';
            } else {
                current += char;
            }
        }

        result.push(current);
        return result;
    }

    /**
     * Parse WiFi entry
     */
    static _parseWiFiEntry(data) {
        return {
            type: 'WIFI',
            fingerprint: data[1],
            timestamp: data[2],
            latitude: parseFloat(data[3]),
            longitude: parseFloat(data[4]),
            altitude: parseFloat(data[5]),
            satellites: parseInt(data[6]),
            hdop: parseFloat(data[7]),
            ssid: data[8],
            bssid: data[9],
            rssi: parseInt(data[10]),
            channel: parseInt(data[11]),
            band: data[12],
            encryption: data[13]
        };
    }

    /**
     * Parse BLE entry
     */
    static _parseBLEEntry(data) {
        return {
            type: 'BLE',
            fingerprint: data[1],
            timestamp: data[2],
            latitude: parseFloat(data[3]),
            longitude: parseFloat(data[4]),
            altitude: parseFloat(data[5]),
            satellites: parseInt(data[6]),
            hdop: parseFloat(data[7]),
            name: data[8],
            address: data[9],
            rssi: parseInt(data[10]),
            manufacturerData: data[11],
            serviceUuid: data[12]
        };
    }

    /**
     * Parse Zigbee entry
     */
    static _parseZigbeeEntry(data) {
        return {
            type: 'ZIGBEE',
            fingerprint: data[1],
            timestamp: data[2],
            latitude: parseFloat(data[3]),
            longitude: parseFloat(data[4]),
            altitude: parseFloat(data[5]),
            satellites: parseInt(data[6]),
            hdop: parseFloat(data[7]),
            panId: data[8],
            extendedPanId: data[9],
            channel: parseInt(data[10]),
            permitJoin: data[11] === 'Yes',
            routerCapacity: data[12] === 'Yes',
            endDeviceCapacity: data[13] === 'Yes'
        };
    }
}

// Example usage
async function main() {
    try {
        // Replace with your actual log file path
        const logFile = process.argv[2] || 'scan_20260121_153045.txt';

        const results = await SignalScoutParser.parseLogFile(logFile);

        console.log('Parsed Results:');
        console.log(`WiFi networks: ${results.wifi.length}`);
        console.log(`BLE devices: ${results.ble.length}`);
        console.log(`Zigbee networks: ${results.zigbee.length}\n`);

        // Display first WiFi entry as example
        if (results.wifi.length > 0) {
            console.log('First WiFi entry:');
            console.log(JSON.stringify(results.wifi[0], null, 2));
        }

        // Display first BLE entry as example
        if (results.ble.length > 0) {
            console.log('\nFirst BLE entry:');
            console.log(JSON.stringify(results.ble[0], null, 2));
        }

        // Display first Zigbee entry as example
        if (results.zigbee.length > 0) {
            console.log('\nFirst Zigbee entry:');
            console.log(JSON.stringify(results.zigbee[0], null, 2));
        }

    } catch (error) {
        console.error('Error:', error.message);
        process.exit(1);
    }
}

// Export for use as a module
module.exports = SignalScoutParser;

// Run main function if executed directly
if (require.main === module) {
    main();
}
