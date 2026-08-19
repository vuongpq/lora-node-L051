// ChirpStack payload decoder for IoT_Node uplinks (FPort 2, 14 bytes, big-endian).
//
// Byte layout:
//   0     uint8   battery level (0=external power, 1-254=level, 255=unknown)
//   1-2   uint16  bus voltage from INA219 (mV)
//   3-4   int16   current from INA219 (mA)
//   5-6   int16   temperature from DHT22 (tenths of a degree C)
//   7-8   uint16  relative humidity from DHT22 (tenths of a percent)
//   9-10  uint16  LDR raw ADC reading (0-4095)
//   11-12 uint16  soil moisture raw ADC reading (0-4095)
//   13    uint8   idle current from INA219, sampled just before the sensor
//                  cluster powers on for this cycle (0.1 mA/bit, 0-25.5 mA).
//                  MCU is still in full RUN mode when sampled (no STOP mode
//                  implemented yet), so this is a "peripherals idle" reading,
//                  not a true low-power sleep current.
//
// frameCounter/uptime are intentionally omitted: LoRaWAN already tracks the
// uplink sequence via FCntUp, and ChirpStack shows it without needing this
// in the application payload.

function readUint16BE(bytes, offset) {
  return (bytes[offset] << 8) | bytes[offset + 1];
}

function readInt16BE(bytes, offset) {
  var value = readUint16BE(bytes, offset);
  return value > 0x7fff ? value - 0x10000 : value;
}

function decode(bytes) {
  if (bytes.length < 14) {
    return { error: "payload too short, expected 14 bytes, got " + bytes.length };
  }

  return {
    batteryLevel: bytes[0],
    busVoltage_mV: readUint16BE(bytes, 1),
    current_mA: readInt16BE(bytes, 3),
    temperature_C: readInt16BE(bytes, 5) / 10,
    humidity_percent: readUint16BE(bytes, 7) / 10,
    light_raw: readUint16BE(bytes, 9),
    soilMoisture_raw: readUint16BE(bytes, 11),
    idleCurrent_mA: bytes[13] / 10,
  };
}


// ChirpStack v4 codec API
function decodeUplink(input) {
  return { data: decode(input.bytes) };
}

// ChirpStack v3 codec API (kept for compatibility)
function Decode(fPort, bytes) {
  return decode(bytes);
}
