import serial
import struct

SYNC_BYTE    = ord('#')
PAYLOAD_SIZE = 8
PACKET_SIZE  = 1 + PAYLOAD_SIZE + 1  # 10 bytes

def compute_checksum(payload: bytes) -> int:
    checksum = 0
    for b in payload:
        checksum ^= b
    return checksum

def decode_packets(port: str, baud: int = 115200):
    with serial.Serial(port, baud, timeout=2) as ser:
        print(f"Listening on {port} at {baud} baud...\n")

        while True:
            # scan till '#' byte
            byte = ser.read(1)
            if not byte:
                continue
            if byte[0] != SYNC_BYTE:
                print(f"[SYNC LOST] Got 0x{byte[0]:02X}, re-syncing...")
                continue

            # Read the rest of the packet 
            rest = ser.read(PAYLOAD_SIZE + 1)
            if len(rest) < PAYLOAD_SIZE + 1:
                print("[ERROR] Incomplete packet, skipping...")
                continue

            payload  = rest[:PAYLOAD_SIZE]
            received_checksum = rest[PAYLOAD_SIZE]

            # Verify checksum
            expected_checksum = compute_checksum(payload)
            if received_checksum != expected_checksum:
                print(f"[CHECKSUM FAIL] Expected 0x{expected_checksum:02X}, "
                      f"got 0x{received_checksum:02X}")
                continue

            samples = struct.unpack('<4H', payload)

            print(f"S0: {samples[0]:5d}  S1: {samples[1]:5d}  "
                  f"S2: {samples[2]:5d}  S3: {samples[3]:5d}")

if __name__ == "__main__":
    decode_packets(port="COM4", baud=115200)
