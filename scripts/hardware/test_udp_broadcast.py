"""Listen for UDP broadcast from run_interface and print the data. Ctrl+C to quit."""
import socket
import struct

PORT = 9870
FIELDS = (
    [f"act_{i}"  for i in range(10)] +
    [f"pos_{i}"  for i in range(10)] +
    [f"vel_{i}"  for i in range(10)] +
    [f"tau_{i}"  for i in range(10)] +
    ["rpy_r", "rpy_p", "rpy_y"] +
    ["rate_r", "rate_p", "rate_y"] +
    ["acc_x", "acc_y", "acc_z"] +
    ["quat_w", "quat_x", "quat_y", "quat_z"]
)  # 53 fields total

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("", PORT))
print(f"Listening on UDP port {PORT}... (Ctrl+C to quit)\n")

count = 0
try:
    while True:
        data, addr = sock.recvfrom(65535)
        values = [float(x) for x in data.decode().split(',')]
        count += 1
        print(f"\n--- Packet #{count} from {addr[0]} ({len(values)} fields) ---")
        for name, val in zip(FIELDS, values):
            print(f"  {name:10s}: {val:+.4f}")
except KeyboardInterrupt:
    print("\nDone.")
finally:
    sock.close()
