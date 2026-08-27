import zmq
import numpy as np

print("Connecting to QuadRF Daemon...")
context = zmq.Context()
sub = context.socket(zmq.SUB)

# Connect to the ultra-low latency IPC socket
sub.connect("ipc:///tmp/open_space_iq.ipc")

# Subscribe to everything
sub.setsockopt_string(zmq.SUBSCRIBE, "")

print("Listening for IQ data...")
try:
    for i in range(20):
        # Pull the zero-copy message
        msg = sub.recv()
        
        # Interpret the bytes as Complex Float32 (numpy complex64)
        data = np.frombuffer(msg, dtype=np.complex64)
        print(f"Chunk {i}: Received {len(data)} complex samples.")
        
except KeyboardInterrupt:
    print("\nTest finished.")
finally:
    sub.close()
    context.term()

