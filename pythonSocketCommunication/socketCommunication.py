import socket
import struct

def unpack_float(raw_data):
    # Unpacks the received raw data into a float
    return struct.unpack('f', raw_data)[0]

def main():
    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Connect to the server
    client_socket.connect(('192.168.125.1', 55555))
    print("Connected")
    
    try:
        while True:
            # Receive raw data from the server
            raw_data = client_socket.recv(4)
            if not raw_data:
                break
            
            # Unpack the raw data into a float
            received_float = unpack_float(raw_data)
            
            # Print the received float
            print("Received Float:", received_float)
    
    finally:
        # Close the socket connection
        client_socket.close()

if __name__ == "__main__":
    main()
