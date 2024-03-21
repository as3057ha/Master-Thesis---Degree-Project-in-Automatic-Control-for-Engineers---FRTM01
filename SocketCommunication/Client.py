import socket
import time

# Server's IP address and port
# SERVER_IP = '192.168.0.2' 
#SERVER_IP = '192.168.50.152' #Hemma
SERVER_IP = '10.8.121.248' #LTH
SERVER_PORT = 1025

def main():
    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Connect to the server
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print("Connected to server.")
    
    # Send a message to indicate readiness
    initial_message = "Ready to receive TCP_Vel"
    client_socket.sendall(initial_message.encode())

    TCP_Vel_message = "TCP_Vel received"
    
    # Continuously receive TCP velocity values
    try:
        while True:
            # Receive and print the message from the server
            received_message = client_socket.recv(1024).decode()
            # print(received_message)
            client_socket.sendall(TCP_Vel_message.encode())

            
            # Check if the received message is a shutdown signal
            if received_message == "Shutdown acknowledged":
                print("Shutdown signal received. Disconnecting...")
                break
            
            print("Received from server:", received_message)
            
            # Optional: send a response or keep-alive message back to the server
            # For example, acknowledging the received TCP velocity
            # client_socket.sendall("Ack".encode())
            
            # Optional: Introduce a slight delay if needed (e.g., rate limiting)
            # time.sleep(1)
            
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Close the socket
        client_socket.close()
        print("Disconnected from server.")

if __name__ == "__main__":
    main()
