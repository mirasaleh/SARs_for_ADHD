import cv2
import socket
import struct
import pickle

class VideoStreamClient:
    def __init__(self, server_address='127.0.0.1', server_port=3000):
        self.server_address = server_address
        self.server_port = server_port
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.connect((self.server_address, self.server_port))
        self.buffer = b""
        self.header_size = struct.calcsize("Q")

    def receive_frame(self):
        while len(self.buffer) < self.header_size:
            chunk = self.connection.recv(4096)  # Receiving data in chunks
            if not chunk:
                return None
            self.buffer += chunk

        # Unpack the message length
        message_length = struct.unpack("Q", self.buffer[:self.header_size])[0]
        self.buffer = self.buffer[self.header_size:]

        # Ensure we have the full frame data
        while len(self.buffer) < message_length:
            self.buffer += self.connection.recv(4096)

        frame_data = self.buffer[:message_length]
        self.buffer = self.buffer[message_length:]
        frame = pickle.loads(frame_data)
        return frame

    def close(self):
        self.connection.close()

if __name__ == "__main__":
    client = VideoStreamClient(server_address='127.0.0.1', server_port=3000)

    try:
        while True:
            frame = client.receive_frame()
            if frame is None:
                print("No frame received. Exiting...")
                break

            # Display the received frame
            cv2.imshow("Video Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Client terminated.")
    finally:
        client.close_connection()
        cv2.destroyAllWindows()

