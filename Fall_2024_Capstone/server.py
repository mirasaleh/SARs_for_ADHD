import cv2
import socket
import struct
import pickle
import threading

class VideoStreamServer:
    def __init__(self, address='0.0.0.0', port_num=3000, camera_id=6):
        self.address = address
        self.port_num = port_num
        self.capture_device = cv2.VideoCapture(camera_id)

        if not self.capture_device.isOpened():
            raise RuntimeError("Failed to access camera")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.address, self.port_num))
        self.socket.listen(5)

    def run(self):
        print(f"Server is running on {self.address}:{self.port_num}")
        try:
            while True:
                client_connection, client_info = self.socket.accept()
                print(f"Accepted connection from {client_info}")
                threading.Thread(target=self._handle_client, args=(client_connection,)).start()
        except KeyboardInterrupt:
            print("Server stopped manually.")
        finally:
            self._shutdown()

    def _handle_client(self, client_connection):
        try:
            while True:
                is_frame_captured, frame = self.capture_device.read()
                if not is_frame_captured:
                    break

                # Serialize frame data
                serialized_data = pickle.dumps(frame)
                # Send the length and the frame
                message_header = struct.pack("Q", len(serialized_data))
                client_connection.sendall(message_header + serialized_data)
        except Exception as err:
            print(f"Error during frame transmission: {err}")
        finally:
            client_connection.close()

    def _shutdown(self):
        self.capture_device.release()
        self.socket.close()
        print("Server resources have been released.")

if __name__ == "__main__":
    video_server = VideoStreamServer(address='127.0.0.1', port_num=3000, camera_id=6)
    video_server.run()

