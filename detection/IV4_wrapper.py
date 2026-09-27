import socket
import os
import cv2
import numpy as np
from datetime import datetime
import time

class IV4Camera:
    def __init__(self, ip, port=8500, log_folder="logs"):
        """
        Initialize the IV4 camera.
        :param ip: IP address of the camera
        :param port: Communication port of the camera (default: 8500)
        :param log_folder: Folder in which the log file is saved
        """
        self.ip = ip
        self.port = port
        self.log_folder = log_folder

        # Create the log folder if it does not exist
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)

    def _send_command(self, command):
        """Internal method: send a command to the camera and return the response."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)  # 5-second timeout
            s.connect((self.ip, self.port))
            s.sendall(command.encode())
            response = s.recv(1024)
            return response.decode().strip()

    def switch_program(self, program_number):
        """Switch the camera program."""
        command = f"PW,{program_number:03d}\r"
        response = self._send_command(command)
        if response == "PW":
            return f"Program {program_number} switched successfully."
        else:
            return f"Failed to switch program: {response}"

    def trigger_capture(self):
        """Trigger a capture."""
        command = "T1\r"  # trigger command
        response = self._send_command(command)
        if response == "T1":
            return "Capture triggered successfully."
        else:
            return f"Failed to trigger capture: {response}"

    def get_result_status(self):
        """Get the status of the capture result."""
        command = "RS\r"  # result status command
        return self._send_command(command)

    def get_image(self,timeoutsec=10,compress=False):
        """Get the captured image (binary data)."""
        command = f"BR,{int(compress)}\r"  # image data command (compression flag)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,65536)
                s.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,65536)
                s.settimeout(timeoutsec)  # timeout in seconds (default: 10)
                s.connect((self.ip, self.port))
                s.sendall(command.encode())

                header = s.recv(1024)
                if header==b'ER,BR,03\r':
                    print("Error")
                    return None
                header_parts = header.split(b",")
                if len(header_parts)<3:
                    print("Invalid header format.")
                    return None
                data_length = int(header_parts[2].strip())
                # print(f"Expected data length: {data_length} bytes")

                image_data = header[22:]
                total_received = len(image_data)
                while total_received < data_length:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    image_data+=chunk
                    total_received +=len(chunk)
                    # print(f"Received {len(chunk)} bytes, Total: {total_received}/{data_length}")
                if total_received >= data_length:
                    # print("Successfully received all image data.")
                    return image_data
                else:
                    print("Failed to receive full image data.")
                    return None

                return image_data
        except socket.timeout:
            print("Image retrieval timed out.")
            return None
        except Exception as e:
            print(f"Error while receiving image: {e}")
            return None

    def save_image(self, image_data, filename):
        """Save the captured image as a binary file under the given file name."""
        image_path = os.path.join(filename)
        with open(image_path, 'wb') as image_file:
            image_file.write(image_data)
        return image_path

    def check_serial_port(self):
        """Check the serial port status."""
        command = "SP\r"
        return self._send_command(command)

    def check_sd_card(self):
        """Check the SD card status."""
        command = "SD\r"
        return self._send_command(command)

    def get_device_info(self):
        """Get basic device information."""
        command = "DI\r"
        return self._send_command(command)

    def get_error_info(self):
        """Get error information."""
        command = "ER\r"
        return self._send_command(command)

    def log_to_file(self, message):
        """Append a message to the log file."""
        log_file_path = os.path.join(self.log_folder, "camera_log.txt")
        with open(log_file_path, 'a') as log_file:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            log_file.write(f"[{timestamp}] {message}\n")

    def capture_and_log(self, save_dir, image_format='jpg'):
        """
        Capture, write the result to the log file, and save the image.
        """
        # 1. Trigger a capture
        capture_result = self.trigger_capture()
        self.log_to_file(capture_result)

        # 2. Get the capture result status and log it
        result_status = self.get_result_status()
        self.log_to_file(f"Capture result status: {result_status}")

        # 3. Save the image
        image_data = self.get_image()
        if image_data:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            image_path = os.path.join(save_dir, f"captured_image_{timestamp}.{image_format}")
            self.save_image(image_data, image_path)
            self.log_to_file(f"Image saved at: {image_path}")
            return 0
        else:
            self.log_to_file("Failed to capture image.")
            return 1

    def switch_program_and_capture(self, program_number, save_dir, image_format='jpg'):
        """
        Switch the program, then capture and log the result.
        """
        # 1. Switch the program
        switch_result = self.switch_program(program_number)
        self.log_to_file(switch_result)

        # 2. Capture, log, and save the image
        self.capture_and_log(save_dir, image_format)

    def display_realtime_images(self, interval=0.5):
        """
        Capture continuously and display the images (pseudo real time).
        :param interval: Wait time between captures in seconds
        """
        while True:
            # 1. Trigger a capture
            capture_result = self.trigger_capture()
            if "Failed" in capture_result:
                print(capture_result)
                break

            # 2. Get the image
            image_data = self.get_image()
            if not image_data:
                print("Failed to retrieve image.")
                break

            # 3. Convert the binary data into an OpenCV image
            np_array = np.frombuffer(image_data, dtype=np.uint8)
            img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

            # 4. Show the image
            if img is not None:
                cv2.imshow("Camera Image", img)

            # Quit when the "q" key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Wait for the given interval before the next capture
            time.sleep(interval)

        cv2.destroyAllWindows()

# Usage example
if __name__ == "__main__":
    camera = IV4Camera(ip="192.168.1.100", log_folder="camera_logs")

    # (1) Capture, write the result to the log file, and save the image
    camera.capture_and_log(save_dir="images", image_format='jpg')

    # (2) Switch to program 1, then capture
    camera.switch_program_and_capture(program_number=1, save_dir="images", image_format='png')

    # (3) Display real-time images
    camera.display_realtime_images(interval=0.5)
