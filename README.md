
# ESP32CAM QRCode Reader

## Introduction
This project enables the ESP32-CAM module to function as a QR code scanner. It captures QR codes using the camera, decodes them, and provides the decoded data to the user.

## Features
- Scans QR codes using the ESP32-CAM module.
- Provides decoded data from the scanned QR codes.
- Supports custom commands via HTTP requests.

## Setup
1. **WiFi Configuration**: Enter your WiFi network credentials in the code.
2. **Access Point Configuration**: Configure the access point (AP) credentials if needed.
3. **Pin Configuration**: Adjust the pin configuration according to your ESP32-CAM module.
4. **Flash Configuration**: Modify flash settings if required.
5. **Camera Configuration**: Adjust camera settings like resolution and quality.
6. **Custom Commands**: Define custom commands and their functionality in the code.
7. **Web Interface**: Access the web interface to interact with the ESP32-CAM module.

## Usage
- Connect to the ESP32-CAM module's WiFi network.
- Access the web interface through the assigned IP address.
- Scan QR codes using the camera.
- View the decoded data on the web interface.
- Execute custom commands via HTTP requests.

## Web Interface
The web interface provides the following functionalities:
- Toggle between light and dark mode.
- Adjust flash intensity.
- Capture still images.
- Display the scanned QR code and its decoded data.

## Dependencies
- Arduino
- esp_camera 
- WiFi 
- HTTPClient library
- quirc library



