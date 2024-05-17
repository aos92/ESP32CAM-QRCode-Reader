/**
 * Format perintah kustom dapat diakses melalui tautan berikut: http://192.168.xxx.xxx/control?cmd=P1;P2;P3;P4;P5;P6;P7;P8;U9
 * 
 * Dapatkan APIP, STAIP http://192.168.xxx.xxx/?ip
 * Dapatkan Alamat MAC http://192.168.xxx.xxx/?mac
 * Output Digital http://192.168.xxx.xxx/?digitalwrite=pin;nilai
 * Output Analog http://192.168.xxx.xxx/?analogwrite=pin;nilai
 * Baca Digital http://192.168.xxx.xxx/?digitalread=pin
 * Baca Analog http://192.168.xxx.xxx/?analogread=pin
 * Baca Sentuhan http://192.168.xxx.xxx/?touchread=pin
 * Restart Daya http://192.168.xxx.xxx/?restart
 * Pengaturan Flash http://192.168.xxx.xxx/?flash=nilai (rentang nilai: 0~255)
 * Pengaturan Servo http://192.168.xxx.xxx/?servo=pin;nilai (rentang nilai: 0~180)
 * Pengaturan Relay http://192.168.xxx.xxx/?relay=pin;nilai (rentang nilai: 0 atau 1)
 * Pengaturan UART http://192.168.xxx.xxx/?uart=nilai
 * 
 * 
 * Konfigurasi esp32cam:
 * PSRAM: Enable
 * Partition Scheme: hugo APP (3MB No OTA)
 * CPU Frequency: 240MHz
 * Flash Mode: QIO
 * Flash Frequency: 80MHz
 * Flash Size: 4MB
 * Upload Speed: 115200
 * Core Debug Level: None
 * 
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h" // Mengimpor fungsi untuk kamera
#include "soc/soc.h" // Mengimpor pustaka untuk mencegah reboot karena ketidakstabilan daya
#include "soc/rtc_cntl_reg.h" // Mengimpor pustaka untuk mencegah reboot karena ketidakstabilan daya
#include "quirc.h"  // Mengimpor pustaka untuk pemrosesan kode QR

// Menginputkan SSID dan kata sandi WiFi
const char* ssid = "xxxxxxxx";  // Nama SSID jaringan WiFi
const char* password = "xxxxxxxx"; // Kata sandi jaringan WiFi

// Menginputkan SSID dan kata sandi AP (Access Point) http://192.168.4.1
const char* apssid = "xxxxxxxx"; // Nama SSID untuk Access Point
const char* appassword = "xxxxxxxx"; // Kata sandi AP harus minimal 8 karakter

String Feedback = "";   // Menginisialisasi string untuk pesan balik ke klien
// Parameter untuk perintah
String Command = "", cmd = "", P1 = "", P2 = "", P3 = "", P4 = "", P5 = "", P6 = "", P7 = "", P8 = "", P9 = "";
// Status untuk memecah perintah
byte ReceiveState = 0, cmdState = 1, strState = 1, questionstate = 0, equalstate = 0, semicolonstate = 0;

TaskHandle_t Task; // Mendeklarasikan handle untuk task atau tugas

// Mengatur pin pada modul ESP32CAM WROVER KIT
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Struktur untuk menyimpan data kode QR
struct QRCodeData
{
  bool valid;                 // Menyimpan status validitas kode QR
  int dataType;               // Menyimpan tipe data dari kode QR
  uint8_t payload[1024];      // Menyimpan data dari kode QR
  int payloadLen;             // Menyimpan panjang data dari kode QR
};

// Mendeklarasikan variabel untuk penggunaan pustaka quirc
struct quirc *q = NULL;       // Variabel untuk menyimpan struktur quirc
uint8_t *image = NULL;        // Variabel untuk menyimpan gambar
camera_fb_t *fb = NULL;       // Variabel untuk menyimpan frame buffer dari kamera
struct quirc_code code;       // Variabel untuk menyimpan kode yang didekodekan
struct quirc_data data;       // Variabel untuk menyimpan data yang didekodekan
quirc_decode_error_t err;     // Variabel untuk menyimpan kesalahan saat dekode
struct QRCodeData qrCodeData; // Variabel untuk menyimpan hasil data kode QR
String QRCodeResult = "";     // Variabel untuk menyimpan hasil kode QR dalam bentuk string

WiFiServer server(80); // Menginisialisasi server WiFi pada port 80
WiFiClient client;     // Mendeklarasikan klien WiFi

camera_config_t config; // Mendeklarasikan konfigurasi kamera


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Mematikan pengaturan reboot jika ada ketidakstabilan daya
  
  Serial.begin(115200);  // Memulai komunikasi serial dengan baud rate 115200
  Serial.setDebugOutput(true);  // Mengaktifkan output debug
  Serial.println();
  
  // Mengatur konfigurasi kamera
  config.ledc_channel = LEDC_CHANNEL_0;  // Mengatur channel LEDC
  config.ledc_timer = LEDC_TIMER_0;  // Mengatur timer LEDC
  config.pin_d0 = Y2_GPIO_NUM;  // Mengatur pin data kamera
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;  // Mengatur pin clock kamera
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;  // Mengatur frekuensi clock
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // Mengatur format piksel
  config.frame_size = FRAMESIZE_QVGA;  // Mengatur ukuran frame
  config.jpeg_quality = 15;  // Mengatur kualitas JPEG
  config.fb_count = 1;  // Mengatur jumlah frame buffer

  // Inisialisasi kamera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {  // Jika inisialisasi gagal
    Serial.printf("Camera init failed with error 0x%x", err);  // Mencetak pesan kesalahan
    ESP.restart();  // Merestart ESP
  }

  sensor_t * s = esp_camera_sensor_get();  // Mendapatkan sensor kamera
  s->set_framesize(s, FRAMESIZE_QVGA);  // Mengatur ukuran frame

  //s->set_vflip(s, 1);  // Membalikkan gambar secara vertikal
  //s->set_hmirror(s, 1);  // Membalikkan gambar secara horizontal
          
  // Mengatur pin untuk lampu kilat (GPIO2)
  ledcAttachPin(2, 4);  
  ledcSetup(4, 5000, 8); 
  
  WiFi.mode(WIFI_AP_STA);  // Mengatur mode WiFi, bisa juga menggunakan WiFi.mode(WIFI_AP); atau WiFi.mode(WIFI_STA);

  // Menentukan IP statis untuk klien
  // WiFi.config(IPAddress(192, 168, 201, 100), IPAddress(192, 168, 201, 2), IPAddress(255, 255, 255, 0));

  for (int i = 0; i < 2; i++) {
    WiFi.begin(ssid, password);  // Memulai koneksi WiFi
  
    delay(1000);
    Serial.println("");
    Serial.print("Connecting to ");
    Serial.println(ssid);
    
    long int StartTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if ((StartTime + 5000) < millis()) break;  // Menunggu koneksi selama 10 detik
    } 
  
    if (WiFi.status() == WL_CONNECTED) {  // Jika berhasil terkoneksi
      WiFi.softAP((WiFi.localIP().toString() + "_" + (String)apssid).c_str(), appassword);  // Menentukan SSID dan kata sandi AP         
      Serial.println("");
      Serial.println("STAIP address: ");
      Serial.println(WiFi.localIP());
      Serial.println("");
  
      for (int i = 0; i < 5; i++) {  // Jika terkoneksi, lampu kilat berkedip cepat
        ledcWrite(2, 10);
        delay(200);
        ledcWrite(2, 0);
        delay(200);    
      }
      break;
    }
  } 

  if (WiFi.status() != WL_CONNECTED) {  // Jika koneksi gagal
    WiFi.softAP((WiFi.softAPIP().toString() + "_" + (String)apssid).c_str(), appassword);         

    for (int i = 0; i < 2; i++) {  // Jika tidak terkoneksi, lampu kilat berkedip lambat
      ledcWrite(2, 10);
      delay(1000);
      ledcWrite(2, 0);
      delay(1000);    
    }
  } 
  
  // Menentukan IP untuk AP
  // WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0)); 
  Serial.println("");
  Serial.println("APIP address: ");
  Serial.println(WiFi.softAPIP());  
  Serial.println("");
  
  // Mengatur lampu kilat ke posisi low
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW); 

  server.begin();  // Memulai server WiFi

  // Membuat dan menugaskan task ke core 0
  xTaskCreatePinnedToCore(
             QRCodeReader,  /* Fungsi task */
             "Task",  /* Nama task */
             10000,  /* Ukuran stack task */
             NULL,  /* Parameter task */
             1,  /* Prioritas task */
             &Task,  /* Handle task untuk melacak task yang dibuat */
             0);  /* Pin task ke core 0 */

  Serial.print("listenConnection running on core ");
  Serial.println(xPortGetCoreID());  // Mencetak core ID yang menjalankan listenConnection
}

void loop() {
  listenConnection();  // Memanggil fungsi listenConnection di loop
}

void QRCodeReader(void * pvParameters) {
  Serial.print("QRCodeReader running on core ");  // Mencetak core yang menjalankan task QRCodeReader
  Serial.println(xPortGetCoreID());

  while (1) {
      q = quirc_new();  // Membuat objek quirc baru
      if (q == NULL) {  // Jika gagal membuat objek
        Serial.print("can't create quirc object\r\n");  // Mencetak pesan kesalahan
        continue;  // Melanjutkan ke iterasi berikutnya
      }
    
      fb = esp_camera_fb_get();  // Mengambil frame dari kamera
      if (!fb) {  // Jika pengambilan frame gagal
        Serial.println("Camera capture failed");  // Mencetak pesan kesalahan
        continue;  // Melanjutkan ke iterasi berikutnya
      }
      
      // Menyesuaikan ukuran quirc sesuai dengan ukuran frame
      quirc_resize(q, fb->width, fb->height);
      image = quirc_begin(q, NULL, NULL);  // Memulai proses quirc
      memcpy(image, fb->buf, fb->len);  // Menyalin data frame ke buffer quirc
      quirc_end(q);  // Mengakhiri proses quirc
      
      int count = quirc_count(q);  // Menghitung jumlah kode QR yang terdeteksi
      if (count > 0) {  // Jika ada kode QR yang terdeteksi
        Serial.println(count);  // Mencetak jumlah kode QR yang terdeteksi
        quirc_extract(q, 0, &code);  // Mengekstrak kode QR pertama
        err = quirc_decode(&code, &data);  // Mendekode kode QR
    
        if (err) {  // Jika dekode gagal
          Serial.println("Decoding FAILED");  // Mencetak pesan kesalahan
          QRCodeResult = "Decoding FAILED";  // Mengatur hasil QRCodeResult
        } else {
          Serial.printf("Decoding successful:\n");  // Mencetak pesan keberhasilan
          dumpData(&data);  // Memanggil fungsi dumpData untuk mencetak data kode QR
        } 
        Serial.println();  // Mencetak baris kosong
      }
      
      esp_camera_fb_return(fb);  // Mengembalikan frame buffer ke kamera
      fb = NULL;  // Mengatur pointer frame buffer ke NULL
      image = NULL;  // Mengatur pointer image ke NULL
      quirc_destroy(q);  // Menghancurkan objek quirc
  }
}

void dumpData(const struct quirc_data *data) {
  Serial.printf("Version: %d\n", data->version);  // Mencetak versi kode QR
  Serial.printf("ECC level: %c\n", "MLHQ"[data->ecc_level]);  // Mencetak level ECC
  Serial.printf("Mask: %d\n", data->mask);  // Mencetak mask
  Serial.printf("Length: %d\n", data->payload_len);  // Mencetak panjang payload
  Serial.printf("Payload: %s\n", data->payload);  // Mencetak payload
  
  QRCodeResult = (const char *)data->payload;  // Mengatur QRCodeResult dengan payload
}

// Definisi halaman web utama dalam bentuk literal string
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Responsive HTML</title>
    <style>
        body {
            margin: 0;
            font-family: Arial, sans-serif;
            background: #F4F4F4;
            color: black;
        }
        header {
            background: white;
            padding: 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid #ccc;
        }
        h2 {
            margin: 0;
        }
        .container {
            padding: 20px;
        }
        .card {
            background: white;
            padding: 10px 5px 20px 5px;
            box-shadow: 0 4px 8px #D1D1D1;
            margin-top: 20px;
            border-radius: 5px;
            display: flex;
            flex-direction: column;
            align-items: center;
            text-align: center;
            color: black;
        }
        .btn {
            background: black;
            color: white;
            padding: 5px 10px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }
        .switch {
            position: relative;
            display: inline-block;
            width: 60px;
            height: 34px;
        }
        .switch input {
            opacity: 0;
            width: 0;
            height: 0;
        }
        .slider {
            position: absolute;
            cursor: pointer;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: #ccc;
            -webkit-transition: .4s;
            transition: .4s;
            border-radius: 34px;
        }
        .slider:before {
            position: absolute;
            content: "";
            height: 26px;
            width: 26px;
            left: 4px;
            bottom: 4px;
            background-color: white;
            -webkit-transition: .4s;
            transition: .4s;
            border-radius: 50%;
        }
        input:checked+.slider {
            background-color: #2196F3;
        }
        input:focus+.slider {
            box-shadow: 0 0 1px #2196F3;
        }
        input:checked+.slider:before {
            -webkit-transform: translateX(26px);
            -ms-transform: translateX(26px);
            transform: translateX(26px);
        }
        .mode-dark header {
            background: black;
            color: white;
        }
        .mode-dark body {
            background: #2D2D2D;
            color: white;
        }
        .mode-dark .card {
            background: black;
            box-shadow: 0 4px 8px #565656;
            color: white;
        }
        .mode-dark .btn {
            background: white;
            color: black;
        }
        table {
            width: 100%;
            max-width: 600px;
            margin-top: 20px;
            margin-bottom: 30px;
            margin-left: auto;
            margin-right: auto;
        }
        td {
            padding: 4px;
            text-align: center;
        }
        img {
            max-width: 100%;
            height: auto;
            border-radius: 5px;
            margin-bottom: 20px;
        }
        #result {
            font-size: 20px;
            font-weight: bold;
            color: #FF4500;
            animation: blink 1s infinite;
        }
        @keyframes blink {
            0% { opacity: 1; }
            50% { opacity: 0; }
            100% { opacity: 1; }
        }
    </style>
</head>
<body class="mode-light">
    <header>
        <h2>ESP32CAM</h2>
        <label class="switch">
            <input type="checkbox" id="modeSwitch">
            <span class="slider round"></span>
        </label>
    </header>
    <div class="container">
        <div class="card">
            <canvas id="canvas" width="320" height="240"></canvas>
            <table>
                <tr>
                    <td>Flash :</td>
                    <td>
                        <input type="range" id="flash" min="0" max="255" step="1" value="0">
                    </td>
                    <td>
                        <input type="button" class="btn" value="Get Still" onclick="getStill();">
                    </td>
                </tr>
            </table>
            <br/>
            <p id="result"></div>
        </p>
    </div>
  </body>
</html> 
    <script>
        // Mengatur event listener untuk switch mode gelap
        const modeSwitch = document.getElementById('modeSwitch');
        const body = document.body;

        modeSwitch.addEventListener('change', () => {
            if (modeSwitch.checked) {
                body.classList.add('mode-dark');
                body.style.background = '#2D2D2D';
            } else {
                body.classList.remove('mode-dark');
                body.style.background = '#F4F4F4';
            }
        });
    </script>
<script>
  var canvas = document.getElementById('canvas'); 
  var context = canvas.getContext('2d');
  var flash = document.getElementById('flash');
  var result = document.getElementById('result');
  
  // Mengatur event listener untuk perubahan nilai slider flash
  flash.onchange = function() {
    var query = document.location.origin+"/?flash="+flash.value;
    fetch(query);
  }

  // Fungsi untuk mendapatkan gambar still dari kamera
  function getStill() {
    var query = document.location.origin+"/?getstill";
    fetch(query).then(function(response) {
      return response.text();
    }).then(function(text) {
      result.innerHTML = text.split(",")[1];  // Menampilkan hasil QR code
      text = text.split(",")[0];  // Mendapatkan data gambar
      context.clearRect(0, 0, canvas.width, canvas.height);
      var imgData=context.getImageData(0,0,canvas.width,canvas.height);
      var n = 0;
      for (var i=0;i<imgData.data.length;i+=4) {
        var val = parseInt(text.substr(2*n,2), 16);
        imgData.data[i]=val;
        imgData.data[i+1]=val;
        imgData.data[i+2]=val;
        imgData.data[i+3]=255;
        n++;
      }
      context.putImageData(imgData,0,0);
      setTimeout(function(){getStill();}, 100);  // Memanggil kembali fungsi getStill setelah 100ms
    })
  }
</script>   
)rawliteral";

// Menjalankan perintah yang didefinisikan
void ExecuteCommand() {
  if (cmd != "getstill") {
    Serial.println("cmd= " + cmd + " ,P1= " + P1 + " ,P2= " + P2 + " ,P3= " + P3 + " ,P4= " + P4 + " ,P5= " + P5 + " ,P6= " + P6 + " ,P7= " + P7 + " ,P8= " + P8 + " ,P9= " + P9);
    Serial.println("");
  }

  // Blok perintah yang didefinisikan http://192.168.xxx.xxx?cmd=P1;P2;P3;P4;P5;P6;P7;P8;P9
  if (cmd == "your cmd") {
    // Anda bisa melakukan apa saja
    // Feedback="<font color=\"red\">Hello World</font>";   // Bisa berupa teks biasa atau HTML
  } else if (cmd == "ip") {  // Menanyakan APIP, STAIP
    Feedback = "AP IP: " + WiFi.softAPIP().toString();    
    Feedback += "<br>";
    Feedback += "STA IP: " + WiFi.localIP().toString();
  } else if (cmd == "mac") {  // Menanyakan alamat MAC
    Feedback = "STA MAC: " + WiFi.macAddress();
  } else if (cmd == "restart") {  // Mengatur ulang koneksi WIFI
    ESP.restart();
  } else if (cmd == "digitalwrite") {  // Output digital
    ledcDetachPin(P1.toInt());
    pinMode(P1.toInt(), OUTPUT);
    digitalWrite(P1.toInt(), P2.toInt());
  } else if (cmd == "digitalread") {  // Input digital
    Feedback = String(digitalRead(P1.toInt()));
  } else if (cmd == "analogwrite") {  // Output analog
    if (P1 == "4") {
      ledcAttachPin(2, 4);  
      ledcSetup(4, 5000, 8);
      ledcWrite(4, P2.toInt());     
    } else {
      ledcAttachPin(P1.toInt(), 9);
      ledcSetup(9, 5000, 8);
      ledcWrite(9, P2.toInt());
    }
  } else if (cmd == "analogread") {  // Membaca nilai analog
    Feedback = String(analogRead(P1.toInt()));
  } else if (cmd == "touchread") {  // Membaca nilai sentuh
    Feedback = String(touchRead(P1.toInt()));
  } else if (cmd == "framesize") {  // Mengatur ukuran frame
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt();
    s->set_framesize(s, (framesize_t)val);   
  } else if (cmd == "quality") {  // Mengatur kualitas gambar
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt(); 
    s->set_quality(s, val);
  } else if (cmd == "contrast") {  // Mengatur kontras
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt(); 
    s->set_contrast(s, val);
  } else if (cmd == "brightness") {  // Mengatur kecerahan
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt();  
    s->set_brightness(s, val);  
  } else if (cmd == "hmirror") {  // Mengatur cermin horizontal
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt();  
    s->set_hmirror(s, val); 
  } else if (cmd == "vflip") {  // Mengatur flip vertikal
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt();  
    s->set_vflip(s, val);
  } else if (cmd == "serial") {  // Output serial
    Serial.print(P1);
  } else if (cmd == "restart") {  // Merestart perangkat
    ESP.restart();
  } else if (cmd == "flash") {  // Mengatur lampu kilat
    ledcAttachPin(2, 4);  
    ledcSetup(4, 5000, 8);   
    int val = P1.toInt();
    ledcWrite(4, val);  
  } else if (cmd == "servo") {  // Mengatur servo (0-180)
    ledcAttachPin(P1.toInt(), 3);
    ledcSetup(3, 50, 16);
    int val = 7864 - P2.toInt() * 34.59; 
    if (val > 7864)
       val = 7864;
    else if (val < 1638)
      val = 1638; 
    ledcWrite(3, val);
  } else if (cmd == "relay") {  // Mengatur relay
    pinMode(P1.toInt(), OUTPUT);  
    digitalWrite(13, P2.toInt());  
  } else {
    Feedback = "Command is not defined.";  // Jika perintah tidak terdefinisi
  }  
  
  if (Feedback == "") Feedback = Command;  // Jika Feedback kosong, mengatur Feedback dengan Command
}


void mainPage() {
    // Mengirim respons HTTP berisi halaman HTML utama atau Feedback
    client.println("HTTP/1.1 200 OK");  // Menyatakan bahwa permintaan berhasil
    client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");  // Mengatur header yang diizinkan
    client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");  // Mengatur metode yang diizinkan
    client.println("Content-Type: text/html; charset=utf-8");  // Mengatur tipe konten sebagai HTML
    client.println("Access-Control-Allow-Origin: *");  // Mengizinkan akses dari semua domain
    client.println("Connection: close");  // Menutup koneksi setelah pengiriman data
    client.println();  // Baris kosong untuk mengakhiri header

    String Data = "";
    if (cmd != "")
      Data = Feedback;  // Jika ada perintah, kirimkan Feedback
    else {
      Data = String((const char *)INDEX_HTML);  // Jika tidak, kirimkan halaman HTML utama
    }

    int Index;
    for (Index = 0; Index < Data.length(); Index = Index + 1024) {
      client.print(Data.substring(Index, Index + 1024));  // Mengirim data dalam potongan 1024 byte
    }
}

void listenConnection() {
  // Mengatur ulang variabel untuk perintah dan parameter
  Feedback = ""; Command = ""; cmd = ""; P1 = ""; P2 = ""; P3 = ""; P4 = ""; P5 = ""; P6 = ""; P7 = ""; P8 = ""; P9 = "";
  ReceiveState = 0, cmdState = 1, strState = 1, questionstate = 0, equalstate = 0, semicolonstate = 0;
  
  // Memeriksa apakah ada klien yang terhubung ke server
  client = server.available();
  
  if (client) { 
    String currentLine = "";

    // Selama klien terhubung
    while (client.connected()) {
      // Jika data tersedia dari klien
      if (client.available()) {
        char c = client.read(); 
        getCommand(c);  // Memanggil fungsi untuk mengurai perintah dari data yang diterima

        if (c == '\n') {  // Jika karakter adalah newline
          if (currentLine.length() == 0) {  // Jika baris saat ini kosong, artinya header HTTP telah selesai
            if (cmd == "getstill") {
              getStill();  // Memanggil fungsi untuk mengambil gambar jika perintah adalah getstill
            } else {
              mainPage();  // Memanggil fungsi untuk mengirim halaman utama jika bukan
            }         
            Feedback = "";  // Mengatur ulang Feedback
            break;
          } else {
            currentLine = "";  // Mengatur ulang baris saat ini untuk membaca baris berikutnya
          }
        } else if (c != '\r') {  // Jika karakter bukan carriage return
          currentLine += c;  // Menambahkan karakter ke baris saat ini
        }

        // Memeriksa apakah baris saat ini mengandung perintah HTTP
        if ((currentLine.indexOf("?") != -1) && (currentLine.indexOf(" HTTP") != -1)) {
          // Jika perintah mengandung kata kunci "stop", segera hentikan koneksi
          if (Command.indexOf("stop") != -1) {
            client.println();
            client.println();
            client.stop();
          }
          currentLine = "";  // Mengatur ulang baris saat ini
          Feedback = "";  // Mengatur ulang Feedback
          ExecuteCommand();  // Memanggil fungsi untuk mengeksekusi perintah yang diterima
        }
      }
    }
    delay(1);  // Menambahkan sedikit penundaan
    client.stop();  // Menghentikan koneksi klien
  }
}

void getStill() {
  // Mengambil gambar dari kamera dan mengirimkannya dalam format teks
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();  
  if (!fb) {
    Serial.println("Camera capture failed");  // Mencetak pesan jika pengambilan gambar gagal
    return;
  }

  client.println("HTTP/1.1 200 OK");  // Menyatakan bahwa permintaan berhasil
  client.println("Access-Control-Allow-Origin: *");  // Mengizinkan akses dari semua domain
  client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");  // Mengatur header yang diizinkan
  client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");  // Mengatur metode yang diizinkan
  client.println("Content-Type: text/plain");  // Mengatur tipe konten sebagai teks
  client.println("Connection: close");  // Menutup koneksi setelah pengiriman data
  client.println();  // Baris kosong untuk mengakhiri header

  uint8_t *fbBuf = fb->buf;
  size_t fbLen = fb->len;
  String val = "";
  String v = "";

  for (int n = 0; n < fbLen; n++) {
    v = String(fbBuf[n], HEX);  // Mengonversi byte menjadi string heksadesimal
    if (v.length() == 1)
      val += "0" + v;  // Menambahkan '0' jika panjang string heksadesimal 1
    else
      val += v;
    if ((n + 1) % 1024 == 0) {  // Mengirim data dalam potongan 1024 byte
      client.print(val);
      val = "";
    }
  }
  if (val != "")
    client.print(val);  // Mengirim sisa data

  val = "," + QRCodeResult;
  client.print(val);  // Mengirim hasil dekode QRCode
  QRCodeResult = "";  // Mengatur ulang QRCodeResult

  esp_camera_fb_return(fb);  // Mengembalikan frame buffer ke kamera

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);  // Mengatur pin 2 ke LOW
}

void getCommand(char c) {
  // Menguraikan string perintah dan menempatkannya ke dalam variabel
  if (c == '?') ReceiveState = 1;  // Mulai menerima perintah saat menemukan '?'
  if ((c == ' ') || (c == '\r') || (c == '\n')) ReceiveState = 0;  // Berhenti menerima perintah pada karakter pemisah

  if (ReceiveState == 1) {
    Command += String(c);  // Menambahkan karakter ke string Command
    
    if (c == '=') cmdState = 0;
    if (c == ';') strState++;

    if ((cmdState == 1) && ((c != '?') || (questionstate == 1))) cmd += String(c);  // Menguraikan perintah
    if ((cmdState == 0) && (strState == 1) && ((c != '=') || (equalstate == 1))) P1 += String(c);  // Menguraikan parameter 1
    if ((cmdState == 0) && (strState == 2) && (c != ';')) P2 += String(c);  // Menguraikan parameter 2
    if ((cmdState == 0) && (strState == 3) && (c != ';')) P3 += String(c);  // Menguraikan parameter 3
    if ((cmdState == 0) && (strState == 4) && (c != ';')) P4 += String(c);  // Menguraikan parameter 4
    if ((cmdState == 0) && (strState == 5) && (c != ';')) P5 += String(c);  // Menguraikan parameter 5
    if ((cmdState == 0) && (strState == 6) && (c != ';')) P6 += String(c);  // Menguraikan parameter 6
    if ((cmdState == 0) && (strState == 7) && (c != ';')) P7 += String(c);  // Menguraikan parameter 7
    if ((cmdState == 0) && (strState == 8) && (c != ';')) P8 += String(c);  // Menguraikan parameter 8
    if ((cmdState == 0) && (strState >= 9) && ((c != ';') || (semicolonstate == 1))) P9 += String(c);  // Menguraikan parameter 9

    if (c == '?') questionstate = 1;  // Menandai bahwa '?' telah ditemukan
    if (c == '=') equalstate = 1; 
    if ((strState >= 9) && (c == ';')) semicolonstate = 1; 
  }
}
