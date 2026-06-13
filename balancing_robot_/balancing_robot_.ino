// Update: Bản sạch (Clean Version) - Cascaded PID + Sửa lỗi Encoder 
// Đã gỡ bỏ module ghi CSV để tối ưu tài nguyên hệ thống
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>

// ==========================================
// 1. CẤU HÌNH WEB & MẠNG
// ==========================================
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
unsigned long lastTelemetryTime = 0;

const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>Trạm kiểm soát PID Pro</title>
  <style>
    body { font-family: Arial; text-align: center; background: #1e1e1e; color: #fff; margin: 0; padding: 10px; }
    .card { background: #2a2a2a; padding: 15px; border-radius: 10px; max-width: 600px; margin: 0 auto 15px auto; box-shadow: 0 4px 8px rgba(0,0,0,0.5); }
    .zn-card { background: #1a3c5a; border: 1px solid #00ffcc; }
    .spd-card { background: #3a2a4a; border: 1px solid #cc00ff; }
    input[type=range] { width: 100%; height: 25px; }
    input[type=number] { width: 60px; padding: 8px; border-radius: 4px; border: 1px solid #ccc; text-align: center; font-size: 16px; }
    select, button { padding: 10px; border-radius: 5px; font-weight: bold; cursor: pointer; font-size: 14px; }
    button { background: #00ffcc; color: #000; border: none; transition: 0.2s; box-shadow: 0 2px 5px rgba(0,255,204,0.3); }
    button:active { transform: scale(0.95); }
    canvas { background: #111; width: 100%; max-width: 600px; height: 250px; border: 1px solid #555; border-radius: 5px; margin-top: 5px; touch-action: none; cursor: grab; }
    .val { font-weight: bold; color: #00ffcc; font-size: 1.2em; }
    .val-spd { font-weight: bold; color: #cc00ff; font-size: 1.2em; }
    .btn-reset { display: block; width: 100%; margin-top: 10px; background: #ff4444; color: white; }
  </style>
</head>
<body>
  <h2 style="margin-top: 5px;">DASHBOARD ESP32</h2>

  <div class="card zn-card">
    <h3 style="margin-top:0; color:#00ffcc;">Ziegler-Nichols Tuning</h3>
    <div style="display: flex; justify-content: space-around; margin-bottom: 10px; align-items: center;">
      <div><label>Ku:</label><br><input type="number" id="ku_input" step="0.1" value="25.0"></div>
      <div><label>Tu (s):</label><br><input type="number" id="tu_input" step="0.01" value="0.17"></div>
    </div>
    <div style="display: flex; gap: 10px; justify-content: center;">
      <select id="zn_rule" style="flex:1;">
        <option value="classic" selected>Classic PID</option>
        <option value="pd">PD Control</option>
        <option value="no_over">No Overshoot</option>
      </select>
      <button type="button" onclick="applyZN()">ÁP DỤNG</button>
    </div>
  </div>

  <div class="card" style="position: relative;">
    <h3 style="margin:0 0 5px 0; font-size: 14px; color:#aaa;">ĐỒ THỊ SAI SỐ GÓC (ERROR)</h3>
    <p style="margin:0 0 5px 0; font-size: 12px; color:#888;">(PC: Cuộn=Zoom Time | Shift+Cuộn=Zoom Biên độ)</p>
    <canvas id="chart"></canvas>
    <button type="button" class="btn-reset" onclick="resetScale()">VỀ MẶC ĐỊNH & TRACKING</button>
  </div>

  <div class="card">
    <h3 style="margin:0; color:#00ffcc;">PID GÓC (Vòng Trong)</h3>
    <p style="margin:5px 0;">Kp: <span id="kp_val" class="val">15.00</span></p>
    <input type="range" id="kp" min="0" max="100" step="0.1" value="15.00" oninput="updatePID()">
    <p style="margin:5px 0;">Ki: <span id="ki_val" class="val">176.47</span></p>
    <input type="range" id="ki" min="0" max="300" step="0.05" value="176.47" oninput="updatePID()">
    <p style="margin:5px 0;">Kd: <span id="kd_val" class="val">0.32</span></p>
    <input type="range" id="kd" min="0" max="20" step="0.05" value="0.32" oninput="updatePID()">
  </div>

  <div class="card spd-card">
    <h3 style="margin:0; color:#cc00ff;">PID VỊ TRÍ (Vòng Ngoài)</h3>
    <p style="margin:5px 0; font-size:12px; color:#aaa;">(Tăng Kp_spd để chống trôi. Ki_spd giữ vị trí cố định)</p>
    <p style="margin:5px 0;">Kp_spd: <span id="kp_spd_val" class="val-spd">0.00</span></p>
    <input type="range" id="kp_spd" min="0" max="1.0" step="0.005" value="0.0" oninput="updateSpeedPID()">
    <p style="margin:5px 0;">Ki_spd: <span id="ki_spd_val" class="val-spd">0.000</span></p>
    <input type="range" id="ki_spd" min="0" max="0.1" step="0.001" value="0.0" oninput="updateSpeedPID()">
  </div>

  <script>
    var ws = new WebSocket('ws://' + window.location.hostname + ':81/');
    var canvas = document.getElementById('chart');
    var ctx = canvas.getContext('2d');
    
    var historyData = [];
    var autoScroll = true;
    var timeWindow = 5000;    
    var viewEndTime = Date.now(); 
    var ySpan = 180;          
    var yCenter = 0;          

    var isDragging = false;
    var lastMouseX = 0, lastMouseY = 0;
    var lastTouchCenter = null;
    var lastDistX = 0, lastDistY = 0;

    function screenToTime(cssX) { return (viewEndTime - timeWindow) + (cssX / canvas.clientWidth) * timeWindow; }
    function screenToAngle(cssY) { let topAngle = yCenter + ySpan / 2; return topAngle - (cssY / canvas.clientHeight) * ySpan; }

    function applyZoomX(cssX, zoomFactor) {
      autoScroll = false; let ratioX = cssX / canvas.clientWidth;
      let pointerTime = screenToTime(cssX); timeWindow *= zoomFactor;
      if(timeWindow < 200) timeWindow = 200; if(timeWindow > 60000) timeWindow = 60000;
      viewEndTime = pointerTime + (1 - ratioX) * timeWindow;
    }

    function applyZoomY(cssY, zoomFactor) {
      let ratioY = cssY / canvas.clientHeight; let pointerAngle = screenToAngle(cssY);
      ySpan *= zoomFactor; if(ySpan < 2) ySpan = 2; if(ySpan > 1000) ySpan = 1000;
      yCenter = pointerAngle + (ratioY - 0.5) * ySpan;
    }

    canvas.addEventListener('mousedown', e => { isDragging = true; autoScroll = false; lastMouseX = e.clientX; lastMouseY = e.clientY; });
    window.addEventListener('mouseup', () => isDragging = false);
    window.addEventListener('mousemove', e => {
      if(!isDragging || e.buttons !== 1) return;
      let dx = e.clientX - lastMouseX; let dy = e.clientY - lastMouseY;
      viewEndTime -= (dx / canvas.clientWidth) * timeWindow; yCenter += (dy / canvas.clientHeight) * ySpan;
      lastMouseX = e.clientX; lastMouseY = e.clientY;
    });

    canvas.addEventListener('wheel', e => {
      e.preventDefault(); autoScroll = false;
      let zoomFactor = e.deltaY > 0 ? 1.15 : 0.85;
      let rect = canvas.getBoundingClientRect(); let cssX = e.clientX - rect.left; let cssY = e.clientY - rect.top;
      if (e.shiftKey) applyZoomY(cssY, zoomFactor); else applyZoomX(cssX, zoomFactor);            
    }, {passive: false});

    canvas.addEventListener('touchstart', e => {
      e.preventDefault(); isDragging = true; autoScroll = false;
      let rect = canvas.getBoundingClientRect();
      if (e.touches.length === 1) { lastMouseX = e.touches[0].clientX; lastMouseY = e.touches[0].clientY; } 
      else if (e.touches.length === 2) {
        let x1 = e.touches[0].clientX, y1 = e.touches[0].clientY; let x2 = e.touches[1].clientX, y2 = e.touches[1].clientY;
        lastDistX = Math.abs(x1 - x2); lastDistY = Math.abs(y1 - y2); lastTouchCenter = { x: (x1+x2)/2, y: (y1+y2)/2 };
      }
    }, {passive: false});

    canvas.addEventListener('touchmove', e => {
      e.preventDefault(); if (!isDragging) return;
      let rect = canvas.getBoundingClientRect();
      if (e.touches.length === 1) { 
        let dx = e.touches[0].clientX - lastMouseX; let dy = e.touches[0].clientY - lastMouseY;
        viewEndTime -= (dx / rect.width) * timeWindow; yCenter += (dy / rect.height) * ySpan;
        lastMouseX = e.touches[0].clientX; lastMouseY = e.touches[0].clientY;
      } 
      else if (e.touches.length === 2) { 
        let x1 = e.touches[0].clientX, y1 = e.touches[0].clientY; let x2 = e.touches[1].clientX, y2 = e.touches[1].clientY;
        let newDistX = Math.abs(x1 - x2); let newDistY = Math.abs(y1 - y2); let newCenter = { x: (x1+x2)/2, y: (y1+y2)/2 };
        if (lastTouchCenter) {
           let dx = newCenter.x - lastTouchCenter.x; let dy = newCenter.y - lastTouchCenter.y;
           viewEndTime -= (dx / rect.width) * timeWindow; yCenter += (dy / rect.height) * ySpan;
        }
        if (lastDistX > 0 && newDistX > 0) applyZoomX(newCenter.x - rect.left, lastDistX / newDistX);
        if (lastDistY > 0 && newDistY > 0) applyZoomY(newCenter.y - rect.top, lastDistY / newDistY);
        lastDistX = newDistX; lastDistY = newDistY; lastTouchCenter = newCenter;
      }
    }, {passive: false});

    window.addEventListener('touchend', e => {
      if (e.touches.length === 0) { isDragging = false; lastTouchCenter = null; }
      else if (e.touches.length === 1) { lastMouseX = e.touches[0].clientX; lastMouseY = e.touches[0].clientY; lastTouchCenter = null; }
    });

    function resetScale() { autoScroll = true; timeWindow = 5000; ySpan = 180; yCenter = 0; }
    canvas.ondblclick = resetScale;

    function render() {
      requestAnimationFrame(render);
      let pr = window.devicePixelRatio || 1; canvas.width = canvas.clientWidth * pr; canvas.height = canvas.clientHeight * pr;
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      let now = Date.now(); if (autoScroll) viewEndTime = now; let viewStartTime = viewEndTime - timeWindow;
      while(historyData.length > 0 && historyData[0].t < now - 60000) historyData.shift();

      let stepMs = timeWindow <= 1000 ? 100 : (timeWindow <= 2500 ? 250 : 500);
      let firstLineT = Math.ceil(viewStartTime / stepMs) * stepMs;
      
      ctx.lineWidth = 1; ctx.font = (10 * pr) + 'px Arial';
      for (let t = firstLineT; t <= viewEndTime; t += stepMs) {
        let x = ((t - viewStartTime) / timeWindow) * canvas.width;
        ctx.strokeStyle = '#333'; ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
        ctx.fillStyle = '#888'; ctx.fillText(((t - now) / 1000).toFixed(2) + "s", x + 5, canvas.height - 5);
      }

      let stepAngle = ySpan > 500 ? 90 : ySpan > 200 ? 45 : ySpan > 100 ? 30 : ySpan > 50 ? 10 : ySpan > 20 ? 5 : ySpan > 10 ? 2 : 1;
      let topAngle = yCenter + ySpan / 2; let botAngle = yCenter - ySpan / 2;
      let firstLineA = Math.ceil(botAngle / stepAngle) * stepAngle;

      for (let a = firstLineA; a <= topAngle; a += stepAngle) {
        let y = ((topAngle - a) / ySpan) * canvas.height;
        ctx.strokeStyle = (a === 0) ? 'rgba(255,0,0,0.5)' : '#333'; ctx.lineWidth = (a === 0) ? 2 : 1;
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
        ctx.fillStyle = (a === 0) ? '#ff4444' : '#00ffcc'; ctx.fillText(a + "°", 5, y - 5); 
      }

      if (historyData.length > 0) {
        ctx.beginPath(); ctx.strokeStyle = '#00ffcc'; ctx.lineWidth = 2 * pr; let started = false;
        for (let i = 0; i < historyData.length; i++) {
          let pt = historyData[i]; if (pt.t < viewStartTime) continue; if (pt.t > viewEndTime) break;      
          let x = ((pt.t - viewStartTime) / timeWindow) * canvas.width; let y = ((topAngle - pt.y) / ySpan) * canvas.height;
          if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
        }
        ctx.stroke();
      }

      if(!autoScroll) {
        ctx.fillStyle = 'rgba(255, 0, 0, 0.7)'; ctx.fillRect(10, 10, 70 * pr, 20 * pr);
        ctx.fillStyle = '#fff'; ctx.font = 'bold ' + (12 * pr) + 'px Arial'; ctx.fillText("PAUSED", 15 * pr, 25 * pr);
      }
    }
    requestAnimationFrame(render);

    ws.onmessage = function(event) { historyData.push({ t: Date.now(), y: parseFloat(event.data) }); };

    function updatePID() {
      var kp = document.getElementById('kp').value; var ki = document.getElementById('ki').value; var kd = document.getElementById('kd').value;
      document.getElementById('kp_val').innerText = kp; document.getElementById('ki_val').innerText = ki; document.getElementById('kd_val').innerText = kd;
      ws.send("PID," + kp + "," + ki + "," + kd);
    }
    
    function updateSpeedPID() {
      var kps = document.getElementById('kp_spd').value; var kis = document.getElementById('ki_spd').value;
      document.getElementById('kp_spd_val').innerText = kps; document.getElementById('ki_spd_val').innerText = kis;
      ws.send("SPID," + kps + "," + kis);
    }

    function applyZN() {
      var Ku = parseFloat(document.getElementById('ku_input').value); var Tu = parseFloat(document.getElementById('tu_input').value);
      var rule = document.getElementById('zn_rule').value; var p = 0, i = 0, d = 0;

      if(rule === 'classic') { p = 0.6 * Ku; i = (1.2 * Ku) / Tu; d = 0.075 * Ku * Tu; }
      else if(rule === 'no_over') { p = 0.2 * Ku; i = (0.4 * Ku) / Tu; d = 0.066 * Ku * Tu; }
      else if(rule === 'pd') { p = 0.8 * Ku; i = 0; d = 0.1 * Ku * Tu; }

      document.getElementById('kp').max = Math.max(100, p * 1.5).toFixed(0);
      document.getElementById('ki').max = Math.max(300, i * 1.5).toFixed(0); 
      document.getElementById('kd').max = Math.max(20, d * 1.5).toFixed(0);

      document.getElementById('kp').value = p.toFixed(2); document.getElementById('ki').value = i.toFixed(2); document.getElementById('kd').value = d.toFixed(2);
      updatePID();
    }
  </script>
</body>
</html>
)=====";

// ==========================================
// 2. KHAI BÁO PHẦN CỨNG & BIẾN
// ==========================================
const int MPU_ADDR = 0x68;
const int PWMA = 18; const int AIN2 = 4; const int AIN1 = 16; 
const int PWMB = 19; const int BIN2 = 27; const int BIN1 = 17; 
const int freq = 5000; const int resolution = 8; 

// --- KHAI BÁO ENCODER CHUẨN SƠ ĐỒ ---
const int ENCA_A = 32; const int ENCA_B = 33; // Motor Trái
const int ENCB_A = 25; const int ENCB_B = 26; // Motor Phải
volatile long pulseA = 0;
volatile long pulseB = 0;

// Các biến thời gian
unsigned long prevTime = 0;      // Dành cho vòng Góc (100Hz)
unsigned long prevSpeedTime = 0; // Dành cho vòng Vận tốc (50Hz)

// Các biến MPU
float dt = 0;
float accelY, accelZ, gyroX;
float angleAccel = 0, currentAngle = 0; 

// --- CÁC BIẾN CỦA VÒNG TRONG (GÓC) ---
float Kp = 15.0, Ki = 176.47, Kd = 0.32;   
float baseTargetAngle = -2.47; // Trọng tâm vật lý tĩnh
float dynamicTargetAngle = -2.47; // Trọng tâm có bù trừ của Vận Tốc
float errorAngle = 0, integralAngle = 0, derivativeAngle = 0, pidOutput = 0;

// --- CÁC BIẾN CỦA VÒNG NGOÀI (VỊ TRÍ / VẬN TỐC) ---
float Kp_spd = 0.0, Ki_spd = 0.0;
float targetSpeed = 0.0; // 0 = Đứng im giữ vị trí
float currentSpeed = 0.0;
float filteredSpeed = 0.0;
long lastEncoderSum = 0;
float speedIntegral = 0;
float speedOutput = 0; // Đây chính là góc nghiêng cần bù (angle_offset)

// ==========================================
// HÀM NGẮT ENCODER (CHẠY NGẦM SIÊU TỐC)
// ==========================================
void IRAM_ATTR isrA() {
  // ĐÃ SỬA LỖI NGƯỢC CHIỀU: Đổi ++ thành -- và ngược lại
  if (digitalRead(ENCA_A) == digitalRead(ENCA_B)) pulseA--;
  else pulseA++;
}
void IRAM_ATTR isrB() {
  if (digitalRead(ENCB_A) == digitalRead(ENCB_B)) pulseB++;
  else pulseB--;
}

// ==========================================
// CÁC HÀM XỬ LÝ CHÍNH
// ==========================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if(type == WStype_TEXT) {
    String msg = String((char*)payload);
    if(msg.startsWith("PID,")) {
      int c1 = msg.indexOf(','); int c2 = msg.indexOf(',', c1 + 1); int c3 = msg.indexOf(',', c2 + 1);
      Kp = msg.substring(c1 + 1, c2).toFloat();
      Ki = msg.substring(c2 + 1, c3).toFloat();
      Kd = msg.substring(c3 + 1).toFloat();
    }
    else if(msg.startsWith("SPID,")) {
      int c1 = msg.indexOf(','); int c2 = msg.indexOf(',', c1 + 1);
      Kp_spd = msg.substring(c1 + 1, c2).toFloat();
      Ki_spd = msg.substring(c2 + 1).toFloat();
      Serial.printf("UPDATE SPEED PID: Kp_spd=%.3f, Ki_spd=%.4f\n", Kp_spd, Ki_spd);
    }
  }
}

void setupMPU() {
  Wire.begin(); Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) { Serial.println("MPU Error!"); while(1); }
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
}

void calculateAngle() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); 
  Wire.read(); Wire.read(); 
  int16_t rawAccelY = (Wire.read() << 8 | Wire.read());
  int16_t rawAccelZ = (Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); 
  int16_t rawGyroX = (Wire.read() << 8 | Wire.read());
  accelY = (float)rawAccelY / 16384.0; accelZ = (float)rawAccelZ / 16384.0;
  gyroX  = (float)rawGyroX / 131.0; 
  angleAccel = atan2(accelY, accelZ) * 180 / PI;
  currentAngle = 0.98 * (currentAngle + gyroX * dt) + 0.02 * angleAccel;
}

void setMotors(float output) {
  int speed = (int)constrain(output, -255, 255);
  int pwmValue = abs(speed);
  if (pwmValue < 5) pwmValue = 0;
  if (speed > 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  }
  ledcWrite(PWMA, pwmValue); ledcWrite(PWMB, pwmValue);
}

void setup() {
  Serial.begin(115200); delay(1000); 

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  ledcAttach(PWMA, freq, resolution); ledcAttach(PWMB, freq, resolution);
  
  // Cài đặt chân Encoder
  pinMode(ENCA_A, INPUT_PULLUP); pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCB_A, INPUT_PULLUP); pinMode(ENCB_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), isrB, CHANGE);

  setupMPU();
  WiFi.mode(WIFI_AP); WiFi.softAP("Xe_Can_Bang_ESP32", "12345678");
  ArduinoOTA.begin();
  server.on("/", []() { server.send(200, "text/html", webpage); });
  server.begin(); webSocket.begin(); webSocket.onEvent(webSocketEvent);
  
  prevTime = micros();
  prevSpeedTime = micros();
}

void loop() {
  ArduinoOTA.handle(); server.handleClient(); webSocket.loop();
  unsigned long currentMicros = micros(); unsigned long currentMillis = millis();

  // -------------------------------------------------------------
  // VÒNG NGOÀI: VẬN TỐC / VỊ TRÍ (Chạy tần số 50Hz - mỗi 20ms)
  // -------------------------------------------------------------
  if (currentMicros - prevSpeedTime >= 20000) {
    float dt_spd = (currentMicros - prevSpeedTime) / 1000000.0;
    prevSpeedTime = currentMicros;

    // Lấy tổng xung của 2 bánh (Chia trung bình) - Bây giờ cả 2 bánh lăn tới đều Dương
    long currentEncoderSum = (pulseA + pulseB) / 2; 
    
    currentSpeed = (currentEncoderSum - lastEncoderSum) / dt_spd;
    lastEncoderSum = currentEncoderSum;

    // Lọc nhiễu vận tốc (Low-pass filter) để tránh giật xe
    filteredSpeed = 0.7 * filteredSpeed + 0.3 * currentSpeed;

    // Tính toán PI cho Vận Tốc
    float speedError = targetSpeed - filteredSpeed;
    speedIntegral += speedError * dt_spd;
    speedIntegral = constrain(speedIntegral, -3000, 3000); // Chống Wind-up

    // Đầu ra của Vận Tốc chính là GÓC NGHIÊNG CẦN BÙ (Tối đa ép nghiêng +-10 độ)
    speedOutput = (Kp_spd * speedError) + (Ki_spd * speedIntegral);
    speedOutput = constrain(speedOutput, -10.0, 10.0);

    // Cập nhật góc mục tiêu mới cho vòng trong
    dynamicTargetAngle = baseTargetAngle - speedOutput; 
  }

  // -------------------------------------------------------------
  // VÒNG TRONG: PID GÓC (Chạy tần số 100Hz - mỗi 10ms)
  // -------------------------------------------------------------
  if (currentMicros - prevTime >= 10000) {
    dt = (currentMicros - prevTime) / 1000000.0;
    prevTime = currentMicros;
    calculateAngle();
    
    // Ngắt an toàn khi góc vượt quá 20 độ
    if (abs(currentAngle) > 20.0) {
      setMotors(0); integralAngle = 0; speedIntegral = 0; // Ngã thì reset sạch
    } else {
      errorAngle = dynamicTargetAngle - currentAngle; 
      integralAngle += errorAngle * dt;
      integralAngle = constrain(integralAngle, -400, 400); 
      derivativeAngle = -gyroX; 
      pidOutput = (Kp * errorAngle) + (Ki * integralAngle) + (Kd * derivativeAngle);
      setMotors(pidOutput);
    }
  }

  // Bắn dữ liệu (20Hz) - Hiển thị sai số so với góc nền tảng
  if (currentMillis - lastTelemetryTime > 50) {
    float currentError = baseTargetAngle - currentAngle;
    String errorStr = String(currentError); 
    webSocket.broadcastTXT(errorStr);       
    lastTelemetryTime = currentMillis;
  }
}