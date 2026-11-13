/*
  ESP32-CAM + ULN2003 + 28BYJ-48 (5 V) Web control

  ULN2003 -> ESP32-CAM
    IN1 -> GPIO4
    IN2 -> GPIO2
    IN3 -> GPIO14
    IN4 -> GPIO15

  Upload first. Then connect the computer to the Wi-Fi hotspot and open:
    http://192.168.4.1

  Power requirements
    - Use a separate, stable 5 V supply for ULN2003 + 28BYJ-48.
    - Connect external 5 V GND and ESP32-CAM GND together.
    - Do not power the motor from the USB-to-serial programmer.
*/

#include <WiFi.h>
#include <WebServer.h>

constexpr uint8_t IN1_PIN = 4;
constexpr uint8_t IN2_PIN = 2;
constexpr uint8_t IN3_PIN = 14;
constexpr uint8_t IN4_PIN = 15;

// Change these two values before uploading if desired.
constexpr char AP_SSID[] = "ESP32-CAM-Motor";
constexpr char AP_PASSWORD[] = "opto2026";  // Wi-Fi passwords require 8+ characters.

constexpr uint8_t HALF_STEP[8][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1},
};

WebServer server(80);

int8_t phase = 0;
int8_t direction = 1;
uint32_t stepsRemaining = 0;
uint32_t stepIntervalUs = 2000;  // Initial value: 2 ms per half-step.
uint32_t nextStepDueUs = 0;
bool motorRunning = false;
bool continuousMode = false;
bool whiteLedOn = false;

const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32-CAM 步进电机控制</title>
  <style>
    :root{color-scheme:dark;--bg:#101827;--card:#182235;--line:#31415c;--accent:#4cc9f0;--stop:#ef476f}
    *{box-sizing:border-box}body{margin:0;font:16px system-ui,"Microsoft YaHei",sans-serif;background:var(--bg);color:#eef5ff}
    main{max-width:760px;margin:auto;padding:22px 16px 38px}h1{font-size:1.5rem;margin:0 0 8px}h2{font-size:1.05rem;margin:0 0 12px}
    .hint{color:#b8c7de;line-height:1.55}.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-top:14px}
    .grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}@media(max-width:480px){.grid{grid-template-columns:1fr}}
    label{display:block;color:#b8c7de;font-size:.9rem}input{width:100%;margin-top:6px;padding:10px;border-radius:8px;border:1px solid var(--line);background:#0d1420;color:white;font-size:1rem}
    .buttons{display:flex;flex-wrap:wrap;gap:10px;margin-top:14px}button{border:0;border-radius:8px;padding:10px 14px;background:var(--accent);color:#06202a;font-weight:700;cursor:pointer}
    button.secondary{background:#90a4c4}button.stop{background:var(--stop);color:white}code,pre{background:#0d1420;border-radius:8px}pre{margin:0;padding:12px;overflow:auto;white-space:pre-wrap;line-height:1.5}
    #state{font-variant-numeric:tabular-nums;color:#cfefff}
  </style>
</head>
<body>
<main>
  <h1>ESP32-CAM · 28BYJ-48 控制台</h1>
  <p class="hint">单次转动完成后会自动释放线圈；“连续运行”会一直保持通电，需点击停止或释放。白灯和 IN1 共用 GPIO4：电机转动时白灯会随步进闪烁。</p>

  <section class="card">
    <h2>转动参数</h2>
    <div class="grid">
      <label>单次步数（半步，1–500000）<input id="steps" type="number" min="1" max="500000" value="512"></label>
      <label>步进间隔（毫秒，1–50）<input id="interval" type="number" min="1" max="50" value="2" step="1"></label>
    </div>
    <div class="buttons">
      <button onclick="move(1)">正转单次</button>
      <button onclick="move(-1)">反转单次</button>
      <button class="secondary" onclick="run(1)">持续正转</button>
      <button class="secondary" onclick="run(-1)">持续反转</button>
      <button class="stop" onclick="post('/api/stop')">停止</button>
      <button class="stop" onclick="post('/api/release')">停止并释放</button>
      <button class="secondary" onclick="post('/api/led?on=1')">白灯开（空闲时）</button>
      <button class="secondary" onclick="post('/api/led?on=0')">白灯关</button>
    </div>
  </section>

  <section class="card">
    <h2>实时状态</h2>
    <div id="state">正在读取…</div>
  </section>

  <section class="card">
    <h2>网页/程序接口</h2>
    <p class="hint">可直接从电脑上的脚本、LabVIEW 或其他软件发送 HTTP 请求；参数修改会立即应用。</p>
    <pre><code>POST /api/move?steps=512&interval=2
POST /api/run?dir=1&interval=2     # dir=1 正转，dir=-1 反转
POST /api/stop
POST /api/release
POST /api/led?on=1               # 仅电机空闲时设置
GET  /api/status
GET  /api/help</code></pre>
    <div class="buttons"><button class="secondary" onclick="copyExample()">复制 JavaScript 示例</button></div>
  </section>
</main>
<script>
const $ = id => document.getElementById(id);
function value(id, low, high) {
  const v = Number($(id).value);
  return Number.isFinite(v) ? Math.max(low, Math.min(high, Math.round(v))) : low;
}
async function post(path) {
  try {
    const response = await fetch(path, {method:'POST', cache:'no-store'});
    if (!response.ok) throw new Error(await response.text());
    await refresh();
  } catch (error) { $('state').textContent = '请求失败：' + error; }
}
function move(dir) { post('/api/move?steps=' + (dir * value('steps',1,500000)) + '&interval=' + value('interval',1,50)); }
function run(dir)  { post('/api/run?dir=' + dir + '&interval=' + value('interval',1,50)); }
async function refresh() {
  try {
    const s = await (await fetch('/api/status', {cache:'no-store'})).json();
    const mode = s.running ? (s.continuous ? '连续运行' : '单次运行') : '空闲';
    const d = s.direction > 0 ? '正转' : '反转';
    const led = s.ledOn ? '开' : '关';
    const ledNote = s.ledContended ? '（电机正在转动，白灯会闪烁）' : '';
    $('state').textContent = '状态：' + mode + '；方向：' + d + '；步进间隔：' + s.intervalMs + ' ms；剩余步数：' + s.remaining + '；白灯：' + led + ledNote;
  } catch (_) { $('state').textContent = '控制器未响应，请检查 Wi-Fi 连接。'; }
}
async function copyExample() {
  const example = "fetch('http://192.168.4.1/api/move?steps=1024&interval=2', {method: 'POST'});";
  await navigator.clipboard.writeText(example);
}
refresh(); setInterval(refresh, 500);
</script>
</body></html>
)HTML";

void writePhase(uint8_t index) {
  digitalWrite(IN1_PIN, HALF_STEP[index][0]);
  digitalWrite(IN2_PIN, HALF_STEP[index][1]);
  digitalWrite(IN3_PIN, HALF_STEP[index][2]);
  digitalWrite(IN4_PIN, HALF_STEP[index][3]);
}

void releaseMotor() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

// GPIO4 is both the ESP32-CAM flash LED pin and ULN2003 IN1. Therefore this
// function can make the white LED steady only while the motor is idle; when
// whiteLedOn is true it also energizes the motor's IN1 coil.
void applyIdleOutputs() {
  digitalWrite(IN1_PIN, whiteLedOn ? HIGH : LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

uint32_t readIntervalUs() {
  long milliseconds = server.hasArg("interval") ? server.arg("interval").toInt() : 2;
  milliseconds = constrain(milliseconds, 1, 50);
  return static_cast<uint32_t>(milliseconds) * 1000UL;
}

void startMove(int32_t signedSteps, uint32_t intervalUs) {
  if (signedSteps == 0) return;
  direction = signedSteps > 0 ? 1 : -1;
  stepsRemaining = static_cast<uint32_t>(abs(signedSteps));
  stepIntervalUs = intervalUs;
  continuousMode = false;
  motorRunning = true;
  nextStepDueUs = micros();
}

void startContinuous(int8_t newDirection, uint32_t intervalUs) {
  direction = newDirection >= 0 ? 1 : -1;
  stepIntervalUs = intervalUs;
  stepsRemaining = 0;
  continuousMode = true;
  motorRunning = true;
  nextStepDueUs = micros();
}

// This is deliberately non-blocking: WebServer can answer Stop/parameter
// requests even while the motor is turning continuously.
void updateMotor() {
  if (!motorRunning) return;

  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextStepDueUs) < 0) return;

  phase = (phase + direction + 8) % 8;
  writePhase(phase);
  nextStepDueUs = nowUs + stepIntervalUs;

  if (!continuousMode && --stepsRemaining == 0) {
    motorRunning = false;
    applyIdleOutputs();
  }
}

void sendStatus() {
  const char* mode = motorRunning ? (continuousMode ? "continuous" : "move") : "idle";
  const String json = String("{\"mode\":\"") + mode +
                      "\",\"running\":" + (motorRunning ? "true" : "false") +
                      ",\"continuous\":" + (continuousMode ? "true" : "false") +
                      ",\"direction\":" + String(direction) +
                      ",\"remaining\":" + String(stepsRemaining) +
                      ",\"intervalMs\":" + String(stepIntervalUs / 1000UL) +
                      ",\"ledOn\":" + (whiteLedOn ? "true" : "false") +
                      ",\"ledContended\":" + (motorRunning ? "true" : "false") + "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void sendNoCache(int status, const char* type, const String& body) {
  server.sendHeader("Cache-Control", "no-store, max-age=0");
  server.send(status, type, body);
}

void setupRoutes() {
  server.on("/", HTTP_GET, []() { sendNoCache(200, "text/html; charset=utf-8", PAGE); });
  server.on("/api/status", HTTP_GET, sendStatus);

  server.on("/api/move", HTTP_POST, []() {
    const long requestedSteps = server.hasArg("steps") ? server.arg("steps").toInt() : 0;
    if (requestedSteps == 0 || requestedSteps < -500000L || requestedSteps > 500000L) {
      sendNoCache(400, "application/json", "{\"error\":\"steps must be -500000..-1 or 1..500000\"}");
      return;
    }
    startMove(static_cast<int32_t>(requestedSteps), readIntervalUs());
    sendStatus();
  });

  server.on("/api/run", HTTP_POST, []() {
    const long requestedDirection = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
    startContinuous(requestedDirection >= 0 ? 1 : -1, readIntervalUs());
    sendStatus();
  });

  server.on("/api/stop", HTTP_POST, []() {
    motorRunning = false;
    continuousMode = false;
    stepsRemaining = 0;
    applyIdleOutputs();
    sendStatus();
  });

  server.on("/api/release", HTTP_POST, []() {
    motorRunning = false;
    continuousMode = false;
    stepsRemaining = 0;
    whiteLedOn = false;  // Full release also turns off GPIO4 / the white LED.
    releaseMotor();
    sendStatus();
  });

  server.on("/api/led", HTTP_POST, []() {
    if (motorRunning) {
      sendNoCache(409, "application/json", "{\"error\":\"stop motor before setting the shared GPIO4 white LED\"}");
      return;
    }
    whiteLedOn = server.hasArg("on") && server.arg("on") == "1";
    applyIdleOutputs();
    sendStatus();
  });

  server.on("/api/help", HTTP_GET, []() {
    sendNoCache(200, "text/plain; charset=utf-8",
      "POST /api/move?steps=512&interval=2\n"
      "POST /api/run?dir=1&interval=2\n"
      "POST /api/stop\nPOST /api/release\nPOST /api/led?on=1\nGET /api/status\n");
  });

  server.onNotFound([]() { sendNoCache(404, "application/json", "{\"error\":\"not found\"}"); });
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  releaseMotor();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  setupRoutes();
  server.begin();

  Serial.println();
  Serial.printf("Wi-Fi: %s\n", AP_SSID);
  Serial.printf("Open: http://%s\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
  server.handleClient();
  updateMotor();
}

