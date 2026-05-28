#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid     = "UNIT-1";
const char* password = "UNIT-1KEY";
ESP8266WebServer server(80);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int pwmMotorA = 5;
const int pwmMotorB = 4;
const int dirMotorA = 0;
const int dirMotorB = 2;
const int TRIG_PIN  = 14;
const int ECHO_PIN  = 16;

int motorSpeed = 150;
#define OBSTACLE_DISTANCE  20
#define TURN_TIME         175
#define BYPASS_FWD_TIME   1500
#define REALIGN_FWD_TIME  1500

enum SystemMode { MODE_WAIT, MODE_MANUAL, MODE_AUTO };
SystemMode systemMode = MODE_WAIT;
bool emergencyStopped = false;

enum RobotState {
  STATE_FORWARD, STATE_STOP_AT_OBSTACLE,
  STATE_TURN_LEFT_1, STATE_BYPASS_FWD_1,
  STATE_TURN_RIGHT_1, STATE_BYPASS_FWD_2,
  STATE_TURN_RIGHT_2, STATE_BYPASS_FWD_3,
  STATE_TURN_LEFT_2,  STATE_RESUME_FORWARD
};
RobotState robotState = STATE_FORWARD;

String        currentAction = "AWAITING ORDERS";
String        currentStatus = "STANDBY";
long          lastDist      = 0;
unsigned long stateTimer    = 0;
String        manualCmd     = "STOP";

void setup() {
  Serial.begin(115200);
  pinMode(pwmMotorA, OUTPUT); pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorA, OUTPUT); pinMode(dirMotorB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);  pinMode(ECHO_PIN, INPUT);
  Wire.begin(13, 12);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  startupSequence();
  connectWiFi();
  setupRoutes();
  server.begin();
  currentStatus = "STANDBY";
  currentAction = "AWAITING ORDERS";
  updateDisplay();
}

void loop() {
  server.handleClient();
  if (emergencyStopped) { stopCar(); updateDisplay(); return; }
  lastDist = getDistance();
  switch (systemMode) {
    case MODE_WAIT:
      stopCar();
      currentStatus = "STANDBY";
      currentAction = "AWAITING ORDERS";
      manualCmd = "HALT";
      updateDisplay();
      break;
    case MODE_MANUAL:
      currentStatus = "MANUAL";
      updateDisplay();
      break;
    case MODE_AUTO:
      runAutoState();
      break;
  }
}

void runAutoState() {
  switch (robotState) {
    case STATE_FORWARD:
      currentStatus = "AUTO";
      currentAction = "ADVANCING";
      manualCmd = "FWD";
      forward(); updateDisplay();
      if (lastDist > 0 && lastDist < OBSTACLE_DISTANCE) { stopCar(); robotState = STATE_STOP_AT_OBSTACLE; }
      break;
    case STATE_STOP_AT_OBSTACLE:
      stopCar(); currentStatus = "OBSTACLE!"; currentAction = "TARGET DETECTED"; manualCmd = "HALT";
      updateDisplay(); delay(700);
      currentAction = "STEP 1: EVADE L"; stateTimer = millis(); robotState = STATE_TURN_LEFT_1;
      break;
    case STATE_TURN_LEFT_1:
      currentAction = "STEP 1: EVADE L"; turnLeft(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) { stopCar(); delay(300); currentAction = "STEP 2: ADVANCE"; stateTimer = millis(); robotState = STATE_BYPASS_FWD_1; }
      break;
    case STATE_BYPASS_FWD_1:
      currentAction = "STEP 2: ADVANCE"; forward(); updateDisplay();
      if (millis() - stateTimer >= BYPASS_FWD_TIME) { stopCar(); delay(300); currentAction = "STEP 3: ALIGN R"; stateTimer = millis(); robotState = STATE_TURN_RIGHT_1; }
      break;
    case STATE_TURN_RIGHT_1:
      currentAction = "STEP 3: ALIGN R"; turnRight(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) { stopCar(); delay(300); currentAction = "STEP 4: BYPASS"; stateTimer = millis(); robotState = STATE_BYPASS_FWD_2; }
      break;
    case STATE_BYPASS_FWD_2:
      currentAction = "STEP 4: BYPASS"; forward(); updateDisplay();
      if (millis() - stateTimer >= BYPASS_FWD_TIME) { stopCar(); delay(300); currentAction = "STEP 5: ALIGN R"; stateTimer = millis(); robotState = STATE_TURN_RIGHT_2; }
      break;
    case STATE_TURN_RIGHT_2:
      currentAction = "STEP 5: ALIGN R"; turnRight(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) { stopCar(); delay(300); currentAction = "STEP 6: RETURN"; stateTimer = millis(); robotState = STATE_BYPASS_FWD_3; }
      break;
    case STATE_BYPASS_FWD_3:
      currentAction = "STEP 6: RETURN"; forward(); updateDisplay();
      if (millis() - stateTimer >= REALIGN_FWD_TIME) { stopCar(); delay(300); currentAction = "STEP 7: REALIGN"; stateTimer = millis(); robotState = STATE_TURN_LEFT_2; }
      break;
    case STATE_TURN_LEFT_2:
      currentAction = "STEP 7: REALIGN"; turnLeft(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) { stopCar(); delay(300); currentAction = "BYPASS COMPLETE"; currentStatus = "AUTO"; robotState = STATE_RESUME_FORWARD; updateDisplay(); delay(500); }
      break;
    case STATE_RESUME_FORWARD:
      currentStatus = "AUTO"; currentAction = "ADVANCING"; forward(); updateDisplay(); robotState = STATE_FORWARD;
      break;
  }
}

void setupRoutes() {
  server.on("/", []() { server.send(200, "text/html", getControlPage()); });
  server.on("/mode", []() {
    if (server.hasArg("m")) {
      String m = server.arg("m");
      if      (m == "start")  { systemMode = MODE_MANUAL; emergencyStopped = false; currentStatus = "MANUAL"; currentAction = "MANUAL CONTROL"; robotState = STATE_FORWARD; }
      else if (m == "manual") { systemMode = MODE_MANUAL; currentStatus = "MANUAL"; currentAction = "MANUAL CONTROL"; stopCar(); manualCmd = "HALT"; }
      else if (m == "auto")   { systemMode = MODE_AUTO;   currentStatus = "AUTO";   currentAction = "AUTO PATROL";    robotState = STATE_FORWARD; }
      server.send(200, "text/plain", "OK");
    }
  });
  server.on("/cmd", []() {
    if (server.hasArg("action") && systemMode == MODE_MANUAL && !emergencyStopped) {
      String a = server.arg("action"); a.toUpperCase(); manualCmd = a;
      if      (a == "FORWARD")  { currentAction = "ADVANCING";  forward();  }
      else if (a == "BACKWARD") { currentAction = "RETREATING"; backward(); }
      else if (a == "LEFT")     { currentAction = "EVADING L";  turnLeft(); }
      else if (a == "RIGHT")    { currentAction = "EVADING R";  turnRight();}
      else                      { currentAction = "HALTED";     stopCar();  manualCmd = "HALT"; }
      server.send(200, "text/plain", "OK");
    } else { server.send(200, "text/plain", "BLOCKED"); }
  });
  server.on("/estop", []() {
    emergencyStopped = true; currentStatus = "E-STOP"; currentAction = "EMERGENCY HALT"; manualCmd = "HALT";
    stopCar(); server.send(200, "text/plain", "ESTOP");
  });
  server.on("/reset", []() {
    emergencyStopped = false; systemMode = MODE_MANUAL; currentStatus = "MANUAL";
    currentAction = "SYSTEMS ONLINE"; manualCmd = "HALT"; robotState = STATE_FORWARD;
    stopCar(); server.send(200, "text/plain", "RESET");
  });
  server.on("/status", []() {
    String json = "{";
    json += "\"mode\":\"" + currentStatus + "\",";
    json += "\"action\":\"" + currentAction + "\",";
    json += "\"dist\":" + String(lastDist) + ",";
    json += "\"speed\":" + String(motorSpeed) + ",";
    json += "\"cmd\":\"" + manualCmd + "\",";
    json += "\"estop\":" + String(emergencyStopped ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0, 0); display.print("STATUS: ");
  if (currentStatus == "OBSTACLE!" || currentStatus == "E-STOP") {
    display.fillRect(48, 0, 80, 9, WHITE);
    display.setTextColor(BLACK); display.setCursor(50, 0); display.print(currentStatus); display.setTextColor(WHITE);
  } else { display.print(currentStatus); }
  display.drawFastHLine(0, 10, 128, WHITE);
  display.setCursor(0, 13); display.print("ACT : "); display.print(currentAction);
  display.setCursor(0, 23); display.print("DIST: ");
  if (lastDist >= 999) display.print("> 5m"); else { display.print(lastDist); display.print(" cm"); }
  display.setCursor(0, 33); display.print("SPD : "); display.print(motorSpeed); display.print(" / 1023");
  display.drawFastHLine(0, 43, 128, WHITE);
  drawEyesInRegion();
  display.display();
}

void drawEyesInRegion() {
  int eyeWidth = 35, eyeHeight = 14, eyeY = 47;
  if (emergencyStopped) {
    display.drawLine(8, eyeY, 8+eyeWidth, eyeY+eyeHeight, WHITE);
    display.drawLine(8+eyeWidth, eyeY, 8, eyeY+eyeHeight, WHITE);
    display.drawLine(85, eyeY, 85+eyeWidth, eyeY+eyeHeight, WHITE);
    display.drawLine(85+eyeWidth, eyeY, 85, eyeY+eyeHeight, WHITE);
  } else if (currentStatus == "OBSTACLE!") {
    display.fillRoundRect(8,  eyeY, eyeWidth, eyeHeight, 5, WHITE);
    display.fillRoundRect(85, eyeY, eyeWidth, eyeHeight, 5, WHITE);
    display.fillTriangle(8, eyeY, 8+eyeWidth/2, eyeY, 8, eyeY+6, BLACK);
    display.fillTriangle(85+eyeWidth/2, eyeY, 85+eyeWidth, eyeY, 85+eyeWidth, eyeY+6, BLACK);
  } else {
    display.fillRoundRect(8,  eyeY, eyeWidth, eyeHeight, 5, WHITE);
    display.fillRoundRect(85, eyeY, eyeWidth, eyeHeight, 5, WHITE);
  }
}

void connectWiFi() {
  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(10, 25); display.print("Connecting WiFi..."); display.display();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  display.clearDisplay();
  display.setCursor(0, 10); display.print("Connected!");
  display.setCursor(0, 28); display.print("Open browser:");
  display.setCursor(0, 44); display.print(WiFi.localIP()); display.display();
  delay(3000);
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 999;
  return dur * 0.034 / 2;
}

void forward()  { digitalWrite(dirMotorA,LOW);  digitalWrite(dirMotorB,LOW);  analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void backward() { digitalWrite(dirMotorA,HIGH); digitalWrite(dirMotorB,HIGH); analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void turnLeft() { digitalWrite(dirMotorA,HIGH); digitalWrite(dirMotorB,LOW);  analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void turnRight(){ digitalWrite(dirMotorA,LOW);  digitalWrite(dirMotorB,HIGH); analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void stopCar()  { analogWrite(pwmMotorA,0); analogWrite(pwmMotorB,0); }

void startupSequence() {
  display.clearDisplay(); display.setTextSize(2); display.setTextColor(WHITE);
  display.setCursor(20,20); display.print("WELCOME"); display.display(); delay(2000);
  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(10,25); display.print("Service Robot");
  display.setCursor(15,40); display.print("Starting..."); display.display(); delay(1500);
  loadingAnimation();
}

void loadingAnimation() {
  for (int i = 0; i <= 100; i += 10) {
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(40,10); display.print("Loading");
    display.drawRect(14,30,100,10,WHITE); display.fillRect(14,30,i,10,WHITE);
    display.display(); delay(200);
  }
}

String getControlPage() {
  return R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UNIT-7 DEFENCE CONTROL</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Oswald:wght@400;600&display=swap');
  *{box-sizing:border-box;margin:0;padding:0;}
  body{
    background:#000;
    color:#fff;
    font-family:'Share Tech Mono',monospace;
    min-height:100vh;
    display:flex;flex-direction:column;align-items:center;
    padding:16px 12px 24px;
    gap:14px;
  }

  /* scan line overlay */
  body::before{
    content:'';position:fixed;inset:0;pointer-events:none;z-index:999;
    background:repeating-linear-gradient(0deg,rgba(255,255,255,0.015) 0px,rgba(255,255,255,0.015) 1px,transparent 1px,transparent 3px);
  }

  /* ── HEADER ── */
  .header{width:100%;max-width:480px;border:1px solid #fff;padding:8px 14px;display:flex;align-items:center;justify-content:space-between;}
  .header-title{font-family:'Oswald',sans-serif;font-size:1rem;font-weight:600;letter-spacing:4px;}
  .header-sub{font-size:0.6rem;color:#aaa;letter-spacing:2px;}
  .blink{animation:blink 1s step-end infinite;}
  @keyframes blink{50%{opacity:0;}}

  /* ── OLED PANEL ── */
  .oled-frame{
    width:100%;max-width:480px;
    border:1px solid #fff;
    padding:1px;
  }
  .oled-titlebar{
    background:#fff;color:#000;
    font-family:'Oswald',sans-serif;font-size:0.65rem;letter-spacing:3px;
    padding:3px 10px;display:flex;justify-content:space-between;align-items:center;
  }
  .oled-inner{
    background:#000;border:1px solid #333;
    padding:10px 12px;font-size:0.72rem;line-height:1.9;
  }
  .oled-row{display:flex;justify-content:space-between;align-items:center;}
  .oled-label{color:#888;min-width:52px;}
  .oled-val{color:#fff;text-align:right;}
  .oled-divider{border-top:1px solid #fff;margin:5px 0;}
  .highlight-inv{background:#fff;color:#000;padding:0 5px;font-weight:bold;}
  .oled-eyes{display:flex;justify-content:space-between;padding:4px 8px 0;gap:8px;height:22px;}
  .eye{background:#fff;border-radius:3px;flex:1;transition:all 0.2s;}
  .eye.angry{clip-path:polygon(0 45%,50% 0%,100% 0%,100% 100%,0 100%);}
  .eye.dead{background:transparent;border:1px solid #fff;position:relative;}
  .eye.dead::before,.eye.dead::after{content:'';position:absolute;top:50%;left:50%;width:100%;height:1px;background:#fff;}
  .eye.dead::before{transform:translate(-50%,-50%) rotate(35deg);}
  .eye.dead::after{transform:translate(-50%,-50%) rotate(-35deg);}

  /* ── SECTION LABELS ── */
  .section-label{
    width:100%;max-width:480px;
    font-size:0.6rem;letter-spacing:3px;color:#888;
    border-bottom:1px solid #333;padding-bottom:3px;
    display:flex;justify-content:space-between;
  }

  /* ── MODE BUTTONS ── */
  .mode-row{display:flex;gap:8px;width:100%;max-width:480px;}
  .mode-btn{
    flex:1;padding:10px 4px;
    background:#000;border:1px solid #555;
    color:#888;font-family:'Share Tech Mono',monospace;
    font-size:0.7rem;letter-spacing:1px;cursor:pointer;
    transition:all 0.15s;text-transform:uppercase;
  }
  .mode-btn:hover{border-color:#fff;color:#fff;}
  .mode-btn.active{background:#fff;color:#000;border-color:#fff;}

  /* ── D-PAD ── */
  .dpad-wrap{position:relative;width:100%;max-width:480px;}
  .dpad-border{border:1px solid #333;padding:14px;display:flex;flex-direction:column;align-items:center;gap:6px;}
  .dpad-row{display:flex;gap:6px;align-items:center;}
  .btn{
    width:74px;height:74px;
    background:#000;border:1px solid #555;
    color:#fff;font-size:1.4rem;cursor:pointer;
    display:flex;align-items:center;justify-content:center;
    transition:all 0.1s;user-select:none;
    font-family:'Share Tech Mono',monospace;
    letter-spacing:0;
  }
  .btn:hover{border-color:#fff;}
  .btn:active,.btn.active{background:#fff;color:#000;border-color:#fff;}
  .btn.disabled{opacity:0.18;pointer-events:none;}
  .btn-center{width:74px;height:74px;background:#111;border:1px solid #555;display:flex;align-items:center;justify-content:center;font-size:0.6rem;color:#555;letter-spacing:1px;}

  /* ── ESTOP ── */
  .estop-zone{width:100%;max-width:480px;display:flex;flex-direction:column;align-items:center;gap:8px;}
  #btn-ESTOP{
    width:100%;padding:14px;
    background:#000;border:2px solid #fff;
    color:#fff;font-family:'Oswald',sans-serif;
    font-size:1rem;font-weight:600;letter-spacing:4px;
    cursor:pointer;transition:all 0.1s;
    display:flex;align-items:center;justify-content:center;gap:12px;
  }
  #btn-ESTOP:hover{background:#fff;color:#000;}
  #btn-ESTOP:active{transform:scale(0.98);}
  .estop-indicator{
    width:14px;height:14px;border:2px solid #fff;border-radius:50%;
    animation:pulse-ring 1.2s ease-in-out infinite;
  }
  @keyframes pulse-ring{0%,100%{transform:scale(1);opacity:1;}50%{transform:scale(1.3);opacity:0.5;}}
  #btn-ESTOP:hover .estop-indicator{border-color:#000;}

  #estop-banner{
    display:none;width:100%;
    border:1px solid #fff;padding:8px;text-align:center;
    font-size:0.7rem;letter-spacing:4px;
    animation:blink 0.6s step-end infinite;
  }
  #btn-RESET{
    display:none;padding:10px 32px;
    background:#000;border:1px solid #fff;
    color:#fff;font-family:'Share Tech Mono',monospace;
    font-size:0.75rem;letter-spacing:2px;cursor:pointer;
    transition:all 0.15s;
  }
  #btn-RESET:hover{background:#fff;color:#000;}

  /* ── FOOTER ── */
  .footer{
    width:100%;max-width:480px;
    border-top:1px solid #222;padding-top:8px;
    font-size:0.58rem;color:#444;letter-spacing:1px;
    display:flex;justify-content:space-between;flex-wrap:wrap;gap:4px;
    text-align:center;
  }
</style>
</head>
<body>

<!-- HEADER -->
<div class="header">
  <div>
    <div class="header-title">UNIT-7 // DEFENCE BOT</div>
    <div class="header-sub">REMOTE TACTICAL CONTROL SYSTEM</div>
  </div>
  <div style="font-size:0.65rem;color:#888;text-align:right;">
    <span class="blink">&#9632;</span> ONLINE<br>
    <span id="h-ip" style="color:#555;font-size:0.55rem;">---</span>
  </div>
</div>

<!-- OLED MIRROR -->
<div class="oled-frame">
  <div class="oled-titlebar">
    <span>// ONBOARD DISPLAY</span>
    <span id="oled-mode" style="letter-spacing:2px;">STANDBY</span>
  </div>
  <div class="oled-inner">
    <div class="oled-row">
      <span class="oled-label">STATUS</span>
      <span class="oled-val" id="s-status">STANDBY</span>
    </div>
    <div class="oled-divider"></div>
    <div class="oled-row">
      <span class="oled-label">ACT</span>
      <span class="oled-val" id="s-action">AWAITING ORDERS</span>
    </div>
    <div class="oled-row">
      <span class="oled-label">DIST</span>
      <span class="oled-val" id="s-dist">---</span>
    </div>
    <div class="oled-row">
      <span class="oled-label">SPD</span>
      <span class="oled-val"><span id="s-speed">150</span> / 1023</span>
    </div>
    <div class="oled-divider"></div>
    <div class="oled-eyes">
      <div class="eye" id="eye-l"></div>
      <div class="eye" id="eye-r"></div>
    </div>
  </div>
</div>

<!-- MODE SELECT -->
<div class="section-label"><span>// MODE SELECT</span><span>ENTER=START · 1=MANUAL · 2=AUTO</span></div>
<div class="mode-row">
  <button class="mode-btn" id="btn-start"  onclick="setMode('start')">[ ENTER ]<br>DEPLOY</button>
  <button class="mode-btn" id="btn-manual" onclick="setMode('manual')">[  1  ]<br>MANUAL</button>
  <button class="mode-btn" id="btn-auto"   onclick="setMode('auto')">[  2  ]<br>AUTO</button>
</div>

<!-- DIRECTIONAL CONTROL -->
<div class="section-label"><span>// DIRECTIONAL CONTROL</span><span>W A S D</span></div>
<div class="dpad-wrap">
  <div class="dpad-border">
    <div class="dpad-row">
      <button class="btn disabled" id="btn-FORWARD"  ontouchstart="send('FORWARD')"  ontouchend="send('STOP')">&#9650;</button>
    </div>
    <div class="dpad-row">
      <button class="btn disabled" id="btn-LEFT"     ontouchstart="send('LEFT')"     ontouchend="send('STOP')">&#9668;</button>
      <div class="btn-center">HALT</div>
      <button class="btn disabled" id="btn-RIGHT"    ontouchstart="send('RIGHT')"    ontouchend="send('STOP')">&#9658;</button>
    </div>
    <div class="dpad-row">
      <button class="btn disabled" id="btn-BACKWARD" ontouchstart="send('BACKWARD')" ontouchend="send('STOP')">&#9660;</button>
    </div>
  </div>
</div>

<!-- EMERGENCY -->
<div class="section-label"><span>// EMERGENCY OVERRIDE</span><span>KEY: I</span></div>
<div class="estop-zone">
  <div id="estop-banner">!! EMERGENCY STOP ENGAGED !!</div>
  <button id="btn-ESTOP" onclick="emergencyStop()">
    <div class="estop-indicator"></div>
    EMERGENCY STOP
    <div class="estop-indicator"></div>
  </button>
  <button id="btn-RESET" onclick="resetStop()">[ RESET &amp; RESUME OPERATIONS ]</button>
</div>

<!-- FOOTER -->
<div class="footer">
  <span>ENTER=DEPLOY</span>
  <span>1=MANUAL · 2=AUTO</span>
  <span>WASD=DRIVE</span>
  <span>I=E-STOP</span>
</div>

<script>
  const keyMap = {
    'w':'FORWARD','ArrowUp':'FORWARD',
    's':'BACKWARD','ArrowDown':'BACKWARD',
    'a':'LEFT','ArrowLeft':'LEFT',
    'd':'RIGHT','ArrowRight':'RIGHT'
  };
  let active=null, stopped=false, currentMode='wait';

  function setMode(m) {
    fetch('/mode?m='+m).catch(()=>{});
    currentMode=(m==='start')?'manual':m;
    ['btn-start','btn-manual','btn-auto'].forEach(id=>document.getElementById(id).classList.remove('active'));
    if(m==='start'||m==='manual') document.getElementById('btn-manual').classList.add('active');
    if(m==='auto') document.getElementById('btn-auto').classList.add('active');
    const ids=['FORWARD','BACKWARD','LEFT','RIGHT'];
    ids.forEach(id=>{
      const b=document.getElementById('btn-'+id);
      if(currentMode==='manual'&&!stopped) b.classList.remove('disabled');
      else b.classList.add('disabled');
    });
  }

  function send(action) {
    if(stopped||currentMode!=='manual') return;
    document.querySelectorAll('.btn').forEach(b=>b.classList.remove('active'));
    const b=document.getElementById('btn-'+action);
    if(b) b.classList.add('active');
    fetch('/cmd?action='+action).catch(()=>{});
  }

  function emergencyStop() {
    stopped=true; active=null;
    document.getElementById('estop-banner').style.display='block';
    document.getElementById('btn-RESET').style.display='block';
    ['FORWARD','BACKWARD','LEFT','RIGHT'].forEach(id=>document.getElementById('btn-'+id).classList.add('disabled'));
    fetch('/estop').catch(()=>{});
  }

  function resetStop() {
    stopped=false;
    document.getElementById('estop-banner').style.display='none';
    document.getElementById('btn-RESET').style.display='none';
    setMode('manual');
    fetch('/reset').catch(()=>{});
  }

  document.addEventListener('keydown',e=>{
    if(e.key==='Enter')              {setMode('start');return;}
    if(e.key==='1')                  {setMode('manual');return;}
    if(e.key==='2')                  {setMode('auto');return;}
    if(e.key==='i'||e.key==='I')     {emergencyStop();return;}
    const cmd=keyMap[e.key];
    if(cmd&&active!==cmd){active=cmd;send(cmd);}
  });
  document.addEventListener('keyup',e=>{
    if(keyMap[e.key]){active=null;send('STOP');}
  });

  function updateOLED(d) {
    document.getElementById('s-action').textContent = d.action;
    document.getElementById('s-speed').textContent  = d.speed;
    document.getElementById('s-dist').textContent   = d.dist>=999 ? '> 5m' : d.dist+' cm';
    document.getElementById('oled-mode').textContent = d.mode;

    const stEl=document.getElementById('s-status');
    if(d.estop)             stEl.innerHTML='<span class="highlight-inv">E-STOP</span>';
    else if(d.mode==='OBSTACLE!') stEl.innerHTML='<span class="highlight-inv">OBSTACLE!</span>';
    else                    stEl.textContent=d.mode;

    const el=document.getElementById('eye-l');
    const er=document.getElementById('eye-r');
    el.className='eye'; er.className='eye';
    if(d.estop)            {el.classList.add('dead'); er.classList.add('dead');}
    else if(d.mode==='OBSTACLE!') {el.classList.add('angry'); er.classList.add('angry');}
  }

  setInterval(()=>{ fetch('/status').then(r=>r.json()).then(updateOLED).catch(()=>{}); },400);
</script>
</body>
</html>
)rawhtml";
}
