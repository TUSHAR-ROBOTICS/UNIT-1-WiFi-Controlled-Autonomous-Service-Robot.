#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid     = "tushar";
const char* password = "tushar11";
ESP8266WebServer server(80);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int pwmMotorA = 5;
const int pwmMotorB = 4;
const int dirMotorA = 0;
const int dirMotorB = 2;
const int TRIG_PIN  = 13;
const int ECHO_PIN  = 15;

int motorSpeed = 150;
#define OBSTACLE_DISTANCE_AUTO   20    // cm — auto mode threshold
#define OBSTACLE_DISTANCE_SMART  30    // cm — smart manual threshold (NEW)
#define TURN_TIME         175
#define BYPASS_FWD_TIME   1500
#define REALIGN_FWD_TIME  1500

// ── System Mode ───────────────────────────────────────────────
// MODE_WAIT    = standby
// MODE_MANUAL  = key 1 — pure manual, NO auto-avoid
// MODE_AUTO    = key 2 — full autonomous
// MODE_SMART   = key 3 — manual drive + auto obstacle avoid (NEW)
enum SystemMode { MODE_WAIT, MODE_MANUAL, MODE_AUTO, MODE_SMART };
SystemMode systemMode = MODE_WAIT;

bool emergencyStopped  = false;
bool smartAvoiding     = false;   // true while smart-mode is mid-bypass

enum RobotState {
  STATE_FORWARD, STATE_STOP_AT_OBSTACLE,
  STATE_TURN_LEFT_1, STATE_BYPASS_FWD_1,
  STATE_TURN_RIGHT_1, STATE_BYPASS_FWD_2,
  STATE_TURN_RIGHT_2, STATE_BYPASS_FWD_3,
  STATE_TURN_LEFT_2,  STATE_RESUME_FORWARD
};
RobotState robotState = STATE_FORWARD;

String        currentAction  = "AWAITING ORDERS";
String        currentStatus  = "STANDBY";
long          lastDist       = 0;
bool          obstacleNear   = false;   // NEW — for display section
unsigned long stateTimer     = 0;
String        manualCmd      = "HALT";

// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(pwmMotorA, OUTPUT); pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorA, OUTPUT); pinMode(dirMotorB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);  pinMode(ECHO_PIN, INPUT);
  Wire.begin(14, 12);
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

// ═════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();

  if (emergencyStopped) {
    stopCar();
    updateDisplay();
    return;
  }

  lastDist    = getDistance();
  obstacleNear = (lastDist > 0 && lastDist < OBSTACLE_DISTANCE_SMART);

  switch (systemMode) {

    case MODE_WAIT:
      stopCar();
      currentStatus = "STANDBY";
      currentAction = "AWAITING ORDERS";
      manualCmd     = "HALT";
      updateDisplay();
      break;

    case MODE_MANUAL:
      // Pure manual — no auto avoid, just show sensor data
      if (lastDist > 0 && lastDist < OBSTACLE_DISTANCE_SMART)
        currentStatus = "WARNING";       // soft warning only, no action
      else
        currentStatus = "MANUAL";
      updateDisplay();
      break;

    // ── AUTO: full autonomous bypass ─────────────────────────
    case MODE_AUTO:
      runAutoState(OBSTACLE_DISTANCE_AUTO);
      break;

    // ── SMART: manual drive + auto bypass at 30 cm ───────────
    case MODE_SMART:
      runSmartMode();
      break;
  }
}

// ══ AUTO STATE MACHINE (used by both AUTO and SMART) ══════════
void runAutoState(int threshold) {
  switch (robotState) {

    case STATE_FORWARD:
      currentStatus = (systemMode == MODE_SMART) ? "SMART" : "AUTO";
      currentAction = "ADVANCING";
      manualCmd     = "FWD";
      if (!smartAvoiding) forward();  // in smart mode driver still controls
      updateDisplay();
      if (lastDist > 0 && lastDist < threshold) {
        stopCar(); robotState = STATE_STOP_AT_OBSTACLE;
      }
      break;

    case STATE_STOP_AT_OBSTACLE:
      stopCar();
      currentStatus = "OBSTACLE!";
      currentAction = "TARGET DETECTED";
      manualCmd     = "HALT";
      updateDisplay(); delay(700);
      currentAction = "STEP 1: EVADE L";
      stateTimer    = millis();
      robotState    = STATE_TURN_LEFT_1;
      break;

    case STATE_TURN_LEFT_1:
      currentAction = "STEP 1: EVADE L"; turnLeft(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) {
        stopCar(); delay(300);
        currentAction = "STEP 2: ADVANCE";
        stateTimer = millis(); robotState = STATE_BYPASS_FWD_1;
      }
      break;

    case STATE_BYPASS_FWD_1:
      currentAction = "STEP 2: ADVANCE"; forward(); updateDisplay();
      if (millis() - stateTimer >= BYPASS_FWD_TIME) {
        stopCar(); delay(300);
        currentAction = "STEP 3: ALIGN R";
        stateTimer = millis(); robotState = STATE_TURN_RIGHT_1;
      }
      break;

    case STATE_TURN_RIGHT_1:
      currentAction = "STEP 3: ALIGN R"; turnRight(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) {
        stopCar(); delay(300);
        currentAction = "STEP 4: BYPASS";
        stateTimer = millis(); robotState = STATE_BYPASS_FWD_2;
      }
      break;

    case STATE_BYPASS_FWD_2:
      currentAction = "STEP 4: BYPASS"; forward(); updateDisplay();
      if (millis() - stateTimer >= BYPASS_FWD_TIME) {
        stopCar(); delay(300);
        currentAction = "STEP 5: ALIGN R";
        stateTimer = millis(); robotState = STATE_TURN_RIGHT_2;
      }
      break;

    case STATE_TURN_RIGHT_2:
      currentAction = "STEP 5: ALIGN R"; turnRight(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) {
        stopCar(); delay(300);
        currentAction = "STEP 6: RETURN";
        stateTimer = millis(); robotState = STATE_BYPASS_FWD_3;
      }
      break;

    case STATE_BYPASS_FWD_3:
      currentAction = "STEP 6: RETURN"; forward(); updateDisplay();
      if (millis() - stateTimer >= REALIGN_FWD_TIME) {
        stopCar(); delay(300);
        currentAction = "STEP 7: REALIGN";
        stateTimer = millis(); robotState = STATE_TURN_LEFT_2;
      }
      break;

    case STATE_TURN_LEFT_2:
      currentAction = "STEP 7: REALIGN"; turnLeft(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) {
        stopCar(); delay(300);
        currentAction = "BYPASS COMPLETE";
        currentStatus = (systemMode == MODE_SMART) ? "SMART" : "AUTO";
        smartAvoiding = false;
        robotState = STATE_RESUME_FORWARD;
        updateDisplay(); delay(500);
      }
      break;

    case STATE_RESUME_FORWARD:
      currentStatus = (systemMode == MODE_SMART) ? "SMART" : "AUTO";
      currentAction = (systemMode == MODE_SMART) ? "MANUAL CTRL" : "ADVANCING";
      if (systemMode == MODE_AUTO) forward();
      updateDisplay();
      robotState = STATE_FORWARD;
      break;
  }
}

// ══ SMART MODE: manual + auto-avoid ══════════════════════════
void runSmartMode() {
  // If mid-bypass, keep running the state machine
  if (smartAvoiding) {
    runAutoState(OBSTACLE_DISTANCE_SMART);
    return;
  }

  // Normal smart mode — check for obstacle
  currentStatus = "SMART";
  if (lastDist > 0 && lastDist < OBSTACLE_DISTANCE_SMART) {
    // Trigger bypass regardless of what key driver pressed
    smartAvoiding = true;
    robotState    = STATE_STOP_AT_OBSTACLE;
    currentAction = "AUTO-EVADE!";
    manualCmd     = "EVADE";
    runAutoState(OBSTACLE_DISTANCE_SMART);
  } else {
    // Normal manual control allowed
    currentAction = "MANUAL CTRL";
    updateDisplay();
    // Motor is controlled by /cmd web calls
  }
}

// ══ Web Routes ════════════════════════════════════════════════
void setupRoutes() {
  server.on("/", []() { server.send(200, "text/html", getControlPage()); });

  server.on("/mode", []() {
    if (server.hasArg("m")) {
      String m = server.arg("m");
      smartAvoiding = false;
      robotState    = STATE_FORWARD;
      if      (m == "start")  { systemMode = MODE_MANUAL; emergencyStopped = false; currentStatus = "MANUAL"; currentAction = "MANUAL CONTROL"; }
      else if (m == "manual") { systemMode = MODE_MANUAL; currentStatus = "MANUAL"; currentAction = "MANUAL CONTROL"; stopCar(); manualCmd = "HALT"; }
      else if (m == "auto")   { systemMode = MODE_AUTO;   currentStatus = "AUTO";   currentAction = "AUTO PATROL"; }
      else if (m == "smart")  { systemMode = MODE_SMART;  currentStatus = "SMART";  currentAction = "SMART PATROL"; manualCmd = "HALT"; }
      server.send(200, "text/plain", "OK");
    }
  });

  server.on("/cmd", []() {
    if (server.hasArg("action") && !emergencyStopped) {
      // Allow manual drive in MANUAL and SMART (unless smart is mid-bypass)
      if ((systemMode == MODE_MANUAL || (systemMode == MODE_SMART && !smartAvoiding))) {
        String a = server.arg("action"); a.toUpperCase(); manualCmd = a;
        if      (a == "FORWARD")  { currentAction = "ADVANCING";  forward();  }
        else if (a == "BACKWARD") { currentAction = "RETREATING"; backward(); }
        else if (a == "LEFT")     { currentAction = "EVADING L";  turnLeft(); }
        else if (a == "RIGHT")    { currentAction = "EVADING R";  turnRight();}
        else                      { currentAction = "HALTED";     stopCar();  manualCmd = "HALT"; }
        server.send(200, "text/plain", "OK");
      } else {
        server.send(200, "text/plain", "BLOCKED");
      }
    }
  });

  server.on("/estop", []() {
    emergencyStopped = true; smartAvoiding = false;
    currentStatus = "E-STOP"; currentAction = "EMERGENCY HALT"; manualCmd = "HALT";
    stopCar(); server.send(200, "text/plain", "ESTOP");
  });

  server.on("/reset", []() {
    emergencyStopped = false; smartAvoiding = false;
    systemMode = MODE_MANUAL; currentStatus = "MANUAL";
    currentAction = "SYSTEMS ONLINE"; manualCmd = "HALT";
    robotState = STATE_FORWARD; stopCar();
    server.send(200, "text/plain", "RESET");
  });

  server.on("/status", []() {
    String json = "{";
    json += "\"mode\":\""   + currentStatus  + "\",";
    json += "\"action\":\"" + currentAction  + "\",";
    json += "\"dist\":"     + String(lastDist) + ",";
    json += "\"speed\":"    + String(motorSpeed) + ",";
    json += "\"cmd\":\""    + manualCmd      + "\",";
    json += "\"obstacle\":" + String(obstacleNear ? "true" : "false") + ",";
    json += "\"avoiding\":" + String(smartAvoiding ? "true" : "false") + ",";
    json += "\"estop\":"    + String(emergencyStopped ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });
}

// ══ OLED Display ══════════════════════════════════════════════
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(WHITE);

  // Row 1 — Status
  display.setCursor(0, 0); display.print("STATUS: ");
  if (currentStatus == "OBSTACLE!" || currentStatus == "E-STOP" || currentStatus == "WARNING") {
    display.fillRect(48, 0, 80, 9, WHITE);
    display.setTextColor(BLACK); display.setCursor(50, 0); display.print(currentStatus); display.setTextColor(WHITE);
  } else { display.print(currentStatus); }

  display.drawFastHLine(0, 10, 128, WHITE);

  // Row 2 — Action
  display.setCursor(0, 13); display.print("ACT : "); display.print(currentAction);

  // Row 3 — Distance + obstacle indicator
  display.setCursor(0, 23); display.print("DIST: ");
  if (lastDist >= 999) display.print("> 5m");
  else { display.print(lastDist); display.print(" cm"); }
  // Obstacle dot indicator on same row
  if (obstacleNear) {
    display.fillRect(110, 23, 8, 8, WHITE);  // filled square = obstacle
  } else {
    display.drawRect(110, 23, 8, 8, WHITE);  // empty square = clear
  }

  // Row 4 — Speed
  display.setCursor(0, 33); display.print("SPD : "); display.print(motorSpeed); display.print(" / 1023");

  display.drawFastHLine(0, 43, 128, WHITE);

  // Eyes
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
  } else if (currentStatus == "OBSTACLE!" || currentStatus == "WARNING") {
    display.fillRoundRect(8,  eyeY, eyeWidth, eyeHeight, 5, WHITE);
    display.fillRoundRect(85, eyeY, eyeWidth, eyeHeight, 5, WHITE);
    display.fillTriangle(8, eyeY, 8+eyeWidth/2, eyeY, 8, eyeY+6, BLACK);
    display.fillTriangle(85+eyeWidth/2, eyeY, 85+eyeWidth, eyeY, 85+eyeWidth, eyeY+6, BLACK);
  } else {
    display.fillRoundRect(8,  eyeY, eyeWidth, eyeHeight, 5, WHITE);
    display.fillRoundRect(85, eyeY, eyeWidth, eyeHeight, 5, WHITE);
  }
}

// ══ WiFi ══════════════════════════════════════════════════════
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

// ══ Ultrasonic ════════════════════════════════════════════════
long getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 999;
  return dur * 0.034 / 2;
}

// ══ Motors ════════════════════════════════════════════════════
void forward()  { digitalWrite(dirMotorA,LOW);  digitalWrite(dirMotorB,LOW);  analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void backward() { digitalWrite(dirMotorA,HIGH); digitalWrite(dirMotorB,HIGH); analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void turnLeft() { digitalWrite(dirMotorA,HIGH); digitalWrite(dirMotorB,LOW);  analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void turnRight(){ digitalWrite(dirMotorA,LOW);  digitalWrite(dirMotorB,HIGH); analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void stopCar()  { analogWrite(pwmMotorA,0); analogWrite(pwmMotorB,0); }

// ══ Startup ═══════════════════════════════════════════════════
void startupSequence() {
  display.clearDisplay(); display.setTextSize(2); display.setTextColor(WHITE);
  display.setCursor(20,20); display.print("WELCOME"); display.display(); delay(2000);
  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(10,25); display.print("NEXUS SR");
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

// ══ HTML Page ════════════════════════════════════════════════
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
    background:#000;color:#fff;
    font-family:'Share Tech Mono',monospace;
    min-height:100vh;display:flex;flex-direction:column;
    align-items:center;padding:16px 12px 28px;gap:13px;
  }
  body::before{
    content:'';position:fixed;inset:0;pointer-events:none;z-index:999;
    background:repeating-linear-gradient(0deg,rgba(255,255,255,0.016) 0px,rgba(255,255,255,0.016) 1px,transparent 1px,transparent 3px);
  }
  .w{width:100%;max-width:480px;}

  /* ── HEADER ── */
  .header{border:1px solid #fff;padding:7px 12px;display:flex;align-items:center;justify-content:space-between;}
  .header-title{font-family:'Oswald',sans-serif;font-size:0.95rem;font-weight:600;letter-spacing:4px;}
  .header-sub{font-size:0.58rem;color:#aaa;letter-spacing:2px;}
  .blink{animation:blink 1s step-end infinite;}
  @keyframes blink{50%{opacity:0;}}

  /* ── OLED MIRROR ── */
  .oled-frame{border:1px solid #fff;padding:1px;}
  .oled-tb{background:#fff;color:#000;font-family:'Oswald',sans-serif;font-size:0.62rem;letter-spacing:3px;padding:3px 10px;display:flex;justify-content:space-between;}
  .oled-in{background:#000;border:1px solid #333;padding:9px 12px;font-size:0.7rem;line-height:1.85;}
  .oled-r{display:flex;justify-content:space-between;align-items:center;}
  .oled-l{color:#888;min-width:52px;}
  .oled-v{color:#fff;}
  .odiv{border-top:1px solid #fff;margin:5px 0;}
  .eyes{display:flex;justify-content:space-between;padding:4px 8px 0;gap:8px;height:20px;}
  .eye{background:#fff;border-radius:3px;flex:1;transition:all 0.2s;}
  .eye.angry{clip-path:polygon(0 45%,50% 0%,100% 0%,100% 100%,0 100%);}
  .eye.dead{background:transparent;border:1px solid #fff;position:relative;}
  .eye.dead::before,.eye.dead::after{content:'';position:absolute;top:50%;left:50%;width:100%;height:1px;background:#fff;}
  .eye.dead::before{transform:translate(-50%,-50%) rotate(35deg);}
  .eye.dead::after{transform:translate(-50%,-50%) rotate(-35deg);}
  .hinv{background:#fff;color:#000;padding:0 4px;font-weight:bold;}

  /* ── OBJECT DETECTION SECTION (NEW) ── */
  .radar-frame{border:1px solid #fff;padding:1px;}
  .radar-tb{background:#fff;color:#000;font-family:'Oswald',sans-serif;font-size:0.62rem;letter-spacing:3px;padding:3px 10px;display:flex;justify-content:space-between;}
  .radar-body{background:#000;border:1px solid #333;padding:10px 14px;display:flex;align-items:center;gap:14px;}
  .radar-arc{position:relative;width:70px;height:38px;overflow:hidden;flex-shrink:0;}
  .radar-arc-bg{position:absolute;bottom:0;left:50%;transform:translateX(-50%);width:70px;height:70px;border-radius:50%;border:1px solid #333;clip-path:polygon(0 50%,100% 50%,100% 0%,0 0%);}
  .radar-arc-ring{position:absolute;bottom:0;left:50%;transform:translateX(-50%);width:46px;height:46px;border-radius:50%;border:1px solid #444;clip-path:polygon(0 50%,100% 50%,100% 0%,0 0%);}
  .radar-arc-ring2{position:absolute;bottom:0;left:50%;transform:translateX(-50%);width:24px;height:24px;border-radius:50%;border:1px solid #444;clip-path:polygon(0 50%,100% 50%,100% 0%,0 0%);}
  .radar-sweep{position:absolute;bottom:0;left:50%;width:1px;height:34px;background:#fff;transform-origin:bottom center;transform:rotate(0deg);transition:transform 0.4s linear;}
  .radar-dot{position:absolute;bottom:0;left:50%;width:6px;height:6px;border-radius:50%;background:#fff;transform:translate(-50%,-5px);display:none;}
  .radar-stats{flex:1;display:flex;flex-direction:column;gap:5px;font-size:0.68rem;}
  .rstat{display:flex;justify-content:space-between;align-items:center;}
  .rstat-l{color:#888;}
  .rstat-v{color:#fff;font-size:0.72rem;}
  .dist-bar-wrap{width:100%;height:6px;background:#111;border:1px solid #333;margin-top:3px;}
  .dist-bar{height:100%;background:#fff;transition:width 0.3s;}
  .obj-badge{
    display:inline-block;padding:2px 8px;font-size:0.6rem;letter-spacing:2px;
    border:1px solid #fff;color:#fff;background:#000;transition:all 0.2s;
  }
  .obj-badge.detected{background:#fff;color:#000;animation:flash 0.4s step-end infinite;}
  @keyframes flash{50%{opacity:0.4;}}
  .threat-row{display:flex;gap:6px;margin-top:4px;flex-wrap:wrap;}
  .zone{font-size:0.58rem;padding:2px 6px;border:1px solid #333;color:#555;letter-spacing:1px;}
  .zone.hot{border-color:#fff;color:#fff;background:#fff;color:#000;}

  /* ── SECTION LABEL ── */
  .slbl{font-size:0.58rem;letter-spacing:3px;color:#555;border-bottom:1px solid #1a1a1a;padding-bottom:3px;display:flex;justify-content:space-between;}

  /* ── MODE BUTTONS ── */
  .mrow{display:flex;gap:7px;}
  .mbtn{flex:1;padding:9px 4px;background:#000;border:1px solid #444;color:#666;font-family:'Share Tech Mono',monospace;font-size:0.63rem;letter-spacing:1px;cursor:pointer;text-transform:uppercase;transition:all 0.15s;line-height:1.6;}
  .mbtn:hover{border-color:#fff;color:#fff;}
  .mbtn.act{background:#fff;color:#000;border-color:#fff;}
  /* smart mode button accent */
  #btn-smart{border-color:#555;}
  #btn-smart.act{background:#fff;color:#000;}

  /* ── D-PAD ── */
  .dpad{border:1px solid #222;padding:12px;display:flex;flex-direction:column;align-items:center;gap:5px;}
  .drow{display:flex;gap:5px;align-items:center;}
  .db{width:70px;height:70px;background:#000;border:1px solid #444;color:#fff;font-size:1.4rem;display:flex;align-items:center;justify-content:center;cursor:pointer;transition:all 0.1s;user-select:none;}
  .db:hover{border-color:#fff;}
  .db:active,.db.active{background:#fff;color:#000;border-color:#fff;}
  .db.disabled{opacity:0.18;pointer-events:none;}
  .dc{width:70px;height:70px;background:#0a0a0a;border:1px solid #222;display:flex;align-items:center;justify-content:center;font-size:0.55rem;color:#444;letter-spacing:1px;}

  /* smart override notice */
  #smart-notice{
    display:none;width:100%;text-align:center;
    font-size:0.6rem;letter-spacing:2px;color:#888;
    border:1px solid #333;padding:5px;
    animation:blink 0.8s step-end infinite;
  }

  /* ── ESTOP ── */
  .ez{display:flex;flex-direction:column;align-items:center;gap:7px;}
  #btn-ESTOP{
    width:100%;padding:13px;background:#000;border:2px solid #fff;
    color:#fff;font-family:'Oswald',sans-serif;font-size:0.95rem;font-weight:600;
    letter-spacing:4px;cursor:pointer;display:flex;align-items:center;
    justify-content:center;gap:12px;transition:all 0.1s;
  }
  #btn-ESTOP:hover{background:#fff;color:#000;}
  #btn-ESTOP:active{transform:scale(0.98);}
  .ering{width:13px;height:13px;border:2px solid #fff;border-radius:50%;animation:pr 1.2s ease-in-out infinite;}
  #btn-ESTOP:hover .ering{border-color:#000;}
  @keyframes pr{0%,100%{transform:scale(1);opacity:1;}50%{transform:scale(1.3);opacity:0.4;}}
  #estop-banner{display:none;width:100%;border:1px solid #fff;padding:8px;text-align:center;font-size:0.7rem;letter-spacing:4px;animation:blink 0.6s step-end infinite;}
  #btn-RESET{display:none;padding:10px 32px;background:#000;border:1px solid #fff;color:#fff;font-family:'Share Tech Mono',monospace;font-size:0.72rem;letter-spacing:2px;cursor:pointer;transition:all 0.15s;}
  #btn-RESET:hover{background:#fff;color:#000;}

  /* ── FOOTER ── */
  .foot{border-top:1px solid #1a1a1a;padding-top:7px;font-size:0.55rem;color:#333;letter-spacing:1px;display:flex;justify-content:space-between;flex-wrap:wrap;gap:3px;}
</style>
</head>
<body>

<!-- HEADER -->
<div class="header w">
  <div>
    <div class="header-title">UNIT-7 // DEFENCE BOT</div>
    <div class="header-sub">REMOTE TACTICAL CONTROL SYSTEM</div>
  </div>
  <div style="font-size:0.62rem;color:#888;text-align:right;">
    <span class="blink">&#9632;</span> ONLINE<br>
    <span style="color:#444;font-size:0.54rem;" id="h-mode">STANDBY</span>
  </div>
</div>

<!-- OLED MIRROR -->
<div class="oled-frame w">
  <div class="oled-tb"><span>// ONBOARD DISPLAY</span><span id="oled-mode">STANDBY</span></div>
  <div class="oled-in">
    <div class="oled-r"><span class="oled-l">STATUS</span><span class="oled-v" id="s-status">STANDBY</span></div>
    <div class="odiv"></div>
    <div class="oled-r"><span class="oled-l">ACT</span><span class="oled-v" id="s-action">AWAITING ORDERS</span></div>
    <div class="oled-r">
      <span class="oled-l">DIST</span>
      <span class="oled-v" id="s-dist">---</span>
    </div>
    <div class="oled-r"><span class="oled-l">SPD</span><span class="oled-v"><span id="s-speed">150</span> / 1023</span></div>
    <div class="odiv"></div>
    <div class="eyes"><div class="eye" id="eye-l"></div><div class="eye" id="eye-r"></div></div>
  </div>
</div>

<!-- ══ OBJECT DETECTION SECTION (NEW) ══ -->
<div class="slbl w"><span>// OBJECT DETECTION RADAR</span><span>LIVE SENSOR FEED</span></div>
<div class="radar-frame w">
  <div class="radar-tb">
    <span>// PROXIMITY SENSOR</span>
    <span id="radar-badge" class="obj-badge">CLEAR</span>
  </div>
  <div class="radar-body">

    <!-- Mini radar arc visual -->
    <div class="radar-arc">
      <div class="radar-arc-bg"></div>
      <div class="radar-arc-ring"></div>
      <div class="radar-arc-ring2"></div>
      <div class="radar-sweep" id="radar-sweep"></div>
      <div class="radar-dot" id="radar-dot"></div>
    </div>

    <!-- Stats -->
    <div class="radar-stats">
      <div class="rstat">
        <span class="rstat-l">RANGE</span>
        <span class="rstat-v" id="r-dist">--- cm</span>
      </div>
      <div class="dist-bar-wrap">
        <div class="dist-bar" id="r-bar" style="width:0%"></div>
      </div>
      <div class="rstat" style="margin-top:3px;">
        <span class="rstat-l">STATUS</span>
        <span class="rstat-v" id="r-status">SCANNING</span>
      </div>
      <div class="rstat">
        <span class="rstat-l">THRESHOLD</span>
        <span class="rstat-v" id="r-thresh">30 cm</span>
      </div>
      <!-- Threat zones -->
      <div class="threat-row">
        <div class="zone" id="z-safe">SAFE &gt;30</div>
        <div class="zone" id="z-warn">WARN 15-30</div>
        <div class="zone" id="z-crit">CRIT &lt;15</div>
      </div>
    </div>
  </div>
</div>

<!-- MODE SELECT -->
<div class="slbl w"><span>// MODE SELECT</span><span>ENTER=START · 1=MANUAL · 2=AUTO · 3=SMART</span></div>
<div class="mrow w">
  <button class="mbtn" id="btn-start"  onclick="setMode('start')">[ ENTER ]<br>DEPLOY</button>
  <button class="mbtn" id="btn-manual" onclick="setMode('manual')">[  1  ]<br>MANUAL</button>
  <button class="mbtn" id="btn-auto"   onclick="setMode('auto')">[  2  ]<br>AUTO</button>
  <button class="mbtn" id="btn-smart"  onclick="setMode('smart')">[  3  ]<br>SMART</button>
</div>

<!-- D-PAD -->
<div class="slbl w"><span>// DIRECTIONAL CONTROL</span><span>W A S D</span></div>
<div class="dpad w">
  <div class="drow">
    <button class="db disabled" id="btn-FORWARD"  ontouchstart="send('FORWARD')"  ontouchend="send('STOP')">&#9650;</button>
  </div>
  <div class="drow">
    <button class="db disabled" id="btn-LEFT"     ontouchstart="send('LEFT')"     ontouchend="send('STOP')">&#9668;</button>
    <div class="dc">HALT</div>
    <button class="db disabled" id="btn-RIGHT"    ontouchstart="send('RIGHT')"    ontouchend="send('STOP')">&#9658;</button>
  </div>
  <div class="drow">
    <button class="db disabled" id="btn-BACKWARD" ontouchstart="send('BACKWARD')" ontouchend="send('STOP')">&#9660;</button>
  </div>
</div>
<div id="smart-notice" class="w">!! AUTO-EVADE ACTIVE — MANUAL OVERRIDE LOCKED !!</div>

<!-- EMERGENCY -->
<div class="slbl w"><span>// EMERGENCY OVERRIDE</span><span>KEY: I</span></div>
<div class="ez w">
  <div id="estop-banner">!! EMERGENCY STOP ENGAGED !!</div>
  <button id="btn-ESTOP" onclick="emergencyStop()">
    <div class="ering"></div>EMERGENCY STOP<div class="ering"></div>
  </button>
  <button id="btn-RESET" onclick="resetStop()">[ RESET &amp; RESUME OPERATIONS ]</button>
</div>

<!-- FOOTER -->
<div class="foot w">
  <span>ENTER=DEPLOY</span><span>1=MANUAL · 2=AUTO · 3=SMART</span>
  <span>WASD=DRIVE</span><span>I=E-STOP</span>
</div>

<script>
  const keyMap = {'w':'FORWARD','ArrowUp':'FORWARD','s':'BACKWARD','ArrowDown':'BACKWARD','a':'LEFT','ArrowLeft':'LEFT','d':'RIGHT','ArrowRight':'RIGHT'};
  let active=null, stopped=false, currentMode='wait';
  const dpadIds=['FORWARD','BACKWARD','LEFT','RIGHT'];

  // ── Radar sweep animation ────────────────────────────────────
  let sweepAngle = -90;
  setInterval(()=>{
    sweepAngle = (sweepAngle + 4) % 180 - 90;
    document.getElementById('radar-sweep').style.transform = 'rotate('+sweepAngle+'deg)';
  }, 40);

  // ── Mode ─────────────────────────────────────────────────────
  function setMode(m) {
    fetch('/mode?m='+m).catch(()=>{});
    currentMode = (m==='start') ? 'manual' : m;
    ['btn-start','btn-manual','btn-auto','btn-smart'].forEach(id=>document.getElementById(id).classList.remove('act'));
    if(m==='start'||m==='manual') document.getElementById('btn-manual').classList.add('act');
    if(m==='auto')   document.getElementById('btn-auto').classList.add('act');
    if(m==='smart')  document.getElementById('btn-smart').classList.add('act');

    document.getElementById('h-mode').textContent = currentMode.toUpperCase();

    // Update threshold display
    document.getElementById('r-thresh').textContent = (currentMode==='auto') ? '20 cm' : '30 cm';

    enableDpad(currentMode==='manual' || currentMode==='smart');
  }

  function enableDpad(on) {
    dpadIds.forEach(id=>{
      const b=document.getElementById('btn-'+id);
      if(on&&!stopped) b.classList.remove('disabled');
      else b.classList.add('disabled');
    });
  }

  // ── Send movement ────────────────────────────────────────────
  function send(action) {
    if(stopped) return;
    if(currentMode!=='manual'&&currentMode!=='smart') return;
    document.querySelectorAll('.db').forEach(b=>b.classList.remove('active'));
    const b=document.getElementById('btn-'+action);
    if(b) b.classList.add('active');
    fetch('/cmd?action='+action).catch(()=>{});
  }

  // ── E-STOP ───────────────────────────────────────────────────
  function emergencyStop() {
    stopped=true; active=null;
    document.getElementById('estop-banner').style.display='block';
    document.getElementById('btn-RESET').style.display='block';
    document.getElementById('smart-notice').style.display='none';
    dpadIds.forEach(id=>document.getElementById('btn-'+id).classList.add('disabled'));
    fetch('/estop').catch(()=>{});
  }

  function resetStop() {
    stopped=false;
    document.getElementById('estop-banner').style.display='none';
    document.getElementById('btn-RESET').style.display='none';
    setMode('manual');
    fetch('/reset').catch(()=>{});
  }

  // ── Keyboard ─────────────────────────────────────────────────
  document.addEventListener('keydown',e=>{
    if(e.key==='Enter')          {setMode('start');return;}
    if(e.key==='1')              {setMode('manual');return;}
    if(e.key==='2')              {setMode('auto');return;}
    if(e.key==='3')              {setMode('smart');return;}
    if(e.key==='i'||e.key==='I'){emergencyStop();return;}
    const cmd=keyMap[e.key];
    if(cmd&&active!==cmd){active=cmd;send(cmd);}
  });
  document.addEventListener('keyup',e=>{
    if(keyMap[e.key]){active=null;send('STOP');}
  });

  // ── Poll status ───────────────────────────────────────────────
  function updateUI(d) {
    // OLED mirror
    document.getElementById('s-action').textContent = d.action;
    document.getElementById('s-speed').textContent  = d.speed;
    document.getElementById('oled-mode').textContent = d.mode;
    document.getElementById('h-mode').textContent    = d.mode;

    const distTxt = d.dist>=999 ? '> 5m' : d.dist+' cm';
    document.getElementById('s-dist').textContent = distTxt;

    const stEl=document.getElementById('s-status');
    if(d.estop)              stEl.innerHTML='<span class="hinv">E-STOP</span>';
    else if(d.mode==='OBSTACLE!') stEl.innerHTML='<span class="hinv">OBSTACLE!</span>';
    else if(d.mode==='WARNING')  stEl.innerHTML='<span class="hinv">WARNING</span>';
    else                     stEl.textContent=d.mode;

    // Eyes
    const el=document.getElementById('eye-l');
    const er=document.getElementById('eye-r');
    el.className='eye'; er.className='eye';
    if(d.estop)                    {el.classList.add('dead');  er.classList.add('dead');}
    else if(d.obstacle||d.mode==='OBSTACLE!') {el.classList.add('angry'); er.classList.add('angry');}

    // ── Radar / Object Detection section ─────────────────────
    const dist = d.dist>=999 ? 500 : d.dist;
    document.getElementById('r-dist').textContent = d.dist>=999 ? '> 5m' : d.dist+' cm';

    // Bar: 0 cm = 100% full, 100 cm = 0%
    const pct = Math.max(0, Math.min(100, Math.round((1 - dist/100)*100)));
    document.getElementById('r-bar').style.width = pct+'%';

    const badge   = document.getElementById('radar-badge');
    const rStatus = document.getElementById('r-status');
    const dot     = document.getElementById('radar-dot');
    const zSafe   = document.getElementById('z-safe');
    const zWarn   = document.getElementById('z-warn');
    const zCrit   = document.getElementById('z-crit');

    // Reset zones
    zSafe.classList.remove('hot'); zWarn.classList.remove('hot'); zCrit.classList.remove('hot');

    if(d.obstacle) {
      badge.textContent='DETECTED'; badge.classList.add('detected');
      dot.style.display='block';
      if(dist < 15) { rStatus.textContent='!! CRITICAL !!'; zCrit.classList.add('hot'); }
      else          { rStatus.textContent='!! WARNING !!';  zWarn.classList.add('hot'); }
    } else {
      badge.textContent='CLEAR'; badge.classList.remove('detected');
      dot.style.display='none';
      rStatus.textContent='SCANNING'; zSafe.classList.add('hot');
    }

    // Smart mode dpad lock during bypass
    if(d.avoiding) {
      document.getElementById('smart-notice').style.display='block';
      dpadIds.forEach(id=>document.getElementById('btn-'+id).classList.add('disabled'));
    } else if(!stopped && (currentMode==='manual'||currentMode==='smart')) {
      document.getElementById('smart-notice').style.display='none';
      dpadIds.forEach(id=>document.getElementById('btn-'+id).classList.remove('disabled'));
    }
  }

  setInterval(()=>{fetch('/status').then(r=>r.json()).then(updateUI).catch(()=>{});}, 350);
</script>
</body>
</html>
)rawhtml";
}
