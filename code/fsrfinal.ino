#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ── WiFi ──────────────────────────────────────────────────────
const char* ssid     = "tushar";
const char* password = "tushar11";
ESP8266WebServer server(80);

// ── OLED ──────────────────────────────────────────────────────
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── Motor Pins ────────────────────────────────────────────────
const int pwmMotorA = 5;   // D1
const int pwmMotorB = 4;   // D2
const int dirMotorA = 0;   // D3
const int dirMotorB = 2;   // D4

// ── Ultrasonic Pins ───────────────────────────────────────────
const int TRIG_PIN = 13;   // D7
const int ECHO_PIN = 15;   // D8

// ── Config ────────────────────────────────────────────────────
int motorSpeed               = 170;
#define CRITICAL_DISTANCE     10   // cm — hard stop ANY mode
#define OBSTACLE_DISTANCE_AUTO 20   // cm — auto mode threshold
#define OBSTACLE_DISTANCE_SMART 30  // cm — smart mode threshold
#define TURN_TIME            255    // ms — ~90 degree turn
#define BYPASS_FWD_TIME      2000   // ms
#define REALIGN_FWD_TIME     2000   // ms

// ── System Mode ───────────────────────────────────────────────
enum SystemMode { MODE_WAIT, MODE_MANUAL, MODE_AUTO, MODE_SMART };
SystemMode systemMode = MODE_WAIT;

bool emergencyStopped = false;
bool smartAvoiding    = false;
bool criticalHalt     = false;

// ── Auto State Machine ────────────────────────────────────────
enum RobotState {
  STATE_FORWARD, STATE_STOP_AT_OBSTACLE,
  STATE_TURN_LEFT_1,  STATE_BYPASS_FWD_1,
  STATE_TURN_RIGHT_1, STATE_BYPASS_FWD_2,
  STATE_TURN_RIGHT_2, STATE_BYPASS_FWD_3,
  STATE_TURN_LEFT_2,  STATE_RESUME_FORWARD
};
RobotState robotState = STATE_FORWARD;

// ── Display State ─────────────────────────────────────────────
String        currentAction = "AWAITING ORDERS";
String        currentStatus = "STANDBY";
long          lastDist      = 0;
bool          obstacleNear  = false;
unsigned long stateTimer    = 0;
String        manualCmd     = "HALT";

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(pwmMotorA, OUTPUT); pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorA, OUTPUT); pinMode(dirMotorB, OUTPUT);
  pinMode(TRIG_PIN,  OUTPUT); pinMode(ECHO_PIN,  INPUT);

  Wire.begin(14, 12);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(2);
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
//  LOOP
// ═════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();

  // Emergency stop overrides everything
  if (emergencyStopped) {
    stopCar();
    updateDisplay();
    return;
  }

  lastDist     = getDistance();
  obstacleNear = (lastDist > 0 && lastDist < OBSTACLE_DISTANCE_SMART);

  // ── CRITICAL HARD STOP — fires in ANY mode < 10 cm ────────
  if (lastDist > 0 && lastDist < CRITICAL_DISTANCE) {
    criticalHalt  = true;
    smartAvoiding = false;
    stopCar();
    currentStatus = "CRITICAL!";
    currentAction = "OBJ < 10CM HALT";
    manualCmd     = "HALT";
    obstacleNear  = true;
    updateDisplay();
    return;
  } else {
    criticalHalt = false;
  }

  switch (systemMode) {

    // ── STANDBY ───────────────────────────────────────────────
    case MODE_WAIT:
      stopCar();
      currentStatus = "STANDBY";
      currentAction = "AWAITING ORDERS";
      manualCmd     = "HALT";
      updateDisplay();
      break;

    // ── MANUAL: pure driver control, soft warning only ────────
    case MODE_MANUAL:
      currentStatus = (lastDist > 0 && lastDist < OBSTACLE_DISTANCE_SMART)
                      ? "WARNING" : "MANUAL";
      updateDisplay();
      break;

    // ── AUTO: full autonomous obstacle bypass ─────────────────
    case MODE_AUTO:
      runAutoState(OBSTACLE_DISTANCE_AUTO);
      break;

    // ── SMART: manual drive + auto-bypass at 30 cm ───────────
    case MODE_SMART:
      runSmartMode();
      break;
  }
}

// ═════════════════════════════════════════════════════════════
//  AUTO STATE MACHINE
// ═════════════════════════════════════════════════════════════
void runAutoState(int threshold) {
  switch (robotState) {

    case STATE_FORWARD:
      currentStatus = (systemMode == MODE_SMART) ? "SMART" : "AUTO";
      currentAction = "ADVANCING";
      manualCmd     = "FWD";
      if (systemMode == MODE_AUTO) forward();
      updateDisplay();
      if (lastDist > 0 && lastDist < threshold) {
        stopCar();
        robotState = STATE_STOP_AT_OBSTACLE;
      }
      break;

    case STATE_STOP_AT_OBSTACLE:
      stopCar();
      currentStatus = "OBSTACLE!";
      currentAction = "TARGET DETECTED";
      manualCmd     = "HALT";
      updateDisplay();
      delay(700);
      currentAction = "STEP 1: EVADE L";
      stateTimer    = millis();
      robotState    = STATE_TURN_LEFT_1;
      break;

    case STATE_TURN_LEFT_1:
      currentAction = "STEP 1: EVADE L";
      turnLeft(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) {
        stopCar(); delay(400);
        currentAction = "STEP 2: ADVANCE";
        stateTimer = millis(); robotState = STATE_BYPASS_FWD_1;
      }
      break;

    case STATE_BYPASS_FWD_1:
      currentAction = "STEP 2: ADVANCE";
      forward(); updateDisplay();
      if (millis() - stateTimer >= BYPASS_FWD_TIME) {
        stopCar(); delay(400);
        currentAction = "STEP 3: ALIGN R";
        stateTimer = millis(); robotState = STATE_TURN_RIGHT_1;
      }
      break;

    case STATE_TURN_RIGHT_1:
      currentAction = "STEP 3: ALIGN R";
      turnRight(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) {
        stopCar(); delay(400);
        currentAction = "STEP 4: BYPASS";
        stateTimer = millis(); robotState = STATE_BYPASS_FWD_2;
      }
      break;

    case STATE_BYPASS_FWD_2:
      currentAction = "STEP 4: BYPASS";
      forward(); updateDisplay();
      if (millis() - stateTimer >= BYPASS_FWD_TIME) {
        stopCar(); delay(400);
        currentAction = "STEP 5: ALIGN R";
        stateTimer = millis(); robotState = STATE_TURN_RIGHT_2;
      }
      break;

    case STATE_TURN_RIGHT_2:
      currentAction = "STEP 5: ALIGN R";
      turnRight(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) {
        stopCar(); delay(400);
        currentAction = "STEP 6: RETURN";
        stateTimer = millis(); robotState = STATE_BYPASS_FWD_3;
      }
      break;

    case STATE_BYPASS_FWD_3:
      currentAction = "STEP 6: RETURN";
      forward(); updateDisplay();
      if (millis() - stateTimer >= REALIGN_FWD_TIME) {
        stopCar(); delay(400);
        currentAction = "STEP 7: REALIGN";
        stateTimer = millis(); robotState = STATE_TURN_LEFT_2;
      }
      break;

    case STATE_TURN_LEFT_2:
      currentAction = "STEP 7: REALIGN";
      turnLeft(); updateDisplay();
      if (millis() - stateTimer >= TURN_TIME) {
        stopCar(); delay(400);
        currentAction   = "BYPASS COMPLETE";
        currentStatus   = (systemMode == MODE_SMART) ? "SMART" : "AUTO";
        smartAvoiding   = false;
        robotState      = STATE_RESUME_FORWARD;
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

// ═════════════════════════════════════════════════════════════
//  SMART MODE
// ═════════════════════════════════════════════════════════════
void runSmartMode() {
  if (smartAvoiding) {
    runAutoState(OBSTACLE_DISTANCE_SMART);
    return;
  }
  currentStatus = "SMART";
  if (lastDist > 0 && lastDist < OBSTACLE_DISTANCE_SMART) {
    smartAvoiding = true;
    robotState    = STATE_STOP_AT_OBSTACLE;
    currentAction = "AUTO-EVADE!";
    manualCmd     = "EVADE";
    runAutoState(OBSTACLE_DISTANCE_SMART);
  } else {
    currentAction = "MANUAL CTRL";
    updateDisplay();
  }
}

// ═════════════════════════════════════════════════════════════
//  WEB ROUTES
// ═════════════════════════════════════════════════════════════
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
    if (server.hasArg("action") && !emergencyStopped && !criticalHalt) {
      if (systemMode == MODE_MANUAL || (systemMode == MODE_SMART && !smartAvoiding)) {
        String a = server.arg("action"); a.toUpperCase(); manualCmd = a;
        if      (a == "FORWARD")  { currentAction = "ADVANCING";  forward();  }
        else if (a == "BACKWARD") { currentAction = "RETREATING"; backward(); }
        else if (a == "LEFT")     { currentAction = "EVADING L";  turnLeft(); }
        else if (a == "RIGHT")    { currentAction = "EVADING R";  turnRight();}
        else                      { currentAction = "HALTED";     stopCar();  manualCmd = "HALT"; }
        server.send(200, "text/plain", "OK");
      } else { server.send(200, "text/plain", "BLOCKED"); }
    } else { server.send(200, "text/plain", "BLOCKED"); }
  });

  server.on("/estop", []() {
    emergencyStopped = true; smartAvoiding = false;
    currentStatus = "E-STOP"; currentAction = "EMERGENCY HALT"; manualCmd = "HALT";
    stopCar(); server.send(200, "text/plain", "ESTOP");
  });

  server.on("/reset", []() {
    emergencyStopped = false; smartAvoiding = false; criticalHalt = false;
    systemMode = MODE_MANUAL; currentStatus = "MANUAL";
    currentAction = "SYSTEMS ONLINE"; manualCmd = "HALT";
    robotState = STATE_FORWARD; stopCar();
    server.send(200, "text/plain", "RESET");
  });

  server.on("/status", []() {
    String json = "{";
    json += "\"mode\":\""    + currentStatus + "\",";
    json += "\"action\":\""  + currentAction + "\",";
    json += "\"dist\":"      + String(lastDist) + ",";
    json += "\"speed\":"     + String(motorSpeed) + ",";
    json += "\"cmd\":\""     + manualCmd + "\",";
    json += "\"obstacle\":"  + String(obstacleNear  ? "true" : "false") + ",";
    json += "\"avoiding\":"  + String(smartAvoiding ? "true" : "false") + ",";
    json += "\"critical\":"  + String(criticalHalt  ? "true" : "false") + ",";
    json += "\"estop\":"     + String(emergencyStopped ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });
}

// ═════════════════════════════════════════════════════════════
//  OLED DISPLAY
// ═════════════════════════════════════════════════════════════
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Row 1 — Status
  display.setCursor(0, 0);
  display.print("STATUS: ");
  bool inverted = (currentStatus == "OBSTACLE!" || currentStatus == "E-STOP" ||
                   currentStatus == "WARNING"   || currentStatus == "CRITICAL!");
  if (inverted) {
    display.fillRect(48, 0, 80, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(50, 0);
    display.print(currentStatus);
    display.setTextColor(WHITE);
  } else {
    display.print(currentStatus);
  }

  display.drawFastHLine(0, 10, 128, WHITE);

  // Row 2 — Action
  display.setCursor(0, 13);
  display.print("ACT : ");
  display.print(currentAction);

  // Row 3 — Distance + indicator square
  display.setCursor(0, 23);
  display.print("DIST: ");
  if (lastDist >= 999) display.print("> 5m");
  else { display.print(lastDist); display.print(" cm"); }
  if (obstacleNear) display.fillRect(110, 23, 8, 8, WHITE);
  else              display.drawRect(110, 23, 8, 8, WHITE);

  // Row 4 — Speed
  display.setCursor(0, 33);
  display.print("SPD : ");
  display.print(motorSpeed);
  display.print(" / 1023");

  display.drawFastHLine(0, 43, 128, WHITE);

  drawEyesInRegion();
  display.display();
}

void drawEyesInRegion() {
  int ew = 35, eh = 14, ey = 47;
  if (emergencyStopped || criticalHalt) {
    // X eyes
    display.drawLine(8,  ey, 8+ew,  ey+eh, WHITE);
    display.drawLine(8+ew,  ey, 8,  ey+eh, WHITE);
    display.drawLine(85, ey, 85+ew, ey+eh, WHITE);
    display.drawLine(85+ew, ey, 85, ey+eh, WHITE);
  } else if (currentStatus == "OBSTACLE!" || currentStatus == "WARNING" || currentStatus == "CRITICAL!") {
    // Angry eyes
    display.fillRoundRect(8,  ey, ew, eh, 5, WHITE);
    display.fillRoundRect(85, ey, ew, eh, 5, WHITE);
    display.fillTriangle(8,    ey, 8+ew/2,    ey, 8,    ey+6, BLACK);
    display.fillTriangle(85+ew/2, ey, 85+ew, ey, 85+ew, ey+6, BLACK);
  } else {
    // Normal eyes
    display.fillRoundRect(8,  ey, ew, eh, 5, WHITE);
    display.fillRoundRect(85, ey, ew, eh, 5, WHITE);
  }
}

// ═════════════════════════════════════════════════════════════
//  WIFI
// ═════════════════════════════════════════════════════════════
void connectWiFi() {
  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(10, 25); display.print("Connecting WiFi..."); display.display();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  display.clearDisplay();
  display.setCursor(0, 10); display.print("Connected!");
  display.setCursor(0, 28); display.print("Open browser:");
  display.setCursor(0, 44); display.print(WiFi.localIP());
  display.display(); delay(3000);
}

// ═════════════════════════════════════════════════════════════
//  ULTRASONIC SENSOR
// ═════════════════════════════════════════════════════════════
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 999;
  return dur * 0.034 / 2;
}

// ═════════════════════════════════════════════════════════════
//  MOTORS
// ═════════════════════════════════════════════════════════════
void forward()  { digitalWrite(dirMotorA,LOW);  digitalWrite(dirMotorB,LOW);  analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void backward() { digitalWrite(dirMotorA,HIGH); digitalWrite(dirMotorB,HIGH); analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void turnLeft() { digitalWrite(dirMotorA,HIGH); digitalWrite(dirMotorB,LOW);  analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void turnRight(){ digitalWrite(dirMotorA,LOW);  digitalWrite(dirMotorB,HIGH); analogWrite(pwmMotorA,motorSpeed); analogWrite(pwmMotorB,motorSpeed); }
void stopCar()  { analogWrite(pwmMotorA,0); analogWrite(pwmMotorB,0); }

// ═════════════════════════════════════════════════════════════
//  STARTUP
// ═════════════════════════════════════════════════════════════
void startupSequence() {
  display.clearDisplay(); display.setTextSize(2); display.setTextColor(WHITE);
  display.setCursor(20, 20); display.print("WELCOME"); display.display(); delay(2000);
  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(10, 25); display.print("NEXUS SR");
  display.setCursor(15, 40); display.print("Starting..."); display.display(); delay(1500);
  loadingAnimation();
}

void loadingAnimation() {
  for (int i = 0; i <= 100; i += 10) {
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(40, 10); display.print("Loading");
    display.drawRect(14, 30, 100, 10, WHITE);
    display.fillRect(14, 30, i, 10, WHITE);
    display.display(); delay(200);
  }
}

// ═════════════════════════════════════════════════════════════
//  HTML PAGE
// ═════════════════════════════════════════════════════════════
String getControlPage() {
  return R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UNIT-1 SERVICE BOT CONTROL</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Oswald:wght@400;600&display=swap');
  *{box-sizing:border-box;margin:0;padding:0;}
  body{
    background:#000;color:#fff;
    font-family:'Share Tech Mono',monospace;
    min-height:100vh;
    display:grid;
    grid-template-columns:1fr 1fr;
    align-items:start;
    padding:18px 24px 28px;
    gap:13px;
    max-width:1100px;
    margin:0 auto;
  }
  body::before{
    content:'';position:fixed;inset:0;pointer-events:none;z-index:999;
    background:repeating-linear-gradient(0deg,rgba(255,255,255,0.016) 0px,rgba(255,255,255,0.016) 1px,transparent 1px,transparent 3px);
  }
  .full{grid-column:1 / -1;}
  .col{display:flex;flex-direction:column;gap:13px;}

  /* HEADER */
  .header{border:1px solid #fff;padding:8px 14px;display:flex;align-items:center;justify-content:space-between;}
  .header-title{font-family:'Oswald',sans-serif;font-size:1.05rem;font-weight:600;letter-spacing:5px;}
  .header-sub{font-size:0.6rem;color:#aaa;letter-spacing:2px;margin-top:2px;}
  .blink{animation:blink 1s step-end infinite;}
  @keyframes blink{50%{opacity:0;}}

  /* SECTION LABEL */
  .slbl{font-size:0.58rem;letter-spacing:3px;color:#555;border-bottom:1px solid #1a1a1a;padding-bottom:3px;display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;}

  /* OLED */
  .oled-frame{border:1px solid #fff;padding:1px;}
  .oled-tb{background:#fff;color:#000;font-family:'Oswald',sans-serif;font-size:0.62rem;letter-spacing:3px;padding:3px 10px;display:flex;justify-content:space-between;}
  .oled-in{background:#000;border:1px solid #333;padding:10px 12px;font-size:0.7rem;line-height:1.9;}
  .oled-r{display:flex;justify-content:space-between;align-items:center;}
  .oled-l{color:#666;min-width:52px;}
  .oled-v{color:#fff;}
  .odiv{border-top:1px solid #fff;margin:5px 0;}
  .eyes{display:flex;justify-content:space-between;padding:5px 8px 0;gap:8px;height:22px;}
  .eye{background:#fff;border-radius:3px;flex:1;transition:all 0.2s;}
  .eye.angry{clip-path:polygon(0 45%,50% 0%,100% 0%,100% 100%,0 100%);}
  .eye.dead{background:transparent;border:1px solid #fff;position:relative;}
  .eye.dead::before,.eye.dead::after{content:'';position:absolute;top:50%;left:50%;width:100%;height:1px;background:#fff;}
  .eye.dead::before{transform:translate(-50%,-50%) rotate(35deg);}
  .eye.dead::after{transform:translate(-50%,-50%) rotate(-35deg);}
  .hinv{background:#fff;color:#000;padding:0 5px;font-weight:bold;}

  /* ULTRASONIC SENSOR PANEL */
  .radar-frame{border:1px solid #fff;padding:1px;}
  .radar-tb{background:#fff;color:#000;font-family:'Oswald',sans-serif;font-size:0.62rem;letter-spacing:3px;padding:3px 10px;display:flex;justify-content:space-between;align-items:center;}
  .radar-body{background:#000;border:1px solid #333;padding:12px 14px;display:flex;align-items:center;gap:16px;}
  .radar-arc{position:relative;width:84px;height:46px;overflow:hidden;flex-shrink:0;}
  .rarc{position:absolute;bottom:0;left:50%;transform:translateX(-50%);border-radius:50%;clip-path:polygon(0 50%,100% 50%,100% 0%,0 0%);}
  .rarc1{width:84px;height:84px;border:1px solid #2a2a2a;}
  .rarc2{width:56px;height:56px;border:1px solid #333;}
  .rarc3{width:28px;height:28px;border:1px solid #3a3a3a;}
  .rsweep{position:absolute;bottom:0;left:50%;width:1px;height:42px;background:#fff;transform-origin:bottom center;}
  .rdot{position:absolute;bottom:8px;left:50%;width:7px;height:7px;border-radius:50%;background:#fff;transform:translateX(-50%);display:none;}
  .radar-stats{flex:1;display:flex;flex-direction:column;gap:5px;font-size:0.68rem;}
  .rstat{display:flex;justify-content:space-between;align-items:center;}
  .rstat-l{color:#666;}
  .rstat-v{color:#fff;}
  .dbar-wrap{width:100%;height:6px;background:#0a0a0a;border:1px solid #2a2a2a;margin-top:2px;}
  .dbar{height:100%;background:#fff;transition:width 0.35s;}
  .obj-badge{font-size:0.58rem;letter-spacing:2px;border:1px solid #555;color:#555;padding:1px 7px;}
  .obj-badge.det{border-color:#fff;background:#fff;color:#000;animation:flash 0.5s step-end infinite;}
  .obj-badge.crit{border-color:#fff;background:#fff;color:#000;animation:flash 0.25s step-end infinite;}
  @keyframes flash{50%{opacity:0.3;}}
  .zones{display:flex;gap:5px;margin-top:4px;flex-wrap:wrap;}
  .zone{font-size:0.55rem;padding:2px 5px;border:1px solid #222;color:#444;letter-spacing:1px;}
  .zone.hot{border-color:#fff;background:#fff;color:#000;}

  /* CRITICAL BANNER */
  #critical-banner{
    display:none;
    width:100%;border:2px solid #fff;padding:9px;
    text-align:center;font-family:'Oswald',sans-serif;
    font-size:0.9rem;letter-spacing:5px;font-weight:600;
    animation:flash 0.3s step-end infinite;
  }

  /* MODE BUTTONS */
  .mrow{display:flex;gap:7px;}
  .mbtn{
    flex:1;padding:11px 4px;
    background:#000;border:1px solid #3a3a3a;
    color:#555;font-family:'Share Tech Mono',monospace;
    font-size:0.63rem;letter-spacing:1px;
    cursor:pointer;text-transform:uppercase;
    transition:all 0.15s;line-height:1.7;
  }
  .mbtn:hover{border-color:#fff;color:#fff;}
  .mbtn.act{background:#fff;color:#000;border-color:#fff;}

  /* D-PAD */
  .dpad{border:1px solid #1a1a1a;padding:18px;display:flex;flex-direction:column;align-items:center;gap:7px;}
  .drow{display:flex;gap:7px;align-items:center;}
  .db{
    width:78px;height:78px;
    background:#000;border:1px solid #3a3a3a;
    color:#fff;font-size:1.5rem;
    display:flex;align-items:center;justify-content:center;
    cursor:pointer;transition:all 0.1s;user-select:none;
  }
  .db:hover{border-color:#fff;}
  .db:active,.db.active{background:#fff;color:#000;border-color:#fff;}
  .db.disabled{opacity:0.15;pointer-events:none;}
  .dc{
    width:78px;height:78px;
    background:#080808;border:1px solid #1a1a1a;
    display:flex;align-items:center;justify-content:center;
    font-size:0.55rem;color:#333;letter-spacing:1px;
  }

  /* SMART NOTICE */
  #smart-notice{
    display:none;width:100%;text-align:center;
    font-size:0.6rem;letter-spacing:2px;color:#777;
    border:1px solid #333;padding:6px;
    animation:blink 0.8s step-end infinite;
    margin-top:6px;
  }

  /* ESTOP */
  .ez{display:flex;flex-direction:column;align-items:center;gap:8px;}
  #btn-ESTOP{
    width:100%;padding:16px;
    background:#000;border:2px solid #fff;
    color:#fff;font-family:'Oswald',sans-serif;
    font-size:1rem;font-weight:600;letter-spacing:5px;
    cursor:pointer;display:flex;align-items:center;
    justify-content:center;gap:14px;transition:all 0.1s;
  }
  #btn-ESTOP:hover{background:#fff;color:#000;}
  #btn-ESTOP:active{transform:scale(0.98);}
  .ering{width:13px;height:13px;border:2px solid #fff;border-radius:50%;animation:pr 1.2s ease-in-out infinite;}
  #btn-ESTOP:hover .ering{border-color:#000;}
  @keyframes pr{0%,100%{transform:scale(1);}50%{transform:scale(1.35);opacity:0.4;}}
  #estop-banner{
    display:none;width:100%;
    border:1px solid #fff;padding:9px;
    text-align:center;font-size:0.7rem;letter-spacing:4px;
    animation:blink 0.6s step-end infinite;
  }
  #btn-RESET{
    display:none;padding:11px 36px;
    background:#000;border:1px solid #fff;
    color:#fff;font-family:'Share Tech Mono',monospace;
    font-size:0.72rem;letter-spacing:2px;cursor:pointer;transition:all 0.15s;
  }
  #btn-RESET:hover{background:#fff;color:#000;}

  /* FOOTER */
  .foot{
    border-top:1px solid #111;padding-top:8px;
    font-size:0.55rem;color:#2a2a2a;letter-spacing:1px;
    display:flex;justify-content:space-between;flex-wrap:wrap;gap:3px;
  }
</style>
</head>
<body>

<!-- ══ HEADER ══ -->
<div class="header full">
  <div>
    <div class="header-title">UNIT-1 // SERVICE BOT</div>
    <div class="header-sub">REMOTE TACTICAL CONTROL SYSTEM</div>
  </div>
  <div style="font-size:0.62rem;color:#666;text-align:right;line-height:1.8;">
    <span class="blink">&#9632;</span> ONLINE<br>
    <span style="color:#333;font-size:0.54rem;" id="h-mode">STANDBY</span>
  </div>
</div>

<!-- ══ CRITICAL BANNER ══ -->
<div id="critical-banner" class="full">!! CRITICAL — OBJECT UNDER 10 CM — HARD STOP !!</div>

<!-- ══ LEFT COLUMN ══ -->
<div class="col">

  <!-- OLED MIRROR -->
  <div>
    <div class="slbl"><span>// ONBOARD DISPLAY</span><span id="oled-mode">STANDBY</span></div>
    <div class="oled-frame">
      <div class="oled-tb"><span>// OLED MIRROR</span><span id="oled-mode2">STANDBY</span></div>
      <div class="oled-in">
        <div class="oled-r"><span class="oled-l">STATUS</span><span class="oled-v" id="s-status">STANDBY</span></div>
        <div class="odiv"></div>
        <div class="oled-r"><span class="oled-l">ACT</span><span class="oled-v" id="s-action">AWAITING ORDERS</span></div>
        <div class="oled-r"><span class="oled-l">DIST</span><span class="oled-v" id="s-dist">---</span></div>
        <div class="oled-r"><span class="oled-l">SPD</span><span class="oled-v"><span id="s-speed">150</span> / 1023</span></div>
        <div class="odiv"></div>
        <div class="eyes"><div class="eye" id="eye-l"></div><div class="eye" id="eye-r"></div></div>
      </div>
    </div>
  </div>

  <!-- ULTRASONIC SENSOR -->
  <div>
    <div class="slbl"><span>// OBJECT DETECTION</span><span>ULTRASONIC SENSOR FEED</span></div>
    <div class="radar-frame">
      <div class="radar-tb">
        <span>// ULTRASONIC SENSOR</span>
        <span id="radar-badge" class="obj-badge">CLEAR</span>
      </div>
      <div class="radar-body">
        <div class="radar-arc">
          <div class="rarc rarc1"></div>
          <div class="rarc rarc2"></div>
          <div class="rarc rarc3"></div>
          <div class="rsweep" id="rsweep"></div>
          <div class="rdot" id="rdot"></div>
        </div>
        <div class="radar-stats">
          <div class="rstat"><span class="rstat-l">RANGE</span><span class="rstat-v" id="r-dist">--- cm</span></div>
          <div class="dbar-wrap"><div class="dbar" id="r-bar" style="width:0%"></div></div>
          <div class="rstat" style="margin-top:3px;"><span class="rstat-l">STATUS</span><span class="rstat-v" id="r-status">SCANNING</span></div>
          <div class="rstat"><span class="rstat-l">HARD STOP</span><span class="rstat-v">&lt; 10 cm</span></div>
          <div class="zones">
            <div class="zone" id="z-safe">SAFE &gt;30</div>
            <div class="zone" id="z-warn">WARN 10-30</div>
            <div class="zone" id="z-crit">CRIT &lt;10</div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <!-- MODE SELECT -->
  <div>
    <div class="slbl"><span>// MODE SELECT</span><span>ENTER · 1 · 2 · 3</span></div>
    <div class="mrow">
      <button class="mbtn" id="btn-start"  onclick="setMode('start')">[ ENTER ]<br>DEPLOY</button>
      <button class="mbtn" id="btn-manual" onclick="setMode('manual')">[  1  ]<br>MANUAL</button>
      <button class="mbtn" id="btn-auto"   onclick="setMode('auto')">[  2  ]<br>AUTO</button>
      <button class="mbtn" id="btn-smart"  onclick="setMode('smart')">[  3  ]<br>SMART</button>
    </div>
  </div>

</div><!-- end left col -->

<!-- ══ RIGHT COLUMN ══ -->
<div class="col">

  <!-- D-PAD -->
  <div>
    <div class="slbl"><span>// DIRECTIONAL CONTROL</span><span>W A S D</span></div>
    <div class="dpad">
      <div class="drow">
        <button class="db disabled" id="btn-FORWARD"
          ontouchstart="send('FORWARD')" ontouchend="send('STOP')">&#9650;</button>
      </div>
      <div class="drow">
        <button class="db disabled" id="btn-LEFT"
          ontouchstart="send('LEFT')" ontouchend="send('STOP')">&#9668;</button>
        <div class="dc">HALT</div>
        <button class="db disabled" id="btn-RIGHT"
          ontouchstart="send('RIGHT')" ontouchend="send('STOP')">&#9658;</button>
      </div>
      <div class="drow">
        <button class="db disabled" id="btn-BACKWARD"
          ontouchstart="send('BACKWARD')" ontouchend="send('STOP')">&#9660;</button>
      </div>
    </div>
    <div id="smart-notice">!! AUTO-EVADE ACTIVE — MANUAL OVERRIDE LOCKED !!</div>
  </div>

  <!-- EMERGENCY STOP -->
  <div>
    <div class="slbl"><span>// EMERGENCY OVERRIDE</span><span>KEY: I</span></div>
    <div class="ez">
      <div id="estop-banner">!! EMERGENCY STOP ENGAGED !!</div>
      <button id="btn-ESTOP" onclick="emergencyStop()">
        <div class="ering"></div>
        EMERGENCY STOP
        <div class="ering"></div>
      </button>
      <button id="btn-RESET" onclick="resetStop()">[ RESET &amp; RESUME OPERATIONS ]</button>
    </div>
  </div>

</div><!-- end right col -->

<!-- ══ FOOTER ══ -->
<div class="foot full">
  <span>ENTER=DEPLOY</span>
  <span>1=MANUAL · 2=AUTO · 3=SMART</span>
  <span>WASD=DRIVE</span>
  <span>I=E-STOP</span>
  <span>AUTO HARD STOP &lt; 10CM</span>
</div>

<script>
const keyMap={
  'w':'FORWARD','ArrowUp':'FORWARD',
  's':'BACKWARD','ArrowDown':'BACKWARD',
  'a':'LEFT','ArrowLeft':'LEFT',
  'd':'RIGHT','ArrowRight':'RIGHT'
};
let active=null, stopped=false, currentMode='wait';
const dpadIds=['FORWARD','BACKWARD','LEFT','RIGHT'];

// Radar sweep
let sweepAngle=-90;
setInterval(()=>{
  sweepAngle=(sweepAngle+4)%180-90;
  document.getElementById('rsweep').style.transform='rotate('+sweepAngle+'deg)';
},40);

function setMode(m){
  fetch('/mode?m='+m).catch(()=>{});
  currentMode=(m==='start')?'manual':m;
  ['btn-start','btn-manual','btn-auto','btn-smart'].forEach(id=>
    document.getElementById(id).classList.remove('act'));
  if(m==='start'||m==='manual') document.getElementById('btn-manual').classList.add('act');
  if(m==='auto')  document.getElementById('btn-auto').classList.add('act');
  if(m==='smart') document.getElementById('btn-smart').classList.add('act');
  document.getElementById('h-mode').textContent=currentMode.toUpperCase();
  enableDpad(currentMode==='manual'||currentMode==='smart');
}

function enableDpad(on){
  dpadIds.forEach(id=>{
    const b=document.getElementById('btn-'+id);
    if(on&&!stopped) b.classList.remove('disabled');
    else b.classList.add('disabled');
  });
}

function send(action){
  if(stopped) return;
  if(currentMode!=='manual'&&currentMode!=='smart') return;
  document.querySelectorAll('.db').forEach(b=>b.classList.remove('active'));
  const b=document.getElementById('btn-'+action);
  if(b) b.classList.add('active');
  fetch('/cmd?action='+action).catch(()=>{});
}

function emergencyStop(){
  stopped=true; active=null;
  document.getElementById('estop-banner').style.display='block';
  document.getElementById('btn-RESET').style.display='block';
  document.getElementById('smart-notice').style.display='none';
  dpadIds.forEach(id=>document.getElementById('btn-'+id).classList.add('disabled'));
  fetch('/estop').catch(()=>{});
}

function resetStop(){
  stopped=false;
  document.getElementById('estop-banner').style.display='none';
  document.getElementById('btn-RESET').style.display='none';
  document.getElementById('critical-banner').style.display='none';
  setMode('manual');
  fetch('/reset').catch(()=>{});
}

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

function updateUI(d){
  // OLED mirror
  document.getElementById('s-action').textContent = d.action;
  document.getElementById('s-speed').textContent  = d.speed;
  document.getElementById('oled-mode').textContent  = d.mode;
  document.getElementById('oled-mode2').textContent = d.mode;
  document.getElementById('h-mode').textContent     = d.mode;
  document.getElementById('s-dist').textContent = d.dist>=999?'> 5m':d.dist+' cm';

  // Status field
  const stEl=document.getElementById('s-status');
  if(d.critical)              stEl.innerHTML='<span class="hinv">CRITICAL!</span>';
  else if(d.estop)            stEl.innerHTML='<span class="hinv">E-STOP</span>';
  else if(d.mode==='OBSTACLE!')stEl.innerHTML='<span class="hinv">OBSTACLE!</span>';
  else if(d.mode==='WARNING') stEl.innerHTML='<span class="hinv">WARNING</span>';
  else                        stEl.textContent=d.mode;

  // Eyes
  const el=document.getElementById('eye-l');
  const er=document.getElementById('eye-r');
  el.className='eye'; er.className='eye';
  if(d.estop||d.critical)              {el.classList.add('dead'); er.classList.add('dead');}
  else if(d.obstacle||d.mode==='OBSTACLE!'){el.classList.add('angry');er.classList.add('angry');}

  // Critical banner
  document.getElementById('critical-banner').style.display = d.critical?'block':'none';

  // Ultrasonic sensor panel
  const dist=d.dist>=999?500:d.dist;
  document.getElementById('r-dist').textContent=d.dist>=999?'> 5m':d.dist+' cm';
  const pct=Math.max(0,Math.min(100,Math.round((1-dist/100)*100)));
  document.getElementById('r-bar').style.width=pct+'%';

  const badge=document.getElementById('radar-badge');
  const rSt=document.getElementById('r-status');
  const dot=document.getElementById('rdot');
  const zS=document.getElementById('z-safe');
  const zW=document.getElementById('z-warn');
  const zC=document.getElementById('z-crit');
  zS.classList.remove('hot'); zW.classList.remove('hot'); zC.classList.remove('hot');

  if(d.critical){
    badge.textContent='!! CRITICAL !!'; badge.className='obj-badge crit';
    dot.style.display='block';
    rSt.textContent='HARD STOP'; zC.classList.add('hot');
  } else if(d.obstacle){
    badge.textContent='DETECTED'; badge.className='obj-badge det';
    dot.style.display='block';
    rSt.textContent='!! WARNING !!'; zW.classList.add('hot');
  } else {
    badge.textContent='CLEAR'; badge.className='obj-badge';
    dot.style.display='none';
    rSt.textContent='SCANNING'; zS.classList.add('hot');
  }

  // Smart mode dpad lock
  if(d.avoiding){
    document.getElementById('smart-notice').style.display='block';
    dpadIds.forEach(id=>document.getElementById('btn-'+id).classList.add('disabled'));
  } else if(!stopped&&(currentMode==='manual'||currentMode==='smart')){
    document.getElementById('smart-notice').style.display='none';
    dpadIds.forEach(id=>document.getElementById('btn-'+id).classList.remove('disabled'));
  }
}

setInterval(()=>{
  fetch('/status').then(r=>r.json()).then(updateUI).catch(()=>{});
},350);
</script>
</body>
</html>
)rawhtml";
}