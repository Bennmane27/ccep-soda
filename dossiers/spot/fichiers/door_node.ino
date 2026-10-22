/*
 * door_node.ino — Module de porte pour Spot (mode AUTOMATIQUE uniquement)
 * Carte cible : Seeit ESP32-RELAY02 (ESP32-WROOM-32E), relais K1 = GPIO16, K2 = GPIO17.
 * Core Arduino : "esp32 by Espressif" 3.x, carte "ESP32 Dev Module".
 *
 * Rôle : recevoir une demande authentifiée (HTTP sur Wi-Fi) et fermer un contact sec
 * pendant UNE impulsion (500 ms par défaut) :
 *   - K1 (GPIO16), en parallèle du bouton vert "Ouverture" : la porte lance son cycle ;
 *   - K2 (GPIO17), en parallèle du bouton de FERMETURE, seulement pour les portes qui ne se
 *     referment pas seules (réglage "set close 1", désactivé par défaut).
 * L'automatisme de la porte gère le mouvement et ses sécurités (cellule, fins de course).
 * Le module ne maintient jamais un contact et ne ferme jamais K1 et K2 en même temps.
 *
 * Ce firmware ne mesure pas la position de la porte. L'ouverture est confirmée côté robot
 * (caméras de Spot) ou, en option, par un contact "porte ouverte" câblé sur INPUT_PIN
 * via une interface isolée (jamais de 24 V directement sur un GPIO).
 *
 * API (port 80, JSON) :
 *   GET  /api/v1/health   sans jeton  -> {"ok":true,"fw":"..."}
 *   GET  /api/v1/status   avec jeton  -> état détaillé
 *   POST /api/v1/open     avec jeton  -> une impulsion (idempotent par op_id, anti-rebond)
 *   POST /api/v1/close    avec jeton  -> une impulsion K2 si "close" est activé, sinon 409
 * Jeton : en-tête "Authorization: Bearer <jeton>".
 *
 * Configuration : par le port série (115200 bauds), stockée en NVS. Taper "help".
 * Aucun identifiant ni mot de passe n'est écrit dans ce fichier.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

#define FW_VERSION "door_node 2.1.0-auto"

// ---------------- Matériel ----------------
static const int RELAY_OPEN_PIN = 16;   // K1 : contact sec en parallèle du bouton vert
static const int RELAY_CLOSE_PIN = 17;  // K2 : contact sec en parallèle du bouton de fermeture
static const bool RELAY_ACTIVE_HIGH = true;

// Bornes de sécurité codées en dur (non modifiables par configuration)
static const uint32_t PULSE_MIN_MS = 100;
static const uint32_t PULSE_MAX_MS = 2000;
static const uint32_t COOLDOWN_MIN_MS = 1000;
static const uint32_t OPID_MEMORY_MS = 120000;  // durée pendant laquelle un op_id est reconnu
static const uint32_t WDT_TIMEOUT_S = 8;
static const uint32_t INTERLOCK_MS = 2000;      // délai mini entre une ouverture et une fermeture

// ---------------- Configuration (NVS) ----------------
struct Config {
  String doorId;       // ex. "P01"
  String ssid;
  String wifiPass;
  String token;        // jeton partagé robot -> porte (16 caractères minimum)
  uint32_t pulseMs;    // durée de l'impulsion
  uint32_t cooldownMs; // délai minimal entre deux impulsions
  int ledLivePin;      // -1 = absent
  int ledWifiPin;
  int ledPulsePin;
  int inputPin;        // contact "porte ouverte" optionnel, -1 = absent
  bool inputActiveLow;
  bool closeEnabled;   // porte à fermeture commandée (K2 autorisé)
};

Preferences prefs;
Config cfg;
WebServer server(80);

// ---------------- État ----------------
enum PulseKind { PULSE_OPEN, PULSE_CLOSE };
volatile bool pulseActive = false;
int pulsePin = RELAY_OPEN_PIN;   // relais en cours d'impulsion
uint32_t pulseStartMs = 0;
uint32_t lastPulseMs = 0;        // dernière ouverture (K1)
bool everPulsed = false;
uint32_t pulseCount = 0;
uint32_t lastCloseMs = 0;        // dernière fermeture (K2)
bool everClosed = false;
uint32_t closeCount = 0;
uint32_t rejectedAuthCount = 0;
String lastOpId = "";
uint32_t lastOpIdMs = 0;
String lastOpResult = "";
String serialLine = "";
uint32_t lastWifiAttemptMs = 0;

// ---------------- Relais ----------------
static inline void relayWrite(int pin, bool on) {
  digitalWrite(pin, (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

static void relaysSafe() {
  relayWrite(RELAY_OPEN_PIN, false);
  relayWrite(RELAY_CLOSE_PIN, false);
  pulseActive = false;
}

static void ledWrite(int pin, bool on) {
  if (pin >= 0) digitalWrite(pin, on ? HIGH : LOW);
}

// Démarre une impulsion K1 (ouverture) ou K2 (fermeture). Retourne false si refusée.
static bool startPulse(PulseKind kind, String &reason) {
  uint32_t now = millis();
  if (pulseActive) { reason = "pulse_in_progress"; return false; }  // jamais K1 et K2 ensemble
  if (kind == PULSE_OPEN) {
    if (everPulsed && (now - lastPulseMs) < cfg.cooldownMs) { reason = "cooldown"; return false; }
    if (everClosed && (now - lastCloseMs) < INTERLOCK_MS) { reason = "too_soon_after_close"; return false; }
  } else {
    if (!cfg.closeEnabled) { reason = "close_disabled"; return false; }
    if (everClosed && (now - lastCloseMs) < cfg.cooldownMs) { reason = "cooldown"; return false; }
    if (everPulsed && (now - lastPulseMs) < INTERLOCK_MS) { reason = "too_soon_after_open"; return false; }
  }
  pulseStartMs = now;
  pulseActive = true;
  if (kind == PULSE_OPEN) {
    lastPulseMs = now; everPulsed = true; pulseCount++; pulsePin = RELAY_OPEN_PIN;
  } else {
    lastCloseMs = now; everClosed = true; closeCount++; pulsePin = RELAY_CLOSE_PIN;
  }
  relayWrite(pulsePin, true);
  ledWrite(cfg.ledPulsePin, true);
  reason = "pulsed";
  return true;
}

// Appelée à chaque tour de loop() : termine l'impulsion quoi qu'il arrive.
static void servicePulse() {
  if (!pulseActive) return;
  uint32_t elapsed = millis() - pulseStartMs;
  uint32_t limit = cfg.pulseMs;
  if (limit > PULSE_MAX_MS) limit = PULSE_MAX_MS;
  if (elapsed >= limit) {
    relayWrite(pulsePin, false);
    ledWrite(cfg.ledPulsePin, false);
    pulseActive = false;
  }
}

// ---------------- Configuration ----------------
static void loadConfig() {
  prefs.begin("door", true);
  cfg.doorId = prefs.getString("door_id", "");
  cfg.ssid = prefs.getString("ssid", "");
  cfg.wifiPass = prefs.getString("wpass", "");
  cfg.token = prefs.getString("token", "");
  cfg.pulseMs = prefs.getUInt("pulse_ms", 500);
  cfg.cooldownMs = prefs.getUInt("cool_ms", 3000);
  cfg.ledLivePin = prefs.getInt("led_live", -1);
  cfg.ledWifiPin = prefs.getInt("led_wifi", -1);
  cfg.ledPulsePin = prefs.getInt("led_pulse", -1);
  cfg.inputPin = prefs.getInt("in_pin", -1);
  cfg.inputActiveLow = prefs.getBool("in_low", true);
  cfg.closeEnabled = prefs.getBool("close", false);
  prefs.end();
  if (cfg.pulseMs < PULSE_MIN_MS) cfg.pulseMs = PULSE_MIN_MS;
  if (cfg.pulseMs > PULSE_MAX_MS) cfg.pulseMs = PULSE_MAX_MS;
  if (cfg.cooldownMs < COOLDOWN_MIN_MS) cfg.cooldownMs = COOLDOWN_MIN_MS;
}

static bool configValid() {
  return cfg.doorId.length() > 0 && cfg.ssid.length() > 0 && cfg.token.length() >= 16;
}

static String maskSecret(const String &s) {
  if (s.length() == 0) return "(vide)";
  return String("****") + " (" + s.length() + " car.)";
}

static void printConfig() {
  Serial.println(F("--- configuration ---"));
  Serial.printf("fw        : %s\n", FW_VERSION);
  Serial.printf("door_id   : %s\n", cfg.doorId.c_str());
  Serial.printf("ssid      : %s\n", cfg.ssid.c_str());
  Serial.printf("wifi_pass : %s\n", maskSecret(cfg.wifiPass).c_str());
  Serial.printf("token     : %s\n", maskSecret(cfg.token).c_str());
  Serial.printf("pulse_ms  : %lu\n", (unsigned long)cfg.pulseMs);
  Serial.printf("cool_ms   : %lu\n", (unsigned long)cfg.cooldownMs);
  Serial.printf("leds      : live=%d wifi=%d pulse=%d\n", cfg.ledLivePin, cfg.ledWifiPin, cfg.ledPulsePin);
  Serial.printf("input     : pin=%d active_low=%d\n", cfg.inputPin, cfg.inputActiveLow);
  Serial.printf("close     : %s\n", cfg.closeEnabled ? "1 (fermeture commandee, K2 actif)" : "0 (la porte se referme seule, K2 inactif)");
  Serial.printf("wifi      : %s ip=%s rssi=%d\n", WiFi.isConnected() ? "connecte" : "deconnecte",
                WiFi.localIP().toString().c_str(), WiFi.isConnected() ? WiFi.RSSI() : 0);
  Serial.printf("valide    : %s\n", configValid() ? "oui" : "NON (door_id, ssid, token>=16 requis)");
}

static void printHelp() {
  Serial.println(F("Commandes :"));
  Serial.println(F("  show                      affiche la configuration (secrets masques)"));
  Serial.println(F("  set door_id <P01>         identifiant de la porte"));
  Serial.println(F("  set ssid <nom>            reseau Wi-Fi"));
  Serial.println(F("  set wifi_pass <mdp>       mot de passe Wi-Fi"));
  Serial.println(F("  set token <jeton>         jeton robot->porte (>= 16 caracteres)"));
  Serial.println(F("  set pulse_ms <100..2000>  duree d'impulsion"));
  Serial.println(F("  set cool_ms <>=1000>      delai mini entre deux impulsions"));
  Serial.println(F("  set led_live|led_wifi|led_pulse <gpio|-1>"));
  Serial.println(F("  set in_pin <gpio|-1> ; set in_low <0|1>  contact porte ouverte (option)"));
  Serial.println(F("  set close <0|1>           1 = porte a fermeture commandee (K2 autorise)"));
  Serial.println(F("  test_pulse                impulsion K1 locale (essai sur banc uniquement)"));
  Serial.println(F("  test_close                impulsion K2 locale (seulement si close=1)"));
  Serial.println(F("  reboot"));
}

static void handleSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line == "help") { printHelp(); return; }
  if (line == "show") { printConfig(); return; }
  if (line == "reboot") { relaysSafe(); delay(100); ESP.restart(); }
  if (line == "test_pulse") {
    String reason;
    bool ok = startPulse(PULSE_OPEN, reason);
    Serial.printf("test_pulse : %s (%s)\n", ok ? "OK" : "refuse", reason.c_str());
    return;
  }
  if (line == "test_close") {
    if (!cfg.closeEnabled) {
      Serial.println(F("test_close : refuse, fermeture commandee desactivee (set close 1 puis reboot)"));
      return;
    }
    String reason;
    bool ok = startPulse(PULSE_CLOSE, reason);
    Serial.printf("test_close : %s (%s)\n", ok ? "OK" : "refuse", reason.c_str());
    return;
  }
  if (line.startsWith("set ")) {
    String rest = line.substring(4);
    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println(F("syntaxe : set <cle> <valeur>")); return; }
    String key = rest.substring(0, sp);
    String val = rest.substring(sp + 1);
    val.trim();
    prefs.begin("door", false);
    bool ok = true;
    if (key == "door_id") prefs.putString("door_id", val);
    else if (key == "ssid") prefs.putString("ssid", val);
    else if (key == "wifi_pass") prefs.putString("wpass", val);
    else if (key == "token") {
      if (val.length() < 16) { Serial.println(F("jeton trop court (16 min)")); ok = false; }
      else prefs.putString("token", val);
    }
    else if (key == "pulse_ms") prefs.putUInt("pulse_ms", (uint32_t)val.toInt());
    else if (key == "cool_ms") prefs.putUInt("cool_ms", (uint32_t)val.toInt());
    else if (key == "led_live") prefs.putInt("led_live", val.toInt());
    else if (key == "led_wifi") prefs.putInt("led_wifi", val.toInt());
    else if (key == "led_pulse") prefs.putInt("led_pulse", val.toInt());
    else if (key == "in_pin") prefs.putInt("in_pin", val.toInt());
    else if (key == "in_low") prefs.putBool("in_low", val.toInt() != 0);
    else if (key == "close") prefs.putBool("close", val.toInt() != 0);
    else { Serial.println(F("cle inconnue")); ok = false; }
    prefs.end();
    if (ok) Serial.println(F("enregistre. Taper 'reboot' pour appliquer."));
    return;
  }
  Serial.println(F("commande inconnue, taper 'help'"));
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length()) handleSerialCommand(serialLine);
      serialLine = "";
    } else if (serialLine.length() < 200) {
      serialLine += c;
    }
  }
}

// ---------------- HTTP ----------------
static String jsonEscape(const String &s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) { o += ' '; }
    else o += c;
  }
  return o;
}

// Comparaison en temps constant pour éviter de révéler le jeton par la durée.
static bool constantTimeEquals(const String &a, const String &b) {
  if (a.length() != b.length()) return false;
  uint8_t diff = 0;
  for (size_t i = 0; i < a.length(); i++) diff |= (uint8_t)a[i] ^ (uint8_t)b[i];
  return diff == 0;
}

static bool authorized() {
  String h = server.header("Authorization");
  const String prefix = "Bearer ";
  if (!h.startsWith(prefix)) return false;
  return constantTimeEquals(h.substring(prefix.length()), cfg.token);
}

static void sendJson(int code, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

static void rejectAuth() {
  rejectedAuthCount++;
  sendJson(401, "{\"ok\":false,\"error\":\"unauthorized\"}");
}

static String inputState() {
  if (cfg.inputPin < 0) return "not_configured";
  bool raw = digitalRead(cfg.inputPin) == HIGH;
  bool active = cfg.inputActiveLow ? !raw : raw;
  return active ? "open_contact_active" : "open_contact_inactive";
}

// Extrait op_id du paramètre d'URL ou d'un corps JSON simple {"op_id":"..."}.
static String extractOpId() {
  if (server.hasArg("op_id")) return server.arg("op_id").substring(0, 64);
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int k = body.indexOf("\"op_id\"");
    if (k >= 0) {
      int q1 = body.indexOf('"', body.indexOf(':', k) + 1);
      int q2 = (q1 >= 0) ? body.indexOf('"', q1 + 1) : -1;
      if (q1 >= 0 && q2 > q1) return body.substring(q1 + 1, q2).substring(0, 64);
    }
  }
  return "";
}

// Vérifie que l'identifiant de porte demandé (si fourni) correspond à cette carte.
static bool doorIdMatches() {
  if (!server.hasArg("door_id")) return true;
  return server.arg("door_id") == cfg.doorId;
}

static void handleHealth() {
  sendJson(200, String("{\"ok\":true,\"fw\":\"") + FW_VERSION + "\"}");
}

static void handleStatus() {
  if (!authorized()) { rejectAuth(); return; }
  uint32_t now = millis();
  long sinceLast = everPulsed ? (long)(now - lastPulseMs) : -1;
  long coolLeft = 0;
  if (everPulsed && (now - lastPulseMs) < cfg.cooldownMs) coolLeft = cfg.cooldownMs - (now - lastPulseMs);
  String b = "{\"ok\":true";
  b += ",\"door_id\":\"" + jsonEscape(cfg.doorId) + "\"";
  b += ",\"fw\":\"" FW_VERSION "\"";
  b += ",\"mode\":\"auto\"";
  b += ",\"uptime_ms\":" + String(now);
  b += ",\"rssi\":" + String(WiFi.RSSI());
  b += ",\"relay_active\":" + String(pulseActive ? "true" : "false");
  b += ",\"pulse_ms\":" + String(cfg.pulseMs);
  b += ",\"pulse_count\":" + String(pulseCount);
  b += ",\"ms_since_last_pulse\":" + String(sinceLast);
  b += ",\"cooldown_left_ms\":" + String(coolLeft);
  b += ",\"rejected_auth\":" + String(rejectedAuthCount);
  b += ",\"open_input\":\"" + inputState() + "\"";
  b += ",\"close_enabled\":" + String(cfg.closeEnabled ? "true" : "false");
  b += ",\"close_count\":" + String(closeCount);
  b += ",\"ms_since_last_close\":" + String(everClosed ? (long)(now - lastCloseMs) : -1L);
  b += ",\"relay_close_active\":" + String(pulseActive && pulsePin == RELAY_CLOSE_PIN ? "true" : "false");
  b += "}";
  sendJson(200, b);
}

// Traite une demande d'impulsion (ouverture K1 ou fermeture K2).
// 200 : impulsion envoyée, ou demande sans effet mais cohérente (rejeu, anti-rebond,
//       impulsion déjà en cours) ; 409 : refus explicite (mauvaise porte, fermeture non
//       autorisée sur cette porte, verrou ouverture/fermeture).
static void handlePulseRequest(PulseKind kind) {
  if (!authorized()) { rejectAuth(); return; }
  if (!doorIdMatches()) {
    sendJson(409, "{\"ok\":false,\"error\":\"door_id_mismatch\",\"door_id\":\"" + jsonEscape(cfg.doorId) + "\"}");
    return;
  }
  String opId = extractOpId();
  String opKey = String(kind == PULSE_OPEN ? "open:" : "close:") + opId;
  uint32_t now = millis();
  // Idempotence : une même opération rejouée ne redéclenche pas d'impulsion.
  if (opId.length() > 0 && opKey == lastOpId && (now - lastOpIdMs) < OPID_MEMORY_MS) {
    sendJson(200, "{\"ok\":true,\"pulsed\":false,\"reason\":\"duplicate_op_id\",\"previous\":\"" + lastOpResult + "\",\"door_id\":\"" + jsonEscape(cfg.doorId) + "\"}");
    return;
  }
  String reason;
  bool pulsed = startPulse(kind, reason);
  bool refused = (reason == "close_disabled" || reason == "too_soon_after_open" || reason == "too_soon_after_close");
  if (opId.length() > 0 && !refused) { lastOpId = opKey; lastOpIdMs = now; lastOpResult = reason; }
  String b = String("{\"ok\":") + (refused ? "false" : "true") + ",\"pulsed\":";
  b += pulsed ? "true" : "false";
  b += ",\"reason\":\"" + reason + "\"";
  b += ",\"relay\":\"" + String(kind == PULSE_OPEN ? "K1" : "K2") + "\"";
  b += ",\"pulse_ms\":" + String(cfg.pulseMs);
  b += ",\"door_id\":\"" + jsonEscape(cfg.doorId) + "\"";
  b += ",\"op_id\":\"" + jsonEscape(opId) + "\"}";
  sendJson(refused ? 409 : 200, b);
}

static void handleOpen() { handlePulseRequest(PULSE_OPEN); }

// Fermeture : seulement pour les portes qui ne se referment pas seules ("set close 1").
static void handleClose() { handlePulseRequest(PULSE_CLOSE); }

static void handleNotFound() {
  sendJson(404, "{\"ok\":false,\"error\":\"not_found\"}");
}

// ---------------- Wi-Fi ----------------
static void wifiStart() {
  // Core 3.x : le nom d'hôte doit être fixé avant WiFi.mode().
  WiFi.setHostname((String("spot-door-") + cfg.doorId).c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfg.ssid.c_str(), cfg.wifiPass.c_str());
  lastWifiAttemptMs = millis();
}

static void wifiService() {
  if (WiFi.isConnected()) return;
  if (millis() - lastWifiAttemptMs > 15000) {
    WiFi.disconnect();
    WiFi.begin(cfg.ssid.c_str(), cfg.wifiPass.c_str());
    lastWifiAttemptMs = millis();
  }
}

// ---------------- LEDs ----------------
static void ledService() {
  uint32_t now = millis();
  ledWrite(cfg.ledLivePin, (now / 500) % 2);
  if (cfg.ledWifiPin >= 0) {
    if (WiFi.isConnected()) ledWrite(cfg.ledWifiPin, true);
    else ledWrite(cfg.ledWifiPin, (now / 100) % 2);
  }
}

// ---------------- Arduino ----------------
void setup() {
  // Première action : sorties relais au repos, avant toute autre initialisation.
  pinMode(RELAY_OPEN_PIN, OUTPUT);
  pinMode(RELAY_CLOSE_PIN, OUTPUT);
  relaysSafe();

  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(FW_VERSION);

  loadConfig();
  if (cfg.ledLivePin >= 0) pinMode(cfg.ledLivePin, OUTPUT);
  if (cfg.ledWifiPin >= 0) pinMode(cfg.ledWifiPin, OUTPUT);
  if (cfg.ledPulsePin >= 0) pinMode(cfg.ledPulsePin, OUTPUT);
  if (cfg.inputPin >= 0) pinMode(cfg.inputPin, INPUT);

  // Chien de garde : en cas de blocage du programme, redémarrage -> relais au repos.
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdtConfig);
  esp_task_wdt_add(NULL);

  printConfig();
  if (!configValid()) {
    Serial.println(F("Configuration incomplete : le serveur HTTP reste arrete. Taper 'help'."));
    return;
  }

  wifiStart();
  const char *headerKeys[] = {"Authorization"};
  server.collectHeaders(headerKeys, 1);
  server.on("/api/v1/health", HTTP_GET, handleHealth);
  server.on("/api/v1/status", HTTP_GET, handleStatus);
  server.on("/api/v1/open", HTTP_POST, handleOpen);
  server.on("/api/v1/close", HTTP_POST, handleClose);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("Serveur HTTP demarre (port 80)."));
}

void loop() {
  esp_task_wdt_reset();
  servicePulse();          // en premier : l'impulsion se termine toujours à l'heure
  pollSerial();
  if (configValid()) {
    wifiService();
    server.handleClient();
    servicePulse();
  }
  ledService();
  delay(2);
}
