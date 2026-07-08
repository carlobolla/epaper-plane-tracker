// ESP32 ADS-B "closest aircraft" tracker, rendered on a 5.79" GxEPD2 e-paper
// display (792x272, GDEY0579T93).
//
// Hybrid data sources:
//   - OpenSky (free): polled every 30s for live state vectors -- position,
//     altitude, vertical speed, track, groundspeed, squawk, callsign.
//   - FlightAware AeroAPI (billed per query): called only when a NEW aircraft
//     becomes closest -- type, registration, route (ICAO codes).
//   - aviationweather.gov (free): METAR for the bottom bar.

#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <time.h>
#include <config.h>

#include <SPI.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>


// ============================== DISPLAY ==============================
// --- E-paper pins ---
#define EPD_BUSY  45
#define EPD_RST   48
#define EPD_DC    47
#define EPD_CS    21
#define EPD_SCK   20
#define EPD_MOSI  19

GxEPD2_BW<GxEPD2_579_GDEY0579T93, GxEPD2_579_GDEY0579T93::HEIGHT> display(
  GxEPD2_579_GDEY0579T93(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// ============================== GLOBALS ==============================

String           g_token;
unsigned long    g_tokenExpiry      = 0;
WiFiClientSecure g_secureClient;

bool             g_firstRender      = true;
int              g_partialCount     = 0;
unsigned long    g_lastUpdateMillis = 0;

char             g_metarRaw[160]    = "";

// FlightAware: queries are billed per call, so only re-query
// when the closest aircraft's callsign changes.
char             g_faCallsign[10] = "";
char             g_faType[16]     = "N/A";
char             g_faReg[12]      = "N/A";
char             g_depIcao[8]     = "----";
char             g_arrIcao[8]     = "----";

// ============================== DATA MODEL ==============================

struct AircraftInfo {
  bool  found    = false;
  char  icao[8]      = "N/A";
  char  call[10]     = "N/A";
  char  typecode[16] = "N/A";
  char  registration[12] = "N/A";
  char  squawk[6]    = "----";
  char  depIcao[8]   = "----";
  char  arrIcao[8]   = "----";
  float dist     = -1;
  float bearing  = 0;
  bool  onGround = false;
  float altFt    = -1;
  float spdKts   = -1;
  float track    = -1;
  float vrFpm    = 0;
};

// === GEO / FIELD HELPERS ===

float distanceNM(float lat1, float lon1, float lat2, float lon2) {
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sinf(dLat / 2) * sinf(dLat / 2)
          + cosf(radians(lat1)) * cosf(radians(lat2)) * sinf(dLon / 2) * sinf(dLon / 2);
  return 3440.065f * 2.0f * atan2f(sqrtf(a), sqrtf(1 - a));
}

float bearingTo(float lat1, float lon1, float lat2, float lon2) {
  float dL  = radians(lon2 - lon1);
  float rl1 = radians(lat1), rl2 = radians(lat2);
  float x = cos(rl2) * sin(dL);
  float y = cos(rl1) * sin(rl2) - sin(rl1) * cos(rl2) * cos(dL);
  return fmod(degrees(atan2(x, y)) + 360.0f, 360.0f);
}

const char* cardinal(float deg) {
  const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return dirs[(int)((deg + 22.5f) / 45.0f) % 8];
}

// Copies a JSON array field into a fixed buffer, falling back to `def` if null,
// and trims leading/trailing whitespace (OpenSky pads callsign with spaces).
void copyJsonField(JsonDocument& d, int index, const char* def, char* out, size_t sz) {
  strlcpy(out, d[index].isNull() ? def : d[index].as<const char*>(), sz);

  size_t len = strlen(out);
  while (len > 0 && isspace((unsigned char)out[len - 1])) out[--len] = '\0';

  size_t start = 0;
  while (out[start] && isspace((unsigned char)out[start])) start++;
  if (start > 0) memmove(out, out + start, len - start + 1);
}

float copyJsonFloat(JsonDocument& d, int index, float def = -1.0f) {
  return d[index].isNull() ? def : d[index].as<float>();
}

// === E-PAPER RENDERING ===

void drawCentered(const char* text, int cx, int y) {
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(text, 0, 0, &tbx, &tby, &tbw, &tbh);
  display.setCursor(cx - tbw / 2 - tbx, y);
  display.print(text);
}

// Filled arrow centered at (cx, cy), pointing in headingDeg (compass degrees).
void drawArrow(int cx, int cy, float headingDeg, float len, float halfWidth, uint16_t color) {
  float rad = radians(headingDeg);
  float dx = sin(rad), dy = -cos(rad);
  float px = -dy,      py =  dx;

  int tipX  = cx + (int)(dx * len);
  int tipY  = cy + (int)(dy * len);
  int tailX = cx - (int)(dx * len * 0.6f);
  int tailY = cy - (int)(dy * len * 0.6f);

  display.fillTriangle(
    tipX, tipY,
    tailX + (int)(px * halfWidth), tailY + (int)(py * halfWidth),
    tailX - (int)(px * halfWidth), tailY - (int)(py * halfWidth),
    color);
}

// Small plane silhouette centered at (cx, cy), nose pointing headingDeg
// s = half-length in pixels. Built from three filled triangles
void drawPlaneIcon(int cx, int cy, float headingDeg, float s, uint16_t color) {
  float rad = radians(headingDeg);
  float dx = sinf(rad), dy = -cosf(rad); // forward unit vector (screen coords)
  float px = -dy,       py =  dx;        // perpendicular unit vector

  int noseX = cx + (int)(dx * s),         noseY = cy + (int)(dy * s);
  int tailX = cx - (int)(dx * s * 0.85f), tailY = cy - (int)(dy * s * 0.85f);

  // Fuselage: thin triangle from nose to tail
  display.fillTriangle(noseX, noseY,
    tailX + (int)(px * s * 0.20f), tailY + (int)(py * s * 0.20f),
    tailX - (int)(px * s * 0.20f), tailY - (int)(py * s * 0.20f), color);

  // Main wings: wide triangle, root ahead of center, tips slightly aft (swept)
  int rootX = cx + (int)(dx * s * 0.35f), rootY = cy + (int)(dy * s * 0.35f);
  int aftX  = cx - (int)(dx * s * 0.15f), aftY  = cy - (int)(dy * s * 0.15f);
  display.fillTriangle(rootX, rootY,
    aftX + (int)(px * s * 0.90f), aftY + (int)(py * s * 0.90f),
    aftX - (int)(px * s * 0.90f), aftY - (int)(py * s * 0.90f), color);

  // Tailplane: small triangle at the rear
  int tpX = cx - (int)(dx * s * 0.60f), tpY = cy - (int)(dy * s * 0.60f);
  display.fillTriangle(tpX, tpY,
    tailX + (int)(px * s * 0.45f), tailY + (int)(py * s * 0.45f),
    tailX - (int)(px * s * 0.45f), tailY - (int)(py * s * 0.45f), color);
}

void getTimestampString(char* buf, size_t n) {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 50)) {
    strftime(buf, n, "%H:%M:%S", &timeinfo);
  } else {
    unsigned long sAgo = (millis() - g_lastUpdateMillis) / 1000UL;
    snprintf(buf, n, "upd %lus ago", sAgo);
  }
}

// y-position of the small status line drawn under the big "Connecting..." text
static const int CONNECTING_STATUS_Y = 272 / 2 + 8 + 30;

void drawConnectingScreen(const char* status = "") {
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&FreeMonoBold18pt7b);
    drawCentered("Connecting...", display.width() / 2, display.height() / 2 + 8);

    if (status[0] != '\0') {
      display.setFont(&FreeMonoBold9pt7b);
      drawCentered(status, display.width() / 2, CONNECTING_STATUS_Y);
    }
  } while (display.nextPage());
  display.hibernate();
}

// Redraws only the small status line under "Connecting...", using a partial
// window so repeated calls (e.g. once per retry) don't trigger a slow full
// e-paper refresh each time.
void updateConnectingStatus(const char* status) {
  const int cx    = display.width() / 2;
  const int rectW = 500;
  const int rectH = 26;
  const int rectX = cx - rectW / 2;
  const int rectY = CONNECTING_STATUS_Y - 20;

  display.setRotation(0);
  display.setPartialWindow(rectX, rectY, rectW, rectH);
  display.firstPage();
  do {
    display.fillRect(rectX, rectY, rectW, rectH, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    drawCentered(status, cx, CONNECTING_STATUS_Y);
  } while (display.nextPage());
}

void drawAircraftScreen(const AircraftInfo& a) {
  display.setRotation(0);

  bool doFullRefresh = g_firstRender || (g_partialCount >= FULL_REFRESH_EVERY);
  if (doFullRefresh) {
    display.setFullWindow();
  } else {
    display.setPartialWindow(0, 0, display.width(), display.height());
  }

  const int W = display.width();   // 792
  const int H = display.height();  // 272

  // Three-column layout: left text | center text | right compass dial.
  const int compassCx = 670;
  const int compassCy = 125;
  const int r         = 58;
  const int leftCx    = 145;
  const int centerCx  = 445;

  // Vertical baselines (shared by both text columns unless noted).
  const int Y_TITLE   =  20; // "Closest aircraft" (9pt)
  const int Y_HEADER  =  58; // combined callsign - type (18pt), centered in text area
  const int Y_ROUTE   =  88; // route
  const int Y_LINE1   = 118; // alt /
  const int Y_LINE2   = 143; // v/s / registration
  const int Y_LINE3   = 168; // track / squawk
  const int Y_LINE4   = 193; // speed / icao24
  const int Y_METAR_SINGLE = 237; // METAR if one line
  const int Y_METAR_1   = 228; // METAR first line (9pt)
  const int Y_METAR_2   = 246; // METAR second line (if needed)
  const int Y_TIMESTAMP = 267; // timestamp, bottom-right corner

  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    // Title
    display.setFont(&FreeMonoBold9pt7b);
    char title[40];
    snprintf(title, sizeof(title), "Closest aircraft");
    drawCentered(title, (leftCx + centerCx) / 2, Y_TITLE);

    if (!a.found) {
      display.setFont(&FreeMonoBold18pt7b);
      drawCentered("NO AIRCRAFT IN RANGE", (leftCx + centerCx) / 2, compassCy + 8);
    } else {
      char buf[48];

      // Header
      display.setFont(&FreeMonoBold18pt7b);
      snprintf(buf, sizeof(buf), "%s", a.call);
      drawCentered(buf, (leftCx + centerCx) / 2, Y_HEADER);

      // Route
      display.setFont(&FreeMonoBold12pt7b);
      {
        int16_t bx, by; uint16_t depW, arrW, tbh;
        display.getTextBounds(a.depIcao, 0, 0, &bx, &by, &depW, &tbh);
        display.getTextBounds(a.arrIcao, 0, 0, &bx, &by, &arrW, &tbh);

        const int iconW = 24;
        const int gap   = 28;
        int total  = iconW + depW + gap + iconW + arrW;
        int x      = ((leftCx + centerCx) / 2) - (total / 2);
        int iconCy = Y_ROUTE - 6;

        drawPlaneIcon(x + 9, iconCy,  45.0f, 10.0f, GxEPD_BLACK);
        display.setCursor(x + iconW, Y_ROUTE);
        display.print(a.depIcao);

        x += iconW + depW + gap;
        drawPlaneIcon(x + 9, iconCy, 135.0f, 10.0f, GxEPD_BLACK);
        display.setCursor(x + iconW, Y_ROUTE);
        display.print(a.arrIcao);
      }

      // Left column: alt / v-s / track / speed
      display.setFont(&FreeMonoBold12pt7b);
      if (a.onGround) {
        drawCentered("Alt: ON GROUND", leftCx, Y_LINE1);
      } else {
        snprintf(buf, sizeof(buf), "Alt: %.0f ft", a.altFt);
        drawCentered(buf, leftCx, Y_LINE1);
      }
      snprintf(buf, sizeof(buf), "V/S: %+.0f fpm", a.vrFpm);
      drawCentered(buf, leftCx, Y_LINE2);
      snprintf(buf, sizeof(buf), "Trk: %.0f deg", a.track);
      drawCentered(buf, leftCx, Y_LINE3);
      snprintf(buf, sizeof(buf), "Spd: %.0f kt", a.spdKts);
      drawCentered(buf, leftCx, Y_LINE4);

      // Center column: type / reg / squawk / icao24
      display.setFont(&FreeMonoBold12pt7b);
      snprintf(buf, sizeof(buf), "Type: %s", a.typecode);
      drawCentered(buf, centerCx, Y_LINE1);
      snprintf(buf, sizeof(buf), "Reg: %s", a.registration);
      drawCentered(buf, centerCx, Y_LINE2);
      snprintf(buf, sizeof(buf), "SQK: %s", a.squawk);
      drawCentered(buf, centerCx, Y_LINE3);
      snprintf(buf, sizeof(buf), "I24: %s", a.icao);
      drawCentered(buf, centerCx, Y_LINE4);

      // Right: compass dial
      display.setFont(&FreeMonoBold9pt7b);
      display.drawCircle(compassCx, compassCy, r, GxEPD_BLACK);
      drawCentered("N", compassCx,          compassCy - r - 12);
      drawCentered("S", compassCx,          compassCy + r + 22);
      drawCentered("W", compassCx - r - 18, compassCy + 5);
      drawCentered("E", compassCx + r + 14, compassCy + 5);

      float rad = radians(a.bearing);
      int markerX = compassCx + (int)(r * sin(rad));
      int markerY = compassCy - (int)(r * cos(rad));

      float headingForArrow = (a.track >= 0) ? a.track : a.bearing;
      drawArrow(markerX, markerY, headingForArrow, 12, 8, GxEPD_BLACK);

      // Bearing readout
      char numStr[8];
      snprintf(numStr, sizeof(numStr), "%.0f", a.bearing);
      const char* card = cardinal(a.bearing);

      int16_t numTbx, numTby; uint16_t numW, numTbh;
      display.getTextBounds(numStr, 0, 0, &numTbx, &numTby, &numW, &numTbh);
      int16_t cardTbx, cardTby; uint16_t cardW, cardTbh;
      display.getTextBounds(card, 0, 0, &cardTbx, &cardTby, &cardW, &cardTbh);

      const int gap = 4, circR = 2;
      int total  = numW + gap + circR * 2 + gap + cardW;
      int lineY  = compassCy - 4;
      int startX = compassCx - total / 2;

      display.setCursor(startX - numTbx, lineY);
      display.print(numStr);
      display.fillCircle(startX + numW + gap + circR, lineY - 8, circR, GxEPD_BLACK);
      display.setCursor(startX + numW + gap + circR * 2 + gap - cardTbx, lineY);
      display.print(card);

      snprintf(buf, sizeof(buf), "%.1fNM", a.dist);
      drawCentered(buf, compassCx, compassCy + 14);
    }

    // Bottom bar: METAR, then timestamp bottom-right
    display.setFont(&FreeMonoBold9pt7b);

    // METAR
    if (g_metarRaw[0] != '\0') {
      const int maxChars = (W - 24) / 11; // ~69 chars at 9pt, ~11px per char

      if ((int)strlen(g_metarRaw) <= maxChars) {
        // Fits on one line
        display.setCursor(12, Y_METAR_SINGLE);
        display.print(g_metarRaw);
      } else {
        // Find the last space at or before the line limit to word-wrap cleanly
        int splitAt = maxChars;
        while (splitAt > 0 && g_metarRaw[splitAt] != ' ') splitAt--;
        if (splitAt == 0) splitAt = maxChars; // no space found, hard split

        // Line 1
        char line1[160];
        strlcpy(line1, g_metarRaw, splitAt + 1); // +1 for null terminator
        display.setCursor(12, Y_METAR_1);
        display.print(line1);

        // Line 2, truncate if still too long
        const char* rest = g_metarRaw + splitAt + (g_metarRaw[splitAt] == ' ' ? 1 : 0);
        char line2[160];
        strlcpy(line2, rest, sizeof(line2));
        if ((int)strlen(line2) > maxChars) {
          line2[maxChars - 2] = '.';
          line2[maxChars - 1] = '.';
          line2[maxChars]     = '\0';
        }
        display.setCursor(12, Y_METAR_2);
        display.print(line2);
      }
    }

    // Timestamp (right-aligned, bottom corner)
    char ts[24];
    getTimestampString(ts, sizeof(ts));
    int16_t tsBx, tsBY; uint16_t tsW, tsTbh;
    display.getTextBounds(ts, 0, 0, &tsBx, &tsBY, &tsW, &tsTbh);
    display.setCursor(W - 8 - tsW, Y_TIMESTAMP);
    display.print(ts);

  } while (display.nextPage());

  display.hibernate();

  g_firstRender  = false;
  g_partialCount = doFullRefresh ? 0 : (g_partialCount + 1);
}

// === NETWORK ===

const char* wifiStatusStr(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:     return "idle";
    case WL_NO_SSID_AVAIL:   return "SSID not found";
    case WL_SCAN_COMPLETED:  return "scan complete";
    case WL_CONNECTED:       return "connected";
    case WL_CONNECT_FAILED:  return "connect failed";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED:    return "disconnected";
    default:                 return "unknown";
  }
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.disconnect(true);
  delay(1000);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  char statusMsg[64];
  snprintf(statusMsg, sizeof(statusMsg), "SSID: %s", WIFI_SSID);
  drawConnectingScreen(statusMsg);

  const int MAX_RETRIES = 20;
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (retries >= MAX_RETRIES) {
      Serial.println("WiFi failed, restarting...");
      updateConnectingStatus("Failed to connect - restarting...");
      delay(1500);
      ESP.restart();
    }
    delay(3000);
    Serial.print(wifiStatusStr(WiFi.status()));
    retries++;

    snprintf(statusMsg, sizeof(statusMsg), "%s  (attempt %d/%d, %s)",
             WIFI_SSID, retries, MAX_RETRIES, wifiStatusStr(WiFi.status()));
    updateConnectingStatus(statusMsg);
  }

  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
  snprintf(statusMsg, sizeof(statusMsg), "Connected: %s", WiFi.localIP().toString().c_str());
  updateConnectingStatus(statusMsg);
  delay(600);
}

void refreshToken() {
  Serial.println("[Auth] Fetching token...");

  HTTPClient http;
  http.setTimeout(15000);
  http.begin(g_secureClient, OPENSKY_TOKEN_URL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "grant_type=client_credentials&client_id=";
  body += OPENSKY_CLIENT_ID;
  body += "&client_secret=";
  body += OPENSKY_CLIENT_SECRET;

  int httpCode = http.POST(body);
  if (httpCode < 0) {
    Serial.printf("[Auth] Error: %s\n", http.errorToString(httpCode).c_str());
  }
  if (httpCode == HTTP_CODE_OK) {
    StaticJsonDocument<512> doc;
    deserializeJson(doc, http.getStream());
    g_token = doc["access_token"].as<String>();
    int expiresIn = doc["expires_in"] | 1800;
    g_tokenExpiry = millis() + (expiresIn - 60) * 1000UL;
    Serial.println("[Auth] Token OK");
  } else {
    Serial.println("[Auth] Token fetch failed");
  }
  http.end();
}

// Looks up aircraft type, registration, and route
// for a callsign via FlightAware AeroAPI v4 (GET /flights/{ident}) and stores
// the results in the g_fa* cache globals. Uses a JSON filter so only the
// fields we need are parsed. Prefers the flight currently en route (actual_off set, actual_on
// null); falls back to the first flight returned.
void lookupFlightAware(const char* callsign) {
  strlcpy(g_faType,     "N/A",  sizeof(g_faType));
  strlcpy(g_faReg,      "N/A",  sizeof(g_faReg));
  strlcpy(g_depIcao,    "----", sizeof(g_depIcao));
  strlcpy(g_arrIcao,    "----", sizeof(g_arrIcao));

  char url[120];
  snprintf(url, sizeof(url), "https://%s/aeroapi/flights/%s?max_pages=1",
           FLIGHTAWARE_API_HOST, callsign);
  Serial.printf("[FA] GET %s\n", url);

  HTTPClient http;
  // AeroAPI uses chunked transfer encoding with HTTP/1.1, which getStream()
  // does NOT decode. ArduinoJson would choke on the raw chunk-size lines.
  // Forcing HTTP/1.0 makes the server send a plain body.
  http.useHTTP10(true);
  http.begin(g_secureClient, url);
  http.setTimeout(15000);
  http.addHeader("x-apikey", FLIGHTAWARE_API_KEY);

  int code = http.GET();
  Serial.printf("[FA] HTTP %d\n", code);

  if (code == HTTP_CODE_OK) {
    StaticJsonDocument<512> filter;
    filter["flights"][0]["origin"]["code_icao"]      = true;
    filter["flights"][0]["destination"]["code_icao"] = true;
    filter["flights"][0]["aircraft_type"]            = true;
    filter["flights"][0]["registration"]             = true;
    filter["flights"][0]["actual_off"]               = true;
    filter["flights"][0]["actual_on"]                = true;

    DynamicJsonDocument doc(8192);
    DeserializationError err =
      deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));

    if (err) {
      Serial.printf("[FA] JSON error: %s\n", err.c_str());
    } else if (!doc.containsKey("flights")) {
      Serial.println("[FA] Unexpected response shape (no 'flights' key)");
    } else {
      JsonArray flights = doc["flights"].as<JsonArray>();
      JsonObject pick;
      for (JsonObject f : flights) {
        // En-route flight: departed but not yet landed.
        if (!f["actual_off"].isNull() && f["actual_on"].isNull()) { pick = f; break; }
      }
      if (pick.isNull() && flights.size() > 0) pick = flights[0];

      if (!pick.isNull()) {
        strlcpy(g_faType,     pick["aircraft_type"]            | "N/A",  sizeof(g_faType));
        strlcpy(g_faReg,      pick["registration"]             | "N/A",  sizeof(g_faReg));
        strlcpy(g_depIcao,    pick["origin"]["code_icao"]      | "----", sizeof(g_depIcao));
        strlcpy(g_arrIcao,    pick["destination"]["code_icao"] | "----", sizeof(g_arrIcao));
        Serial.printf("[FA] %s (%s): %s -> %s\n",
                      g_faType, g_faReg, g_depIcao, g_arrIcao);
      } else {
        Serial.println("[FA] No flights returned");
      }
    }
  } else {
    // 400/404 = ident unknown to FlightAware. Leave defaults in place.
    Serial.printf("[FA] Error body: %s\n", http.getString().c_str());
  }
  http.end();
}

// Cached: only hits FlightAware when the callsign changes.
void applyFlightAware(AircraftInfo& a) {
  bool valid = a.call[0] != '\0' && strcmp(a.call, "N/A") != 0;
  if (!valid) return; // struct defaults (N/A / ----) already in place

  if (strcmp(a.call, g_faCallsign) != 0) {
    lookupFlightAware(a.call);
    strlcpy(g_faCallsign, a.call, sizeof(g_faCallsign));
  }

  strlcpy(a.typecode,     g_faType,     sizeof(a.typecode));
  strlcpy(a.registration, g_faReg,      sizeof(a.registration));
  strlcpy(a.depIcao,      g_depIcao,    sizeof(a.depIcao));
  strlcpy(a.arrIcao,      g_arrIcao,    sizeof(a.arrIcao));
}

// Fetches a METAR from aviationweather.gov
void fetchMetar() {
  char url[120];
  snprintf(url, sizeof(url),
    "https://aviationweather.gov/api/data/metar?ids=%s&format=json",
    METAR_AIRPORT);
  Serial.printf("[METAR] GET %s\n", url);

  HTTPClient http;
  http.begin(g_secureClient, url);
  http.setTimeout(15000);

  int code = http.GET();
  Serial.printf("[METAR] HTTP %d\n", code);

  if (code == HTTP_CODE_OK) {
    StaticJsonDocument<2048> doc;
    DeserializationError err = deserializeJson(doc, http.getStream());
    if (err) {
      Serial.printf("[METAR] JSON error: %s\n", err.c_str());
    } else {
      JsonArray arr = doc.as<JsonArray>();
      if (arr.size() > 0) {
        strlcpy(g_metarRaw, arr[0]["rawOb"] | "", sizeof(g_metarRaw));
        Serial.printf("[METAR] %s\n", g_metarRaw);
      }
    }
  } else {
    Serial.printf("[METAR] Error: %d\n", code);
  }
  http.end();
}

void fetchClosestAircraft() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
    connectWiFi();
  }
  if (g_token.isEmpty() || millis() >= g_tokenExpiry) refreshToken();

  char url[220];
  snprintf(url, sizeof(url),
    "https://%s/api/states/all?lamin=%.4f&lomin=%.4f&lamax=%.4f&lomax=%.4f",
    OPENSKY_API_HOST,
    HOME_LAT - SEARCH_BOX_DEG, HOME_LON - SEARCH_BOX_DEG,
    HOME_LAT + SEARCH_BOX_DEG, HOME_LON + SEARCH_BOX_DEG);
  Serial.printf("[HTTP] GET %s\n", url);

  HTTPClient http;
  http.begin(g_secureClient, url);
  http.setReuse(false);
  http.setTimeout(15000);
  if (!g_token.isEmpty()) http.addHeader("Authorization", "Bearer " + g_token);

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[HTTP] Error %d\n", code);
    http.end();
    return;
  }

  // Stream-parse the "states" array: find the key, skip to the array,
  // then read one sub-array at a time to keep peak memory low.
  WiFiClient* stream = http.getStreamPtr();
  unsigned long deadline = millis() + 15000UL;

  const char* key = "\"states\"";
  int keyMatchLen = 0;
  while (millis() < deadline) {
    if (!stream->available()) { delay(1); continue; }
    char c = stream->read();
    keyMatchLen = (c == key[keyMatchLen]) ? keyMatchLen + 1 : (c == key[0] ? 1 : 0);
    if (keyMatchLen == (int)strlen(key)) break;
  }
  while (millis() < deadline) {
    if (!stream->available()) { delay(1); continue; }
    if (stream->read() == '[') break;
  }

  AircraftInfo best;
  float bestDist = 1e9f;
  float bestLat = 0, bestLon = 0;

  while (millis() < deadline) {
    while (stream->available() &&
           (stream->peek() == ',' || stream->peek() == ' ' ||
            stream->peek() == '\n' || stream->peek() == '\r')) {
      stream->read();
    }
    if (!stream->available()) { delay(1); continue; }
    if (stream->peek() == ']') break;

    StaticJsonDocument<512> s;
    if (deserializeJson(s, *stream)) break;
    if (s[5].isNull() || s[6].isNull()) continue;

    float d = distanceNM(HOME_LAT, HOME_LON, s[6].as<float>(), s[5].as<float>());
    if (d < bestDist) {
      bestDist = d;
      best.found = true;
      bestLat = copyJsonFloat(s, 6);
      bestLon = copyJsonFloat(s, 5);

      copyJsonField(s,  0, "N/A",  best.icao,     sizeof(best.icao));
      copyJsonField(s,  1, "N/A",  best.call,     sizeof(best.call));
      copyJsonField(s, 14, "----", best.squawk,   sizeof(best.squawk));

      best.onGround = s[8] | false;
      float altM  = copyJsonFloat(s, 7); best.altFt  = altM  >= 0 ? altM  * 3.28084f : -1;
      float velMs = copyJsonFloat(s, 9); best.spdKts = velMs >= 0 ? velMs * 1.94384f : -1;
      best.track  = copyJsonFloat(s, 10);
      best.vrFpm  = (s[11] | 0.0f) * 196.85f;
    }
  }
  http.end();

  best.dist = bestDist;

  if (best.found) {
    best.bearing = bearingTo(HOME_LAT, HOME_LON, bestLat, bestLon);
    applyFlightAware(best);
  }

  if (!best.found) {
    Serial.println("[ADSB] No aircraft in range");
  } else {
    Serial.println("+-----------------------------------------+");
    Serial.println("|       CLOSEST AIRCRAFT REPORT          |");
    Serial.println("+-----------------------------------------+");
    Serial.printf( "| ICAO24   : %-28s|\n", best.icao);
    Serial.printf( "| Callsign : %-28s|\n", best.call);
    Serial.printf( "| Type     : %-28s|\n", best.typecode);
    Serial.printf( "| Reg      : %-28s|\n", best.registration);
    Serial.printf( "| Route    : %s > %s\n",
                    best.depIcao, best.arrIcao);
    Serial.printf( "| Distance : %.1f nm %-3s (bearing %.0f deg)  |\n",
                    best.dist, cardinal(best.bearing), best.bearing);
    if (best.onGround)        Serial.println("| Altitude : ON GROUND                   |");
    else if (best.altFt >= 0) Serial.printf( "| Altitude : %.0f ft                      |\n", best.altFt);
    if (best.spdKts >= 0)     Serial.printf( "| Speed    : %.1f kts                     |\n", best.spdKts);
    if (best.track  >= 0)     Serial.printf( "| Track    : %.1f deg                     |\n", best.track);
    Serial.printf(            "| Vrt rate : %+.0f fpm                    |\n", best.vrFpm);
    Serial.printf(            "| Squawk   : %-28s|\n", best.squawk);
    Serial.println(           "+-----------------------------------------+");
  }

  g_lastUpdateMillis = millis();
  drawAircraftScreen(best);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(EPD_SCK, -1, EPD_MOSI, EPD_CS);
  display.init(115200);
  drawConnectingScreen();

  g_secureClient.setInsecure();
  g_secureClient.setTimeout(15);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.setSleep(false);
  connectWiFi();

  configTzTime(NTP_TIMEZONE, NTP_SERVER_1, NTP_SERVER_2);

  fetchMetar();
  fetchClosestAircraft();
}

void loop() {
  static unsigned long lastPoll = millis();
  if (millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();
    fetchMetar();
    fetchClosestAircraft();
  }
  delay(10);
}
