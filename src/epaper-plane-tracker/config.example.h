// ============================== CONFIG ==============================

// --- Wi-Fi ---
const char* WIFI_SSID     = "your-wifi-ssd";
const char* WIFI_PASSWORD = "your-wifi-password";

// --- OpenSky API ---
const char* OPENSKY_CLIENT_ID     = "your-opensky-client-id";
const char* OPENSKY_CLIENT_SECRET = "your-opensky-client-secret";
const char* OPENSKY_TOKEN_URL = "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token";
const char* OPENSKY_API_HOST  = "opensky-network.org";

// --- FlightAware AeroAPI (type / registration / route lookup) ---
const char* FLIGHTAWARE_API_HOST = "aeroapi.flightaware.com";
const char* FLIGHTAWARE_API_KEY  = "your-opensky-api-key";

// --- Location / search box ---
const float HOME_LAT       =  40.1098f;
const float HOME_LON       = -88.2170f;
const float SEARCH_BOX_DEG = 0.50f; // half-width of the bounding box (~30 nm)

// --- Timing ---
const unsigned long POLL_INTERVAL_MS = 30000UL; // polling rate
const int           FULL_REFRESH_EVERY = 10;    // full e-paper refresh every N partials

// --- Time / NTP ---
const char* NTP_TIMEZONE = "CET-1CEST,M3.5.0/2,M10.5.0/3"; // Europe/Rome, CET/CEST DST
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.google.com";

// --- METAR ---
// ICAO code of the airport whose METAR is displayed in the bottom bar.
const char* METAR_AIRPORT = "LIMC";