/**
 * @file main.c
 * @brief Finds the closest aircraft from a dump1090 server and displays it graphically.
 *
 * This program uses SDL2 to create a graphical window. It connects to a dump1090 
 * server, fetches aircraft data, finds the closest one, and looks up details 
 * from an online API. It renders all information as text and a graphical compass
 * in the window and plays an audible alert if an aircraft comes within a 5km radius.
 *
 * Configuration is loaded from `location.conf`.
 *
 * Dependencies:
 * - libcurl, cJSON
 * - SDL2, SDL2_ttf, SDL2_mixer
 *
 * Compilation (using the provided Makefile):
 * Place `PressStart2P-Regular.ttf` in the same directory and run `make`.
 *
 * Usage:
 * ./find_closest_plane
 * (Press Esc to exit)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <curl/curl.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_mixer.h>

#include <cjson/cJSON.h>
#include "font_data.h" // Embedded font from xxd

// --- Configuration ---
#define WINDOW_WIDTH 1024
#define FONT_SIZE 20
#define EARTH_RADIUS_KM 6371.0
#define REFRESH_INTERVAL_SECONDS 5
#define PROXIMITY_ALERT_KM 5.0

// --- Structs ---
struct MemoryStruct {
    char *memory;
    size_t size;
};

struct Aircraft {
    char flight[24], hex[10], squawk[6], registration[24], aircraft_type[24], operator[40];
    double lat, lon, distance_km;
    int altitude_ft, vert_rate_fpm;
    double ground_speed_kts, track_deg;
    double bearing_deg; // Bearing from user to aircraft
};

// --- Globals ---
SDL_Window* g_window = NULL;
SDL_Renderer* g_renderer = NULL;
TTF_Font* g_font = NULL;
Mix_Chunk* g_alert_sound = NULL;
bool g_audio_available = false;
struct Aircraft g_closest_plane;
// Configuration globals
char g_server_ip[40];
double g_user_lat;
double g_user_lon;


// --- Function Prototypes ---
// (Function definitions are below main)
void load_config();
bool init_sdl();
void close_sdl();
void render_text(const char* text, int x, int y, SDL_Color color);
void render_compass(int center_x, int center_y, double bearing);
Mix_Chunk* create_beep(int freq, int duration_ms);
static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp);
double deg2rad(double deg);
double haversine_distance(double lat1, double lon1, double lat2, double lon2);
double calculate_bearing(double lat1, double lon1, double lat2, double lon2);
const char* track_to_direction(double track_deg);
const char* get_squawk_description(const char* squawk);
void fetch_and_process_data();


int main(void) {
    load_config();

    if (!init_sdl()) {
        fprintf(stderr, "Failed to initialize SDL components!\n");
        return 1;
    }
    
    int window_w, window_h;
    SDL_GetWindowSize(g_window, &window_w, &window_h);


    // Initialize with default values
    snprintf(g_closest_plane.flight, sizeof(g_closest_plane.flight), "Waiting for data...");
    snprintf(g_closest_plane.operator, sizeof(g_closest_plane.operator), " ");
    snprintf(g_closest_plane.registration, sizeof(g_closest_plane.registration), " ");
    snprintf(g_closest_plane.aircraft_type, sizeof(g_closest_plane.aircraft_type), " ");
    snprintf(g_closest_plane.hex, sizeof(g_closest_plane.hex), " ");
    snprintf(g_closest_plane.squawk, sizeof(g_closest_plane.squawk), " ");
    g_closest_plane.distance_km = 999999.9;
    g_closest_plane.bearing_deg = 0.0;

    bool running = true;
    SDL_Event event;
    Uint32 last_update_time = 0;
    bool proximity_alert_triggered = false;

    while (running) {
        // --- Event Handling ---
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
            }
        }

        // --- Data Fetching Timer ---
        if (SDL_GetTicks() - last_update_time > REFRESH_INTERVAL_SECONDS * 1000) {
            fetch_and_process_data();
            last_update_time = SDL_GetTicks();
        }

        // --- Proximity Alert Logic ---
        if (g_closest_plane.distance_km < PROXIMITY_ALERT_KM) {
            if (!proximity_alert_triggered && g_alert_sound) {
                Mix_PlayChannel(-1, g_alert_sound, 0);
                proximity_alert_triggered = true;
            }
        } else {
            proximity_alert_triggered = false;
        }

        // --- Rendering ---
        SDL_SetRenderDrawColor(g_renderer, 10, 20, 40, 255); // Dark blue background
        SDL_RenderClear(g_renderer);
        
        // Prepare colors
        SDL_Color white = {255, 255, 255, 255};
        SDL_Color yellow = {255, 255, 0, 255};
        SDL_Color red = {255, 0, 0, 255};
        SDL_Color cyan = {0, 255, 255, 255};

        char buffer[256];
        int y_pos = 10;
        
        render_text("--- Closest Aircraft Monitor ---", 10, y_pos, yellow); y_pos += 40;

        if (proximity_alert_triggered) {
            render_text("!!! PROXIMITY ALERT !!!", 10, y_pos, red); y_pos += 30;
        }

        snprintf(buffer, sizeof(buffer), "Flight:       %s", g_closest_plane.flight);
        render_text(buffer, 10, y_pos, white); y_pos += 25;
        
        snprintf(buffer, sizeof(buffer), "Operator:     %s", g_closest_plane.operator);
        render_text(buffer, 10, y_pos, white); y_pos += 25;

        snprintf(buffer, sizeof(buffer), "Registration: %s", g_closest_plane.registration);
        render_text(buffer, 10, y_pos, white); y_pos += 25;
        
        snprintf(buffer, sizeof(buffer), "Type:         %s", g_closest_plane.aircraft_type);
        render_text(buffer, 10, y_pos, white); y_pos += 25;

        snprintf(buffer, sizeof(buffer), "Hex:          %s", g_closest_plane.hex);
        render_text(buffer, 10, y_pos, white); y_pos += 40;

        snprintf(buffer, sizeof(buffer), "Squawk:       %s (%s)", g_closest_plane.squawk, get_squawk_description(g_closest_plane.squawk));
        render_text(buffer, 10, y_pos, cyan); y_pos += 25;

        snprintf(buffer, sizeof(buffer), "Distance:     %.2f km", g_closest_plane.distance_km);
        render_text(buffer, 10, y_pos, cyan); y_pos += 25;

        snprintf(buffer, sizeof(buffer), "Location:     %.4f, %.4f", g_closest_plane.lat, g_closest_plane.lon);
        render_text(buffer, 10, y_pos, white); y_pos += 40;

        snprintf(buffer, sizeof(buffer), "Altitude:     %d ft", g_closest_plane.altitude_ft);
        render_text(buffer, 10, y_pos, white); y_pos += 25;
        
        snprintf(buffer, sizeof(buffer), "Vert. Rate:   %d fpm", g_closest_plane.vert_rate_fpm);
        render_text(buffer, 10, y_pos, white); y_pos += 25;

        snprintf(buffer, sizeof(buffer), "Speed:        %.0f kts", g_closest_plane.ground_speed_kts);
        render_text(buffer, 10, y_pos, white); y_pos += 25;

        snprintf(buffer, sizeof(buffer), "Track:        %.0f deg (%s)", g_closest_plane.track_deg, track_to_direction(g_closest_plane.track_deg));
        render_text(buffer, 10, y_pos, white); y_pos += 25;

        // Render the compass indicator
        render_compass(window_w - 150, 150, g_closest_plane.bearing_deg);

        SDL_RenderPresent(g_renderer);
    }

    close_sdl();
    return 0;
}


// --- Function Definitions ---

/**
 * @brief Loads server IP and location from `location.conf`. Falls back to defaults.
 */
void load_config() {
    // Set default (dummy) values first
    strcpy(g_server_ip, "127.0.0.1"); // Safe default
    g_user_lat = 51.5074; // London
    g_user_lon = -0.1278;

    FILE* file = fopen("location.conf", "r");
    if (!file) {
        printf("INFO: location.conf not found. Using default values.\n");
        return;
    }

    char line[128];
    while (fgets(line, sizeof(line), file)) {
        char* key = strtok(line, "=");
        char* value = strtok(NULL, "\n");
        if (key && value) {
            if (strcmp(key, "server_ip") == 0) {
                strncpy(g_server_ip, value, sizeof(g_server_ip) - 1);
            } else if (strcmp(key, "lat") == 0) {
                g_user_lat = atof(value);
            } else if (strcmp(key, "lon") == 0) {
                g_user_lon = atof(value);
            }
        }
    }
    fclose(file);
    printf("INFO: Loaded settings from location.conf\n");
}


/**
 * @brief Initializes SDL, TTF, Mixer, creates a window, renderer, and loads resources.
 */
bool init_sdl() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return false;
    }

    if (TTF_Init() == -1) {
        SDL_Log("TTF_Init failed: %s", TTF_GetError());
        SDL_Quit();
        return false;
    }

    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
        SDL_Log("Mix_OpenAudio failed: %s", Mix_GetError());
        g_audio_available = false;
    } else {
        g_audio_available = true;
    }

    g_window = SDL_CreateWindow("Closest Aircraft Finder", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 0, 0, SDL_WINDOW_FULLSCREEN_DESKTOP);
    if (!g_window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        if (g_audio_available) {
            Mix_CloseAudio();
            Mix_Quit();
        }
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    g_renderer = SDL_CreateRenderer(g_window, -1, SDL_RENDERER_ACCELERATED);
    if (!g_renderer) {
        SDL_Log("SDL_CreateRenderer failed: %s", SDL_GetError());
        SDL_DestroyWindow(g_window);
        if (g_audio_available) {
            Mix_CloseAudio();
            Mix_Quit();
        }
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    // Load font from embedded memory
    SDL_RWops* rw = SDL_RWFromConstMem(PressStart2P_Regular_ttf, PressStart2P_Regular_ttf_len);
    g_font = TTF_OpenFontRW(rw, 1, FONT_SIZE); // 1 to close the stream after
    if (!g_font) {
        SDL_Log("TTF_OpenFontRW failed: %s", TTF_GetError());
        SDL_DestroyRenderer(g_renderer);
        SDL_DestroyWindow(g_window);
        if (g_audio_available) {
            Mix_CloseAudio();
            Mix_Quit();
        }
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    if (g_audio_available) {
        // Create synthesized beep sound
        g_alert_sound = create_beep(880, 500); // 880Hz (A5 note) for 500ms
        if (!g_alert_sound) {
            SDL_Log("Failed to create alert sound");
        }
    }

    return true;
}

/**
 * @brief Cleans up all SDL resources.
 */
void close_sdl() {
    if (g_alert_sound) {
        Mix_FreeChunk(g_alert_sound);
        g_alert_sound = NULL;
    }
    if (g_font) {
        TTF_CloseFont(g_font);
        g_font = NULL;
    }
    if (g_renderer) {
        SDL_DestroyRenderer(g_renderer);
        g_renderer = NULL;
    }
    if (g_window) {
        SDL_DestroyWindow(g_window);
        g_window = NULL;
    }
    if (g_audio_available) {
        Mix_CloseAudio();
        Mix_Quit();
        g_audio_available = false;
    }
    TTF_Quit();
    SDL_Quit();
}

/**
 * @brief Renders a line of text to the screen at a given position and color.
 */
void render_text(const char* text, int x, int y, SDL_Color color) {
    if (!text || strlen(text) == 0) return;
    SDL_Surface* text_surface = TTF_RenderText_Blended(g_font, text, color);
    if (text_surface) {
        SDL_Texture* text_texture = SDL_CreateTextureFromSurface(g_renderer, text_surface);
        if (text_texture) {
            SDL_Rect dest_rect = { x, y, text_surface->w, text_surface->h };
            SDL_RenderCopy(g_renderer, text_texture, NULL, &dest_rect);
            SDL_DestroyTexture(text_texture);
        }
        SDL_FreeSurface(text_surface);
    }
}

/**
 * @brief Renders a compass with a fixed North arrow and a rotating bearing arrow.
 */
void render_compass(int center_x, int center_y, double bearing) {
    int radius = 60;
    SDL_Color white = {255, 255, 255, 255};
    SDL_Color cyan = {0, 255, 255, 255};

    // --- Draw North Arrow (fixed) ---
    SDL_SetRenderDrawColor(g_renderer, white.r, white.g, white.b, white.a);
    SDL_RenderDrawLine(g_renderer, center_x, center_y, center_x, center_y - radius);
    render_text("N", center_x - 8, center_y - radius - 25, white);

    // --- Draw Aircraft Bearing Arrow (rotated) ---
    double angle_rad = deg2rad(bearing);
    int end_x = center_x + (int)(radius * sin(angle_rad));
    int end_y = center_y - (int)(radius * cos(angle_rad));
    
    SDL_SetRenderDrawColor(g_renderer, cyan.r, cyan.g, cyan.b, cyan.a);
    // Draw the main line of the arrow
    SDL_RenderDrawLine(g_renderer, center_x, center_y, end_x, end_y);
    
    // Draw the arrowhead
    int arrow_len = 15;
    double arrow_angle_rad = deg2rad(25); // Angle of the arrowhead wings

    int arrow_x1 = end_x - (int)(arrow_len * sin(angle_rad - arrow_angle_rad));
    int arrow_y1 = end_y + (int)(arrow_len * cos(angle_rad - arrow_angle_rad));
    SDL_RenderDrawLine(g_renderer, end_x, end_y, arrow_x1, arrow_y1);
    
    int arrow_x2 = end_x - (int)(arrow_len * sin(angle_rad + arrow_angle_rad));
    int arrow_y2 = end_y + (int)(arrow_len * cos(angle_rad + arrow_angle_rad));
    SDL_RenderDrawLine(g_renderer, end_x, end_y, arrow_x2, arrow_y2);
}


/**
 * @brief Creates a simple sine wave beep sound and returns it as an SDL_mixer Chunk.
 */
Mix_Chunk* create_beep(int freq, int duration_ms) {
    int sample_rate = 44100;
    int num_samples = (duration_ms * sample_rate) / 1000;
    int buffer_size = num_samples * sizeof(Sint16);
    Sint16* buffer = (Sint16*)malloc(buffer_size);

    if (!buffer) return NULL;

    double volume = 4000;
    for (int i = 0; i < num_samples; i++) {
        buffer[i] = (Sint16)(volume * sin(2.0 * M_PI * freq * i / sample_rate));
    }

    SDL_RWops* rw = SDL_RWFromMem(buffer, buffer_size);
    if (!rw) {
        free(buffer);
        return NULL;
    }

    Mix_Chunk* chunk = Mix_LoadWAV_RW(rw, 1); // 1 => SDL frees rw
    free(buffer); // Mix_LoadWAV_RW copies data, so we can free the buffer

    if (chunk) {
        chunk->volume = MIX_MAX_VOLUME / 4;
    }

    return chunk;
}

/**
 * @brief Fetches data from dump1090 and the ADSB API, then updates the global g_closest_plane struct.
 */
void fetch_and_process_data() {
    char dump1090_url[256];
    snprintf(dump1090_url, sizeof(dump1090_url), "http://%s:8080/dump1090-fa/data/aircraft.json", g_server_ip);

    CURL *curl_handle = curl_easy_init();
    if (!curl_handle) return;
    
    struct MemoryStruct chunk = { .memory = malloc(1), .size = 0 };
    if (!chunk.memory) {
        curl_easy_cleanup(curl_handle);
        return;
    }
    curl_easy_setopt(curl_handle, CURLOPT_URL, dump1090_url);
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)&chunk);
    curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 10L);
    CURLcode res = curl_easy_perform(curl_handle);

    if (res == CURLE_OK && chunk.size > 0) {
        cJSON *root = cJSON_Parse(chunk.memory);
        if (root) {
             cJSON *aircraft_array = cJSON_GetObjectItemCaseSensitive(root, "aircraft");
             if (cJSON_IsArray(aircraft_array)) {
                struct Aircraft local_closest = { .distance_km = 999999.9 };
                bool plane_found = false;
                
                cJSON* aircraft_json;
                cJSON_ArrayForEach(aircraft_json, aircraft_array) {
                    cJSON *lat_json = cJSON_GetObjectItemCaseSensitive(aircraft_json, "lat");
                    cJSON *lon_json = cJSON_GetObjectItemCaseSensitive(aircraft_json, "lon");
                    if (cJSON_IsNumber(lat_json) && cJSON_IsNumber(lon_json)) {
                         double dist = haversine_distance(g_user_lat, g_user_lon, lat_json->valuedouble, lon_json->valuedouble);
                         if (dist < local_closest.distance_km) {
                            plane_found = true;
                            local_closest.distance_km = dist;
                            local_closest.lat = lat_json->valuedouble;
                            local_closest.lon = lon_json->valuedouble;
                            local_closest.bearing_deg = calculate_bearing(g_user_lat, g_user_lon, local_closest.lat, local_closest.lon);
                            
                            // Extract all other data from dump1090 json
                            cJSON *item;
                            item = cJSON_GetObjectItemCaseSensitive(aircraft_json, "flight");
                            if (item && item->valuestring) snprintf(local_closest.flight, sizeof(local_closest.flight), "%s", item->valuestring); else snprintf(local_closest.flight, sizeof(local_closest.flight), "N/A");
                            item = cJSON_GetObjectItemCaseSensitive(aircraft_json, "hex");
                            if (item && item->valuestring) snprintf(local_closest.hex, sizeof(local_closest.hex), "%s", item->valuestring); else snprintf(local_closest.hex, sizeof(local_closest.hex), "N/A");
                            item = cJSON_GetObjectItemCaseSensitive(aircraft_json, "squawk");
                            if (item && item->valuestring) snprintf(local_closest.squawk, sizeof(local_closest.squawk), "%s", item->valuestring); else snprintf(local_closest.squawk, sizeof(local_closest.squawk), "N/A");
                            item = cJSON_GetObjectItemCaseSensitive(aircraft_json, "alt_baro");
                            if (item) local_closest.altitude_ft = item->valueint; else local_closest.altitude_ft = 0;
                            item = cJSON_GetObjectItemCaseSensitive(aircraft_json, "gs");
                            if (item) local_closest.ground_speed_kts = item->valuedouble; else local_closest.ground_speed_kts = 0;
                            item = cJSON_GetObjectItemCaseSensitive(aircraft_json, "track");
                            if (item) local_closest.track_deg = item->valuedouble; else local_closest.track_deg = 0;
                            item = cJSON_GetObjectItemCaseSensitive(aircraft_json, "baro_rate");
                            if (item) local_closest.vert_rate_fpm = item->valueint; else local_closest.vert_rate_fpm = 0;
                         }
                    }
                }
                
                if (plane_found) {
                     g_closest_plane = local_closest; // Copy basic data over

                    // Set default values for API fields before the lookup
                    snprintf(g_closest_plane.registration, sizeof(g_closest_plane.registration), "N/A");
                    snprintf(g_closest_plane.aircraft_type, sizeof(g_closest_plane.aircraft_type), "N/A");
                    snprintf(g_closest_plane.operator, sizeof(g_closest_plane.operator), "N/A");

                    // Now fetch API data for the confirmed closest plane
                    char api_url[256];
                    snprintf(api_url, sizeof(api_url), "https://api.adsb.lol/v2/hex/%s", g_closest_plane.hex);
                    struct MemoryStruct api_chunk = { .memory = malloc(1), .size = 0 };
                    if (api_chunk.memory) {
                        curl_easy_setopt(curl_handle, CURLOPT_URL, api_url);
                        curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)&api_chunk);
                        if(curl_easy_perform(curl_handle) == CURLE_OK && api_chunk.size > 0) {
                            cJSON *api_root = cJSON_Parse(api_chunk.memory);
                            if(api_root) {
                                cJSON *ac_array = cJSON_GetObjectItemCaseSensitive(api_root, "ac");
                                if (cJSON_IsArray(ac_array) && cJSON_GetArraySize(ac_array) > 0) {
                                    cJSON *ac_info = cJSON_GetArrayItem(ac_array, 0);
                                    cJSON *item;
                                    item = cJSON_GetObjectItemCaseSensitive(ac_info, "r");
                                    if (item && item->valuestring) snprintf(g_closest_plane.registration, sizeof(g_closest_plane.registration), "%s", item->valuestring);
                                    item = cJSON_GetObjectItemCaseSensitive(ac_info, "t");
                                    if (item && item->valuestring) snprintf(g_closest_plane.aircraft_type, sizeof(g_closest_plane.aircraft_type), "%s", item->valuestring);
                                    item = cJSON_GetObjectItemCaseSensitive(ac_info, "operator");
                                    if (item && cJSON_IsString(item) && item->valuestring && item->valuestring[0] != '\0') {
                                        snprintf(g_closest_plane.operator, sizeof(g_closest_plane.operator), "%s", item->valuestring);
                                    } else {
                                        // Fallback keys used by some APIs
                                        item = cJSON_GetObjectItemCaseSensitive(ac_info, "ownOp");
                                        if (item && cJSON_IsString(item) && item->valuestring && item->valuestring[0] != '\0') {
                                            snprintf(g_closest_plane.operator, sizeof(g_closest_plane.operator), "%s", item->valuestring);
                                        } else {
                                            item = cJSON_GetObjectItemCaseSensitive(ac_info, "op");
                                            if (item && cJSON_IsString(item) && item->valuestring && item->valuestring[0] != '\0') {
                                                snprintf(g_closest_plane.operator, sizeof(g_closest_plane.operator), "%s", item->valuestring);
                                            }
                                        }
                                    }
                                }
                                cJSON_Delete(api_root);
                            }
                        }
                        free(api_chunk.memory);
                    }
                } else {
                    // No planes detected, reset to default state
                    snprintf(g_closest_plane.flight, sizeof(g_closest_plane.flight), "No aircraft in range");
                    snprintf(g_closest_plane.operator, sizeof(g_closest_plane.operator), " ");
                    snprintf(g_closest_plane.registration, sizeof(g_closest_plane.registration), " ");
                    snprintf(g_closest_plane.aircraft_type, sizeof(g_closest_plane.aircraft_type), " ");
                    snprintf(g_closest_plane.hex, sizeof(g_closest_plane.hex), " ");
                    snprintf(g_closest_plane.squawk, sizeof(g_closest_plane.squawk), " ");
                    g_closest_plane.distance_km = 999999.9;
                }
             }
             cJSON_Delete(root);
        }
    }

    free(chunk.memory);
    curl_easy_cleanup(curl_handle);
}


// --- Utility Function Implementations ---

double deg2rad(double deg) { return (deg * M_PI / 180.0); }

static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    size_t realsize = size * nmemb;
    struct MemoryStruct *mem = (struct MemoryStruct *)userp;
    char *ptr = realloc(mem->memory, mem->size + realsize + 1);
    if (ptr == NULL) return 0;
    mem->memory = ptr;
    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;
    return realsize;
}

double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = deg2rad(lat2 - lat1);
    double dLon = deg2rad(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_KM * c;
}

/**
 * @brief Calculates the initial bearing from point 1 to point 2.
 * @return The bearing in degrees (0-360).
 */
double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double lon_diff = deg2rad(lon2 - lon1);
    lat1 = deg2rad(lat1);
    lat2 = deg2rad(lat2);
    double y = sin(lon_diff) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon_diff);
    double bearing = atan2(y, x);
    bearing = fmod((bearing * 180.0 / M_PI + 360.0), 360.0); // Convert to degrees and normalize
    return bearing;
}


const char* track_to_direction(double track_deg) {
    static const char *directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    int index = (int)((track_deg / 22.5) + 0.5) % 16;
    return directions[index];
}

const char* get_squawk_description(const char* squawk) {
    if (strcmp(squawk, "7700") == 0) return "General Emergency";
    if (strcmp(squawk, "7600") == 0) return "Radio Failure";
    if (strcmp(squawk, "7500") == 0) return "Hijacking";
    if (strcmp(squawk, "7000") == 0) return "VFR Conspicuity";
    return "Discrete Code";
}

