// ntsc_mod.cpp
// Analog 5.8 GHz NTSC FM transmitter. Composite levels, CCIR-405 pre-emphasis,
// and SMPTE 170M line timing matching quadrf-ntsc-demod.
// Uses a 4-field (1050-line, 2-frame) sequence to guarantee continuous subcarrier phase (0.0° jump).

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include <iostream>
#include <strings.h>
#include <string>
#include <vector>

static std::atomic<bool> g_run{true};
static void on_sig(int) { g_run.store(false, std::memory_order_release); }

static bool has_arg(int argc, char** argv, const char* key) {
    for (int i = 1; i < argc; i++) if (std::string(argv[i]) == key) return true;
    return false;
}
static std::string get_str(int argc, char** argv, const char* key, const std::string& def) {
    for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == key) return argv[i + 1];
    return def;
}
static double get_dbl(int argc, char** argv, const char* key, double def) {
    for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == key) return std::stod(argv[i + 1]);
    return def;
}
static long long get_i64(int argc, char** argv, const char* key, long long def) {
    for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == key) return std::stoll(argv[i + 1]);
    return def;
}

static constexpr double FSC = 3579545.0;
static constexpr double FS_IQ_DEFAULT = 8.0 * FSC;
static constexpr int PIC_W = 640;
static constexpr int PIC_H = 480;
static constexpr int LINES_FRAME = 525;
static constexpr int TOTAL_LINES = 1050; // 2 frames (4 fields) = 238,875 integer subcarrier cycles
static constexpr float IRE_SYNC = -40.0f;
static constexpr float IRE_BLANK = 0.0f;
static constexpr float IRE_BLACK = 7.5f;
static constexpr float IRE_WHITE = 100.0f;
static constexpr float IRE_BURST = 20.0f;

struct RaceCh {
    const char* name;
    double mhz;
};
static const RaceCh kRace[] = {
    {"R1", 5658}, {"R2", 5695}, {"R3", 5732}, {"R4", 5769},
    {"R5", 5806}, {"R6", 5843}, {"R7", 5880}, {"R8", 5917},
};

static double channel_hz(const std::string& s) {
    for (const auto& c : kRace) {
        if (strcasecmp(s.c_str(), c.name) == 0) return c.mhz * 1e6;
    }
    return 0.0;
}

static void usage() {
    std::cerr
        << "Usage: quadrf-ntsc-mod [options]\n"
        << "Transmit analog NTSC (FM) on 5.8 GHz, matching quadrf-ntsc-demod.\n"
        << "Options:\n"
        << "  --pattern <name>     bars|smpte|grid|checker|white|black|red|green|blue (default: bars)\n"
        << "  --input <path>       image (still) or video/mp4; - reads raw rgb24 640x480 from stdin\n"
        << "  --still              treat --input as a single frame and loop it\n"
        << "  --ch <R1..R8>        Raceband channel (default: R5 = 5806 MHz)\n"
        << "  --freq <Hz>          TX LO (overrides --ch)\n"
        << "  --gain <0..25>       TX gain (default: 20; strictly <= 25 for safe transmission)\n"
        << "  --rate <sps>         IQ sample rate (default: 28.63636e6 = 8*fSC)\n"
        << "  --dev_hz <Hz>        FM deviation at 100 IRE vs blank (default: 3.5e6)\n"
        << "  --amp <0..1>         CS8 amplitude (default: 0.80)\n"
        << "  --bw <Hz>            analog TX bandwidth, 20e6 or 40e6 (default: 20e6)\n"
        << "  --no_color           luma only (no burst / chroma)\n"
        << "  --no_preemph         disable standard CCIR-405-1 pre-emphasis\n"
        << "  --dc_ire <IRE>       skip composite; FM a constant IRE (deviation probe)\n"
        << "  --hue <deg>          chroma phase offset\n"
        << "  --callsign <id>      ID burned into the picture (default: CALLSIGN from\n"
        << "                       /etc/quadrf/quadrf.conf, factory value NOCALL)\n"
        << "  --duration <s>       stop after this many seconds (0 = run until signal)\n"
        << "  --driver <string>    Soapy driver key (default: mipi)\n"
        << "  --args <k=v,...>     extra Soapy device arguments\n"
        << "  --help               this text\n";
}

static bool looks_like_image(const std::string& p) {
    auto dot = p.find_last_of('.');
    if (dot == std::string::npos) return false;
    std::string e = p.substr(dot);
    for (char& c : e) c = (char)tolower((unsigned char)c);
    return e == ".png" || e == ".jpg" || e == ".jpeg" || e == ".bmp" ||
           e == ".ppm" || e == ".pgm" || e == ".webp" || e == ".tga" || e == ".gif";
}

static bool load_ppm(const std::string& path, std::vector<uint8_t>& rgb) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[8] = {};
    if (fscanf(f, "%7s", magic) != 1 || std::string(magic) != "P6") {
        fclose(f);
        return false;
    }
    int w = 0, h = 0, maxv = 0;
    int c;
    while ((c = fgetc(f)) == '#') {
        while ((c = fgetc(f)) != EOF && c != '\n') {}
    }
    if (c != EOF) ungetc(c, f);
    if (fscanf(f, "%d %d %d", &w, &h, &maxv) != 3 || w <= 0 || h <= 0 || maxv != 255) {
        fclose(f);
        return false;
    }
    fgetc(f);
    std::vector<uint8_t> src((size_t)w * (size_t)h * 3);
    if (fread(src.data(), 1, src.size(), f) != src.size()) {
        fclose(f);
        return false;
    }
    fclose(f);
    rgb.assign((size_t)PIC_W * PIC_H * 3, 0);
    for (int y = 0; y < PIC_H; y++) {
        int sy = y * h / PIC_H;
        for (int x = 0; x < PIC_W; x++) {
            int sx = x * w / PIC_W;
            memcpy(&rgb[(y * PIC_W + x) * 3], &src[(sy * w + sx) * 3], 3);
        }
    }
    return true;
}

static std::string shell_quote(const std::string& s) {
    std::string o = "'";
    for (char c : s) {
        if (c == '\'') o += "'\\''";
        else o += c;
    }
    o += "'";
    return o;
}

static FILE* ffmpeg_open(const std::string& path, bool still) {
    const char* vf =
        "'scale=640:480:force_original_aspect_ratio=decrease,"
        "pad=640:480:(ow-iw)/2:(oh-ih)/2:black'";
    char cmd[2048];
    if (still) {
        snprintf(cmd, sizeof(cmd),
                 "ffmpeg -nostdin -v error -i %s -vf %s -frames:v 1 -f rawvideo -pix_fmt rgb24 -",
                 shell_quote(path).c_str(), vf);
    } else {
        snprintf(cmd, sizeof(cmd),
                 "ffmpeg -nostdin -v error -stream_loop -1 -i %s -vf %s "
                 "-r 30000/1001 -f rawvideo -pix_fmt rgb24 -",
                 shell_quote(path).c_str(), vf);
    }
    FILE* p = popen(cmd, "r");
    if (!p) std::cerr << "ffmpeg popen failed: " << cmd << "\n";
    return p;
}

static bool ffmpeg_read_frame(FILE* p, std::vector<uint8_t>& rgb) {
    rgb.resize((size_t)PIC_W * PIC_H * 3);
    size_t n = fread(rgb.data(), 1, rgb.size(), p);
    return n == rgb.size();
}

static void put_px(std::vector<uint8_t>& rgb, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if ((unsigned)x >= PIC_W || (unsigned)y >= PIC_H) return;
    size_t i = ((size_t)y * PIC_W + (size_t)x) * 3;
    rgb[i] = r; rgb[i + 1] = g; rgb[i + 2] = b;
}

static void fill_rect(std::vector<uint8_t>& rgb, int x0, int y0, int x1, int y1,
                      uint8_t r, uint8_t g, uint8_t b) {
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    x1 = std::min(PIC_W, x1); y1 = std::min(PIC_H, y1);
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) put_px(rgb, x, y, r, g, b);
}

// 5x7 column-major glyphs (bit 0 = top). Covers callsign charset plus space.
static const uint8_t* glyph5x7(char ch) {
    static const uint8_t sp[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
    static const uint8_t digits[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E},
    };
    static const uint8_t letters[26][5] = {
        {0x7E, 0x11, 0x11, 0x11, 0x7E},
        {0x7F, 0x49, 0x49, 0x49, 0x36},
        {0x3E, 0x41, 0x41, 0x41, 0x22},
        {0x7F, 0x41, 0x41, 0x22, 0x1C},
        {0x7F, 0x49, 0x49, 0x49, 0x41},
        {0x7F, 0x09, 0x09, 0x09, 0x01},
        {0x3E, 0x41, 0x49, 0x49, 0x7A},
        {0x7F, 0x08, 0x08, 0x08, 0x7F},
        {0x00, 0x41, 0x7F, 0x41, 0x00},
        {0x20, 0x40, 0x41, 0x3F, 0x01},
        {0x7F, 0x08, 0x14, 0x22, 0x41},
        {0x7F, 0x40, 0x40, 0x40, 0x40},
        {0x7F, 0x02, 0x0C, 0x02, 0x7F},
        {0x7F, 0x04, 0x08, 0x10, 0x7F},
        {0x3E, 0x41, 0x41, 0x41, 0x3E},
        {0x7F, 0x09, 0x09, 0x09, 0x06},
        {0x3E, 0x41, 0x51, 0x21, 0x5E},
        {0x7F, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31},
        {0x01, 0x01, 0x7F, 0x01, 0x01},
        {0x3F, 0x40, 0x40, 0x40, 0x3F},
        {0x1F, 0x20, 0x40, 0x20, 0x1F},
        {0x7F, 0x20, 0x18, 0x20, 0x7F},
        {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x07, 0x08, 0x70, 0x08, 0x07},
        {0x61, 0x51, 0x49, 0x45, 0x43},
    };
    if (ch == ' ') return sp;
    if (ch == '-') return dash;
    if (ch == '/') return slash;
    if (ch >= '0' && ch <= '9') return digits[ch - '0'];
    if (ch >= 'A' && ch <= 'Z') return letters[ch - 'A'];
    return sp;
}

static std::string trim_copy(std::string s) {
    while (!s.empty() && std::isspace((unsigned char)s.back())) s.pop_back();
    size_t i = 0;
    while (i < s.size() && std::isspace((unsigned char)s[i])) i++;
    return s.substr(i);
}

static std::string unquote(std::string s) {
    s = trim_copy(std::move(s));
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                          (s.front() == '\'' && s.back() == '\'')))
        return s.substr(1, s.size() - 2);
    return s;
}

static std::string callsign_from_conf(const char* path) {
    FILE* f = std::fopen(path, "r");
    if (!f) return {};
    char line[256];
    std::string found;
    while (std::fgets(line, sizeof(line), f)) {
        const char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || *p == '\0') continue;
        if (std::strncmp(p, "CALLSIGN=", 9) == 0)
            found = unquote(p + 9);
        else if (std::strncmp(p, "QUADRF_CALLSIGN=", 16) == 0)
            found = unquote(p + 16);
    }
    std::fclose(f);
    return found;
}

// Same default as /etc/quadrf/quadrf.conf and the web UI: NOCALL.
static std::string resolve_callsign(const std::string& cli) {
    std::string raw = trim_copy(cli);
    if (raw.empty()) {
        if (const char* e = std::getenv("CALLSIGN")) raw = trim_copy(e);
        else if (const char* e = std::getenv("QUADRF_CALLSIGN")) raw = trim_copy(e);
        if (raw.empty()) {
            const char* path = std::getenv("QUADRF_CONF");
            raw = callsign_from_conf(path && *path ? path : "/etc/quadrf/quadrf.conf");
        }
    }
    for (char& c : raw) c = (char)std::toupper((unsigned char)c);
    if (raw.empty()) raw = "NOCALL";
    if (raw.size() > 16) raw.resize(16);
    return raw;
}

static void overlay_callsign(std::vector<uint8_t>& rgb, const std::string& text) {
    if (rgb.size() < (size_t)PIC_W * PIC_H * 3 || text.empty()) return;
    constexpr int kScale = 3; // 15x21 px glyphs; readable after analog FM
    constexpr int kPad = 4;
    const int tw = (int)text.size() * (5 + 1) * kScale - kScale;
    const int th = 7 * kScale;
    const int box_w = tw + kPad * 2;
    const int box_h = th + kPad * 2;
    const int x0 = PIC_W - 14 - box_w;
    const int y0 = PIC_H - 14 - box_h;
    fill_rect(rgb, x0, y0, x0 + box_w, y0 + box_h, 0, 0, 0);
    int cx = x0 + kPad;
    const int cy = y0 + kPad;
    for (char ch : text) {
        const uint8_t* g = glyph5x7(ch);
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if (!(g[col] & (1u << row))) continue;
                fill_rect(rgb,
                          cx + col * kScale, cy + row * kScale,
                          cx + (col + 1) * kScale, cy + (row + 1) * kScale,
                          255, 255, 255);
            }
        }
        cx += (5 + 1) * kScale;
    }
}

static void make_pattern(const std::string& name, std::vector<uint8_t>& rgb) {
    rgb.assign((size_t)PIC_W * PIC_H * 3, 0);
    auto solid = [&](uint8_t r, uint8_t g, uint8_t b) {
        fill_rect(rgb, 0, 0, PIC_W, PIC_H, r, g, b);
    };
    if (name == "white") { solid(255, 255, 255); return; }
    if (name == "black") { solid(0, 0, 0); return; }
    if (name == "red")   { solid(220, 0, 0); return; }
    if (name == "green") { solid(0, 200, 0); return; }
    if (name == "blue")  { solid(0, 0, 220); return; }

    if (name == "checker") {
        for (int y = 0; y < PIC_H; y++)
            for (int x = 0; x < PIC_W; x++) {
                bool on = ((x / 40) ^ (y / 40)) & 1;
                put_px(rgb, x, y, on ? 240 : 20, on ? 240 : 20, on ? 240 : 20);
            }
        return;
    }
    if (name == "grid") {
        solid(16, 16, 24);
        for (int x = 0; x < PIC_W; x += 32) fill_rect(rgb, x, 0, x + 2, PIC_H, 220, 220, 220);
        for (int y = 0; y < PIC_H; y += 32) fill_rect(rgb, 0, y, PIC_W, y + 2, 220, 220, 220);
        fill_rect(rgb, PIC_W / 2 - 2, 0, PIC_W / 2 + 2, PIC_H, 255, 40, 40);
        fill_rect(rgb, 0, PIC_H / 2 - 2, PIC_W, PIC_H / 2 + 2, 40, 255, 40);
        return;
    }

    // Standard SMPTE-75 color bars:
    // Gray, Yellow, Cyan, Green, Magenta, Red, Blue
    const uint8_t bars[][3] = {
        {191, 191, 191}, // 75% White/Gray
        {191, 191, 0},   // Yellow
        {0, 191, 191},   // Cyan
        {0, 191, 0},     // Green
        {191, 0, 191},   // Magenta
        {191, 0, 0},     // Red
        {0, 0, 191},     // Blue
    };
    int bar_h = (name == "smpte") ? PIC_H * 2 / 3 : PIC_H * 3 / 4;
    for (int x = 0; x < PIC_W; x++) {
        int b = x * 7 / PIC_W;
        if (b > 6) b = 6;
        for (int y = 0; y < bar_h; y++)
            put_px(rgb, x, y, bars[b][0], bars[b][1], bars[b][2]);
    }
    // Bottom: -I, white, +Q, black, plus RGB color squares
    fill_rect(rgb, 0, bar_h, PIC_W, PIC_H, 16, 16, 16);
    fill_rect(rgb, 0, bar_h, PIC_W / 4, PIC_H, 0, 68, 114);
    fill_rect(rgb, PIC_W / 4, bar_h, PIC_W / 2, PIC_H, 255, 255, 255);
    fill_rect(rgb, PIC_W / 2, bar_h, 3 * PIC_W / 4, PIC_H, 72, 0, 127);
    fill_rect(rgb, 3 * PIC_W / 4, bar_h, PIC_W, PIC_H, 0, 0, 0);
    int sq = 56;
    int y0 = bar_h + (PIC_H - bar_h - sq) / 2;
    fill_rect(rgb, 40, y0, 40 + sq, y0 + sq, 255, 32, 32);
    fill_rect(rgb, 120, y0, 120 + sq, y0 + sq, 32, 220, 32);
    fill_rect(rgb, 200, y0, 200 + sq, y0 + sq, 32, 32, 255);
    // Outer white frame border
    fill_rect(rgb, 0, 0, PIC_W, 6, 255, 255, 255);
    fill_rect(rgb, 0, PIC_H - 6, PIC_W, PIC_H, 255, 255, 255);
    fill_rect(rgb, 0, 0, 6, PIC_H, 255, 255, 255);
    fill_rect(rgb, PIC_W - 6, 0, PIC_W, PIC_H, 255, 255, 255);
}

enum class LineKind { Equalizing, Vsync, Blank, Active };

static void line_kind(int line, LineKind& kind, int& src_y) {
    src_y = 0;
    // Standard SMPTE 170M vertical interval across 525 lines:
    // Field 1: lines 0..261. Field 2: lines 262..524.
    if (line < 3) { kind = LineKind::Equalizing; return; }
    if (line < 6) { kind = LineKind::Vsync; return; }
    if (line < 9) { kind = LineKind::Equalizing; return; }
    if (line < 21) { kind = LineKind::Blank; return; }
    if (line < 262) {
        kind = LineKind::Active;
        src_y = (line - 21) * 2;
        return;
    }
    if (line == 262) { kind = LineKind::Blank; return; }
    if (line < 266) { kind = LineKind::Equalizing; return; }
    if (line < 269) { kind = LineKind::Vsync; return; }
    if (line < 272) { kind = LineKind::Equalizing; return; }
    if (line < 284) { kind = LineKind::Blank; return; }
    kind = LineKind::Active;
    src_y = (line - 284) * 2 + 1;
}

struct NtscGeom {
    int samp_line = 0;
    int sync_n = 0;
    int burst0 = 0;
    int burst1 = 0;
    int active0 = 0;
    int active1 = 0;
    int eq_n = 0;   // 2.3 µs equalizing pulse
    int vs_n = 0;   // 27.1 µs serrated broad pulse
    int half = 0;   // 2H slot (half line)
    float d_phi = 0.0f; // fSC phase step, rad / IQ sample

    explicit NtscGeom(double fs) {
        samp_line = (int)std::llround(fs / FSC * 227.5); // 1820 samples at 8*fSC
        sync_n    = (int)std::llround(fs * 4.7e-6);      // 4.7 µs sync pulse
        burst0    = sync_n + (int)std::llround(fs * 0.6e-6); // breezeway
        burst1    = burst0 + (int)std::llround(fs * 2.5e-6); // 9 cycles of 3.58 MHz
        active0   = sync_n + (int)std::llround(fs * 4.7e-6); // back porch end
        active1   = active0 + (int)std::llround(fs * 52.655e-6); // active video
        samp_line = std::max(samp_line, active1 + 8);
        eq_n      = (int)std::llround(fs * 2.3e-6);      // 2.3 µs equalizing pulse
        vs_n      = (int)std::llround(fs * 27.1e-6);     // 27.1 µs broad pulse
        half      = samp_line / 2;
        d_phi     = (float)(2.0 * M_PI * FSC / fs);
    }
};

static void rgb_at(const uint8_t* rgb, int x, int y, float& r, float& g, float& b) {
    x = std::clamp(x, 0, PIC_W - 1);
    y = std::clamp(y, 0, PIC_H - 1);
    const uint8_t* p = rgb + ((size_t)y * PIC_W + (size_t)x) * 3;
    r = p[0] * (1.0f / 255.0f);
    g = p[1] * (1.0f / 255.0f);
    b = p[2] * (1.0f / 255.0f);
}

static void encode_line(float* out, const NtscGeom& g, LineKind kind, int src_y,
                        const uint8_t* rgb, bool color, float hue_rad, float& fsc_phase) {
    const int n = g.samp_line;

    if (kind == LineKind::Equalizing || kind == LineKind::Vsync) {
        // EIA-170 / SMPTE 170M: two pulses per line at 2H (31.777 µs).
        // Equalizing = 2.3 µs; serrated v-sync = 27.1 µs.
        const int pw = (kind == LineKind::Vsync) ? g.vs_n : g.eq_n;
        for (int i = 0; i < n; i++) out[i] = IRE_BLANK;
        for (int i = 0; i < pw && i < n; i++) out[i] = IRE_SYNC;
        for (int i = 0; i < pw && g.half + i < n; i++) out[g.half + i] = IRE_SYNC;
        fsc_phase += (float)n * g.d_phi;
        fsc_phase = std::fmod(fsc_phase + 4.0f * (float)M_PI, 2.0f * (float)M_PI);
        return;
    }

    float c = std::cos(fsc_phase + hue_rad);
    float s = std::sin(fsc_phase + hue_rad);
    const float dc = std::cos(g.d_phi);
    const float ds = std::sin(g.d_phi);
    const int act_len = std::max(1, g.active1 - g.active0);

    for (int i = 0; i < n; i++) {
        float ire;
        if (i < g.sync_n) {
            ire = IRE_SYNC;
        } else if (color && i >= g.burst0 && i < g.burst1) {
            ire = IRE_BLANK - IRE_BURST * s; // -U burst, standard 180° reference phase
        } else if (kind == LineKind::Active && i >= g.active0 && i < g.active1 && rgb) {
            int x = (int)((long long)(i - g.active0) * PIC_W / act_len);
            float R, G, B;
            rgb_at(rgb, x, src_y, R, G, B);
            float Y = 0.299f * R + 0.587f * G + 0.114f * B;
            float U = 0.4921f * (B - Y);
            float V = 0.8773f * (R - Y);
            ire = IRE_BLACK + Y * (IRE_WHITE - IRE_BLACK);
            if (color) ire += 100.0f * (U * s + V * c);
        } else {
            ire = IRE_BLANK;
        }
        out[i] = ire;
        float nc = c * dc - s * ds;
        float ns = s * dc + c * ds;
        if ((i & 63) == 0) {
            float norm = 0.5f * (3.0f - (nc * nc + ns * ns));
            nc *= norm; ns *= norm;
        }
        c = nc; s = ns;
    }
    fsc_phase += (float)n * g.d_phi;
    fsc_phase = std::fmod(fsc_phase + 4.0f * (float)M_PI, 2.0f * (float)M_PI);
}

// 2nd-order Butterworth baseband lowpass filter at 4.2 MHz (SMPTE 170M standard bandwidth)
struct BasebandLowpass4M2 {
    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
    float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;

    BasebandLowpass4M2(double fs) {
        double K = std::tan(M_PI * 4.2e6 / fs);
        double K2 = K * K;
        double denom = 1.0 + std::sqrt(2.0) * K + K2;
        b0 = (float)(K2 / denom);
        b1 = (float)(2.0 * b0);
        b2 = b0;
        a1 = (float)(2.0 * (K2 - 1.0) / denom);
        a2 = (float)((1.0 - std::sqrt(2.0) * K + K2) / denom);
    }
    inline float step(float x) {
        float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
};

// CCIR-405-1 525-line standard pre-emphasis network
struct PreemphasisCCIR405 {
    float b0 = 1.0f, b1 = 0.0f, a1 = 0.0f;
    float x1 = 0.0f, y1 = 0.0f;

    PreemphasisCCIR405(double fs) {
        constexpr double tau_z = 0.85084e-6; 
        constexpr double tau_p = 0.18188e-6; 
        double alpha_z = 2.0 * tau_z * fs; 
        double alpha_p = 2.0 * tau_p * fs;
        b0 = (float)((1.0 + alpha_z) / (1.0 + alpha_p));
        b1 = (float)((1.0 - alpha_z) / (1.0 + alpha_p));
        a1 = (float)((1.0 - alpha_p) / (1.0 + alpha_p));
    }
    inline float step(float x) {
        float y = b0 * x + b1 * x1 - a1 * y1;
        x1 = x; y1 = y;
        return std::clamp(y, -45.0f, 135.0f);
    }
};

static SoapySDR::Kwargs parse_args(const std::string& extra, const std::string& driver) {
    SoapySDR::Kwargs args;
    if (!driver.empty()) args["driver"] = driver;
    size_t start = 0;
    while (start < extra.size()) {
        size_t comma = extra.find(',', start);
        std::string kv = extra.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
        size_t eq = kv.find('=');
        if (eq != std::string::npos) args[kv.substr(0, eq)] = kv.substr(eq + 1);
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return args;
}

int main(int argc, char** argv) {
    if (has_arg(argc, argv, "--help") || has_arg(argc, argv, "-h")) {
        usage();
        return 0;
    }

    std::signal(SIGINT, on_sig);
    std::signal(SIGTERM, on_sig);

    const std::string driver = get_str(argc, argv, "--driver", "mipi");
    const std::string extra = get_str(argc, argv, "--args", "");
    const std::string pattern = get_str(argc, argv, "--pattern", "bars");
    const std::string input = get_str(argc, argv, "--input", "");
    const std::string ch = get_str(argc, argv, "--ch", "R5");
    bool force_still = has_arg(argc, argv, "--still");
    bool no_color = has_arg(argc, argv, "--no_color");
    bool no_preemph = has_arg(argc, argv, "--no_preemph");
    bool dc_mode = has_arg(argc, argv, "--dc_ire");
    double dc_ire = get_dbl(argc, argv, "--dc_ire", 0.0);
    double freq = get_dbl(argc, argv, "--freq", 0.0);
    double rate = get_dbl(argc, argv, "--rate", FS_IQ_DEFAULT);
    double gain = get_dbl(argc, argv, "--gain", 20.0);
    double dev_hz = get_dbl(argc, argv, "--dev_hz", 3.5e6);
    double amp = get_dbl(argc, argv, "--amp", 0.80);
    double bw = get_dbl(argc, argv, "--bw", 20e6);
    double hue_deg = get_dbl(argc, argv, "--hue", 0.0);
    double duration = get_dbl(argc, argv, "--duration", 0.0);
    const std::string id_text = resolve_callsign(get_str(argc, argv, "--callsign", ""));

    if (freq <= 0.0) {
        freq = channel_hz(ch);
        if (freq <= 0.0) {
            std::cerr << "Unknown channel '" << ch << "'. Use R1..R8 or --freq.\n";
            return 2;
        }
    }
    if (gain < 0.0) gain = 0.0;
    if (gain > 25.0) {
        std::cerr << "warning: TX gain " << gain
                  << " > 25; strictly clamped to 25.0 max\n";
        gain = 25.0;
    }
    amp = std::clamp(amp, 0.05, 1.0);

    std::vector<uint8_t> rgb;
    FILE* ff = nullptr;
    bool video_live = false;

    if (dc_mode) {
        std::cerr << "DC IRE probe: " << dc_ire << " IRE (no composite)\n";
    } else if (!input.empty()) {
        bool still = force_still || (input != "-" && looks_like_image(input));
        if (input == "-") {
            rgb.resize((size_t)PIC_W * PIC_H * 3);
            if (fread(rgb.data(), 1, rgb.size(), stdin) != rgb.size()) {
                std::cerr << "failed to read one 640x480 rgb24 frame from stdin\n";
                return 1;
            }
            video_live = !still;
        } else if (still && input.size() >= 4 && input.substr(input.size() - 4) == ".ppm") {
            if (!load_ppm(input, rgb)) {
                std::cerr << "failed to load PPM " << input << "\n";
                return 1;
            }
        } else {
            ff = ffmpeg_open(input, still);
            if (!ff || !ffmpeg_read_frame(ff, rgb)) {
                std::cerr << "failed to decode " << input << " via ffmpeg\n";
                if (ff) pclose(ff);
                return 1;
            }
            if (still) {
                pclose(ff);
                ff = nullptr;
            } else {
                video_live = true;
            }
        }
        std::cerr << "Loaded " << input << (video_live ? " (looping video)\n" : " (still)\n");
        overlay_callsign(rgb, id_text);
    } else {
        make_pattern(pattern, rgb);
        overlay_callsign(rgb, id_text);
        std::cerr << "Pattern: " << pattern << "\n";
    }

    NtscGeom geom(rate);
    const size_t buffer_lines = dc_mode ? 1 : (size_t)TOTAL_LINES; // 1050 lines = 2 frames
    const size_t total_samps = buffer_lines * (size_t)geom.samp_line;
    std::vector<float> ire(dc_mode ? 1 : total_samps, (float)dc_ire);

    auto rebuild = [&](float& phase) {
        if (dc_mode) return;
        for (int line = 0; line < TOTAL_LINES; line++) {
            int frame_line = line % LINES_FRAME;
            LineKind k;
            int sy = 0;
            line_kind(frame_line, k, sy);
            encode_line(ire.data() + (size_t)line * (size_t)geom.samp_line,
                        geom, k, sy, rgb.data(), !no_color, (float)(hue_deg * M_PI / 180.0), phase);
        }
    };
    float fsc_phase = 0.0f;
    rebuild(fsc_phase);

    // Baseband lowpass filter to 4.2 MHz (SMPTE 170M standard bandwidth)
    // Eliminates infinite-slope pixel transitions and pre-emphasis overshoot spikes
    if (!dc_mode) {
        BasebandLowpass4M2 lpf(rate);
        for (size_t i = 0; i < total_samps; i++) {
            ire[i] = lpf.step(ire[i]);
        }
    }

    // Apply CCIR-405-1 pre-emphasis if requested
    if (!dc_mode && !no_preemph) {
        PreemphasisCCIR405 preemph(rate);
        for (size_t i = 0; i < total_samps; i++) {
            ire[i] = preemph.step(ire[i]);
        }
    }

    // 16-bit LUT: phase>>16 indexes cos/sin at the configured CS8 amplitude
    const float a = (float)(amp * 127.0);
    std::vector<int8_t> cos_lut(65536), sin_lut(65536);
    for (int i = 0; i < 65536; i++) {
        float th = (float)(2.0 * M_PI * (i + 0.5) / 65536.0);
        cos_lut[i] = (int8_t)std::lround(a * std::cos(th));
        sin_lut[i] = (int8_t)std::lround(a * std::sin(th));
    }
    const double phase_scale = 4294967296.0 * dev_hz / (100.0 * rate);

    // Pre-modulate into a circular CS8 buffer when transmitting static patterns or stills.
    // This reduces runtime CPU utilization from ~40% to <5%, keeping the Pi cool.
    std::vector<std::complex<int8_t>> premod_iq;
    if (!video_live && !dc_mode) {
        premod_iq.resize(total_samps);
        uint32_t fm_phase = 0;
        for (size_t i = 0; i < total_samps; i++) {
            double inc = (double)ire[i] * phase_scale;
            fm_phase += (uint32_t)(int32_t)inc;
            uint16_t p = (uint16_t)(fm_phase >> 16);
            premod_iq[i] = {cos_lut[p], sin_lut[p]};
        }
    }

    try {
        auto args = parse_args(extra, driver);
        SoapySDR::Device* dev = SoapySDR::Device::make(args);
        if (!dev) {
            std::cerr << "SoapySDR::Device::make failed\n";
            return 1;
        }

        dev->setFrequency(SOAPY_SDR_TX, 0, freq);
        dev->setSampleRate(SOAPY_SDR_TX, 0, rate);
        if (bw > 0.0) dev->setBandwidth(SOAPY_SDR_TX, 0, bw);
        dev->setGain(SOAPY_SDR_TX, 0, gain);
        try { dev->writeSetting("tx_antennas", "15"); } catch (...) {}
        try { dev->writeSetting("tx_enable", "true"); } catch (...) {}

        SoapySDR::Stream* stream = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS8, {0});
        if (!stream) {
            std::cerr << "setupStream TX failed\n";
            SoapySDR::Device::unmake(dev);
            return 1;
        }
        dev->activateStream(stream);

        const size_t mtu = std::max<size_t>(dev->getStreamMTU(stream), 4096);
        std::vector<std::complex<int8_t>> live_buf(mtu);

        std::cerr << "NTSC FM TX (SMPTE 170M Standard)\n"
                  << "  freq     = " << freq / 1e6 << " MHz\n"
                  << "  rate     = " << rate / 1e6 << " Msps\n"
                  << "  gain     = " << gain << " dB (safe <= 25 dB)\n"
                  << "  dev      = " << dev_hz / 1e6 << " MHz / 100 IRE\n"
                  << "  line     = " << geom.samp_line << " IQ samples\n"
                  << "  sequence = " << (dc_mode ? 1 : TOTAL_LINES) << " lines (4-field phase continuous)\n"
                  << "  preemph  = " << (no_preemph ? "off" : "CCIR-405-1") << "\n"
                  << "  color    = " << (no_color ? "off" : "on") << "\n"
                  << "  callsign = " << id_text << "\n"
                  << "  mode     = " << (premod_iq.empty() ? (dc_mode ? "dc_probe" : "live_stream") : "fast_dma_loop") << "\n";

        uint32_t fm_phase = 0;
        size_t stream_pos = 0;
        size_t idx = 0;
        uint64_t sent = 0;
        auto t0 = std::chrono::steady_clock::now();
        int frames_sent = 0;

        PreemphasisCCIR405 live_preemph(rate);
        BasebandLowpass4M2 live_lpf(rate);

        while (g_run.load(std::memory_order_relaxed)) {
            if (duration > 0.0) {
                double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                if (wall >= duration) break;
            }

            int flags = 0;
            int ret = 0;

            if (!premod_iq.empty()) {
                // High-efficiency zero-overhead DMA path for static patterns / stills
                size_t chunk = std::min(mtu, total_samps - stream_pos);
                const void* const buffs[1] = { &premod_iq[stream_pos] };
                ret = dev->writeStream(stream, buffs, chunk, flags, 0, 1000000);
                if (ret > 0) {
                    stream_pos += (size_t)ret;
                    if (stream_pos >= total_samps) {
                        stream_pos = 0;
                        frames_sent += 2;
                    }
                }
            } else {
                // Live streaming path for dynamic video
                for (size_t i = 0; i < mtu; i++) {
                    float ire_s;
                    if (dc_mode) {
                        ire_s = (float)dc_ire;
                    } else {
                        if (idx >= total_samps) {
                            idx = 0;
                            frames_sent++;
                            if (video_live) {
                                std::vector<uint8_t> next;
                                bool got = false;
                                if (ff) got = ffmpeg_read_frame(ff, next);
                                else if (input == "-") {
                                    next.resize((size_t)PIC_W * PIC_H * 3);
                                    got = fread(next.data(), 1, next.size(), stdin) == next.size();
                                }
                                if (got) {
                                    overlay_callsign(next, id_text);
                                    rgb.swap(next);
                                } else if (input == "-" && !ff) {
                                    g_run.store(false);
                                }
                                float ph = fsc_phase;
                                rebuild(ph);
                                fsc_phase = ph;
                            }
                        }
                        ire_s = ire[idx++];
                        if (video_live) {
                            ire_s = live_lpf.step(ire_s);
                            if (!no_preemph) {
                                ire_s = live_preemph.step(ire_s);
                            }
                        }
                    }
                    double inc = (double)ire_s * phase_scale;
                    fm_phase += (uint32_t)(int32_t)inc;
                    uint16_t p = (uint16_t)(fm_phase >> 16);
                    live_buf[i] = {cos_lut[p], sin_lut[p]};
                }
                const void* const buffs[1] = {live_buf.data()};
                ret = dev->writeStream(stream, buffs, mtu, flags, 0, 1000000);
            }

            if (ret == SOAPY_SDR_TIMEOUT) continue;
            if (ret < 0) {
                std::cerr << "writeStream error " << ret << "\n";
                break;
            }
            sent += (uint64_t)ret;
        }

        auto t1 = std::chrono::steady_clock::now();
        double sec = std::chrono::duration<double>(t1 - t0).count();
        std::cerr << "TX stop: " << sent << " samples, " << frames_sent
                  << " frames, " << (sec > 0 ? sent / sec / 1e6 : 0) << " Msps wall\n";

        try { dev->writeSetting("tx_enable", "false"); } catch (...) {}
        dev->deactivateStream(stream, 0, 0);
        dev->closeStream(stream);
        SoapySDR::Device::unmake(dev);
    } catch (const std::exception& ex) {
        std::cerr << "Exception: " << ex.what() << "\n";
        if (ff) pclose(ff);
        return 1;
    }
    if (ff) pclose(ff);
    return 0;
}
