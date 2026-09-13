-- FPV channel hop, saturation, and command help for Camera Decoder.
-- Default band is Race (R1–R8). Left/Right step band; Up/Down step channel.
local bands = {
    {
        name = "Race",
        ch = {
            {n = "R1", f = 5658},
            {n = "R2", f = 5695},
            {n = "R3", f = 5732},
            {n = "R4", f = 5769},
            {n = "R5", f = 5806},
            {n = "R6", f = 5843},
            {n = "R7", f = 5880},
            {n = "R8", f = 5917},
        },
    },
    {
        name = "A",
        ch = {
            {n = "A1", f = 5865},
            {n = "A2", f = 5845},
            {n = "A3", f = 5825},
            {n = "A4", f = 5805},
            {n = "A5", f = 5785},
            {n = "A6", f = 5765},
            {n = "A7", f = 5745},
            {n = "A8", f = 5725},
        },
    },
    {
        name = "B",
        ch = {
            {n = "B1", f = 5733},
            {n = "B2", f = 5752},
            {n = "B3", f = 5771},
            {n = "B4", f = 5790},
            {n = "B5", f = 5809},
            {n = "B6", f = 5828},
            {n = "B7", f = 5847},
            {n = "B8", f = 5866},
        },
    },
    {
        name = "E",
        ch = {
            {n = "E1", f = 5705},
            {n = "E2", f = 5685},
            {n = "E3", f = 5665},
            {n = "E4", f = 5645},
            {n = "E5", f = 5885},
            {n = "E6", f = 5905},
            {n = "E7", f = 5925},
            {n = "E8", f = 5945},
        },
    },
    {
        name = "F",
        ch = {
            {n = "F1", f = 5740},
            {n = "F2", f = 5760},
            {n = "F3", f = 5780},
            {n = "F4", f = 5800},
            {n = "F5", f = 5820},
            {n = "F6", f = 5840},
            {n = "F7", f = 5860},
            {n = "F8", f = 5880},
        },
    },
}

local band_i = 1
local ch_i = 5

-- sat_pct: 0 to 250 (100 is normal color, 0 is monochrome)
local sat_pct = 100
local saved_sat_pct = 100
local is_mono = false
local help_enabled = false

local function cur()
    return bands[band_i].ch[ch_i]
end

-- Write control file for demodulator engine (atomic write via /dev/shm)
local function apply_video_controls()
    local f = io.open("/dev/shm/quadrf-ntsc-ctrl.tmp", "w")
    if not f then f = io.open("/tmp/quadrf-ntsc-ctrl.tmp", "w") end
    if f then
        local s_mult = is_mono and 0.0 or (sat_pct / 100.0)
        f:write(string.format("sat=%.2f\nmono=%d\n", s_mult, is_mono and 1 or 0))
        f:close()
        os.rename("/dev/shm/quadrf-ntsc-ctrl.tmp", "/dev/shm/quadrf-ntsc-ctrl")
        os.rename("/tmp/quadrf-ntsc-ctrl.tmp", "/tmp/quadrf-ntsc-ctrl")
    end
end

local function show_ch()
    local c = cur()
    mp.osd_message(string.format("%s    %d MHz", c.n, c.f), 1.5)
end

local function apply_ch()
    local c = cur()
    os.execute(string.format("quadrf-jtag --rx freq=%d >/dev/null 2>&1 &", c.f))
    show_ch()
end

local function ch_up()
    ch_i = ch_i % #bands[band_i].ch + 1
    apply_ch()
end

local function ch_down()
    ch_i = (ch_i - 2) % #bands[band_i].ch + 1
    apply_ch()
end

local function band_next()
    band_i = band_i % #bands + 1
    apply_ch()
    local c = cur()
    mp.osd_message(string.format("Band %s · %s    %d MHz", bands[band_i].name, c.n, c.f), 1.5)
end

local function band_prev()
    band_i = (band_i - 2) % #bands + 1
    apply_ch()
    local c = cur()
    mp.osd_message(string.format("Band %s · %s    %d MHz", bands[band_i].name, c.n, c.f), 1.5)
end

local function sat_up()
    if is_mono then
        is_mono = false
    end
    sat_pct = math.min(250, sat_pct + 10)
    saved_sat_pct = sat_pct
    apply_video_controls()
    mp.osd_message(string.format("Saturation: %d%%", sat_pct), 1.5)
end

local function sat_down()
    if is_mono then
        is_mono = false
    end
    sat_pct = math.max(0, sat_pct - 10)
    if sat_pct == 0 then
        is_mono = true
        apply_video_controls()
        mp.osd_message("Color Mode: MONOCHROME (B&W)", 1.5)
    else
        saved_sat_pct = sat_pct
        apply_video_controls()
        mp.osd_message(string.format("Saturation: %d%%", sat_pct), 1.5)
    end
end

local function toggle_mono()
    is_mono = not is_mono
    if is_mono then
        saved_sat_pct = sat_pct
        apply_video_controls()
        mp.osd_message("Color Mode: MONOCHROME (B&W)", 2.0)
    else
        sat_pct = (saved_sat_pct == 0) and 100 or saved_sat_pct
        apply_video_controls()
        mp.osd_message(string.format("Color Mode: COLOR (%d%%)", sat_pct), 2.0)
    end
end

local function reset_video()
    sat_pct = 100
    saved_sat_pct = 100
    is_mono = false
    apply_video_controls()
    mp.osd_message("Video: Defaults Reset (Color 100%)", 2.0)
end

local function toggle_help()
    help_enabled = not help_enabled
    if help_enabled then
        local help_text =
            "═════════════════════ [ QuadRF Controls ] ═════════════════════\n" ..
            "  Up / Down, Wheel : Hop channel in the current band (1–8)\n" ..
            "  Left / Right     : Change band (Race, A, B, E, F)\n" ..
            "  s / S            : Adjust Saturation (-10% / +10%)\n" ..
            "  c                : Toggle Monochrome (B&W) / Color Mode\n" ..
            "  r                : Reset Video Defaults (100% Sat)\n" ..
            "  ? / F1           : Toggle this Command Help Screen\n" ..
            "  q / Esc          : Quit Player and Stop Demodulator\n" ..
            "═══════════════════════════════════════════════════════════════"
        mp.osd_message(help_text, 8.0)
    else
        mp.osd_message("", 0.1)
    end
end

local function welcome()
    local c = cur()
    mp.osd_message(string.format(
        "Analog 5.8 GHz FPV · NTSC\n" ..
        "Up/Down: Channel   Left/Right: Band\n" ..
        "s/S: Saturation    c: Mono/Color    ?: Help   q: Quit\n" ..
        "Current: %s  %d MHz", c.n, c.f), 5)
    apply_video_controls()
end

mp.add_forced_key_binding("UP", "ntsc-ch-up", ch_up, {repeatable = true})
mp.add_forced_key_binding("DOWN", "ntsc-ch-down", ch_down, {repeatable = true})
mp.add_forced_key_binding("WHEEL_UP", "ntsc-ch-wheel-up", ch_up)
mp.add_forced_key_binding("WHEEL_DOWN", "ntsc-ch-wheel-down", ch_down)
mp.add_forced_key_binding("LEFT", "ntsc-band-prev", band_prev, {repeatable = true})
mp.add_forced_key_binding("RIGHT", "ntsc-band-next", band_next, {repeatable = true})

mp.add_forced_key_binding("s", "ntsc-sat-down", sat_down, {repeatable = true})
mp.add_forced_key_binding("S", "ntsc-sat-up", sat_up, {repeatable = true})
mp.add_forced_key_binding("c", "ntsc-mono-toggle", toggle_mono)
mp.add_forced_key_binding("r", "ntsc-reset-video", reset_video)

mp.add_forced_key_binding("?", "ntsc-help-toggle-qmark", toggle_help)
mp.add_forced_key_binding("F1", "ntsc-help-toggle-f1", toggle_help)

mp.register_event("file-loaded", welcome)
