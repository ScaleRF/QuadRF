-- Raceband channel hop for Camera Decoder. UP/DOWN retune the MAX2851 LO.
local ch = {
    {n = "R1", f = 5658},
    {n = "R2", f = 5695},
    {n = "R3", f = 5732},
    {n = "R4", f = 5769},
    {n = "R5", f = 5806},
    {n = "R6", f = 5843},
    {n = "R7", f = 5880},
    {n = "R8", f = 5917},
}
local i = 5

local function show()
    local c = ch[i]
    mp.osd_message(string.format("%s    %d MHz", c.n, c.f), 1.5)
end

local function welcome()
    local c = ch[i]
    mp.osd_message(string.format(
        "Analog 5.8 GHz FPV  ·  NTSC\n" ..
        "Up/Down  Raceband channel     q  quit\n" ..
        "Click this window, then use the keys.\n" ..
        "%s  %d MHz", c.n, c.f), 4)
end

local function apply()
    local c = ch[i]
    os.execute(string.format("quadrf-jtag --rx freq=%d >/dev/null 2>&1 &", c.f))
    show()
end

local function up()
    i = i % #ch + 1
    apply()
end

local function down()
    i = (i - 2) % #ch + 1
    apply()
end

mp.add_forced_key_binding("UP", "ntsc-ch-up", up, {repeatable = true})
mp.add_forced_key_binding("DOWN", "ntsc-ch-down", down, {repeatable = true})
mp.add_forced_key_binding("WHEEL_UP", "ntsc-ch-wheel-up", up)
mp.add_forced_key_binding("WHEEL_DOWN", "ntsc-ch-wheel-down", down)
mp.register_event("file-loaded", welcome)
