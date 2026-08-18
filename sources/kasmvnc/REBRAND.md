# Regenerating the KasmVNC overlay

`www/` holds the QuadRF-branded copies of the KasmVNC web client, and
`50-desktop` lays them over `/usr/share/kasmvnc/www` after install. When
KasmVNC is bumped in `packaging/pins.env` the upstream asset filenames change,
so the overlay has to be rebuilt from the new client rather than copied
forward.

Install the new `kasmvncserver`, run the edits below against
`/usr/share/kasmvnc/www`, then copy `index.html`, `vnc.html`, `screen.html`,
`disconnected.html`, `favicon.png` and the touched files under `assets/` back
into this directory.

Title and icons:

```bash
cd /usr/share/kasmvnc/www
sudo perl -pi -e 's|<title>.*?</title>|<title>RF Quad Kit</title>|g' index.html vnc.html screen.html
sudo perl -pi -e 's|<link rel="(apple-touch-)?icon".*?>||g' index.html vnc.html screen.html
sudo perl -pi -e 's|</head>|<link rel="icon" type="image/png" href="./favicon.png"></head>|g' index.html vnc.html screen.html
```

Control panel logo:

```bash
sudo perl -pi -e 's|<h1 class="noVNC_logo">.*?</h1>|<h1 class="noVNC_logo"><a href="https://open.space" target="_blank"><img src="./assets/rf_quad_kit_logo.png"></a></h1>|g' index.html vnc.html
```

Desktop and tab names, in the bundled UI script:

```bash
cd assets
sudo perl -pi -e 's/ - KasmVNC//g' ui-*.js
sudo perl -pi -e 's/o\.desktopName=r\.detail\.name/o.desktopName="RF Quad Kit"/g' ui-*.js
sudo perl -pi -e 's/document\.title=r\.detail\.name\+" - "\+ox/document.title="RF Quad Kit"/g' ui-*.js
```

Logo sizing and the loading screen, in the bundled stylesheet:

```bash
sudo perl -pi -e 's/(\.noVNC_logo\{[^}]*)background:#fff;([^}]*\})/$1background:transparent;$2/g' ui-*.css
sudo perl -pi -e 's/\.noVNC_logo img\{width:45%\}/.noVNC_logo img{width:100%; max-width:785px;}/g' ui-*.css
sudo perl -pi -e 's/\.noVNC_logo img\{width:50%;\}/.noVNC_logo img{width:100%; max-width:140px;}/g' ui-*.css
sudo perl -pi -e 's/background:#fff url\("data:image\/svg\+xml[^"]*"\)/background:#fff url(.\/rf_quad_kit_logo.png)/g' ui-*.css
```

Replace the game cursor button with a link to the control panel:

```bash
cd /usr/share/kasmvnc/www
sudo perl -pi -e 's|<div class="noVNC_button_div[^>]*><input type="image" alt="Game Mode".*?id="noVNC_game_mode_button".*?Game Cursor Mode</div>|<div id="noVNC_game_mode_button" style="display:none"></div><a href="#" target="_blank" style="color:inherit; text-decoration:none;" onmouseover="this.href=\x27http://\x27 + window.location.hostname + \x27/\x27;"><div class="noVNC_button_div noVNC_hide_on_disconnect" style="cursor:pointer;"><svg style="vertical-align:middle; margin:10px 4px 10px 0; padding:4px;" xmlns="http://www.w3.org/2000/svg" width="25" height="25" viewBox="0 0 24 24" fill="none" stroke="#ffffff" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"></polyline></svg> QuadRF Control</div></a>|g' *.html
```

The button may already be gone in a newer client; if the substitution matches
nothing, drop it and hide the element in CSS instead:

```css
#noVNC_game_mode_button { display: none !important; }
div.noVNC_button_div:has(#noVNC_game_mode_button) { display: none !important; }
```
