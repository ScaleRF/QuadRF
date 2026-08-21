"""Resolve web-launcher icons from freedesktop application icon folders."""

import os
import re


ICON_DIRS = tuple(filter(None, os.environ.get(
    "QUADRF_ICON_DIRS",
    ":".join([
        "/usr/local/share/icons/hicolor/scalable/apps",
        "/usr/share/icons/hicolor/scalable/apps",
        "/usr/share/icons/hicolor/512x512/apps",
        "/usr/share/icons/hicolor/256x256/apps",
        "/usr/share/icons/hicolor/128x128/apps",
        "/usr/share/icons/hicolor/64x64/apps",
        "/usr/share/icons/hicolor/48x48/apps",
        "/usr/local/share/pixmaps",
        "/usr/share/pixmaps",
    ]),
).split(":")))
ICON_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
ICON_EXTENSIONS = (".svg", ".png", ".webp")


def find_app_icon(icon, directories=None):
    """Resolve a freedesktop icon name within trusted system icon folders."""
    if not icon or not ICON_NAME_RE.fullmatch(icon):
        return None
    names = [icon] if os.path.splitext(icon)[1].lower() in ICON_EXTENSIONS else [
        f"{icon}{extension}" for extension in ICON_EXTENSIONS
    ]
    for directory in directories or ICON_DIRS:
        root = os.path.realpath(directory)
        for name in names:
            candidate = os.path.realpath(os.path.join(root, name))
            try:
                inside_root = os.path.commonpath((root, candidate)) == root
            except ValueError:
                inside_root = False
            if inside_root and os.path.isfile(candidate):
                return candidate
    return None
