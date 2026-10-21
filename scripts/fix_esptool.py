"""
Post-build-setup script: point $OBJCOPY at the real esptool executable.

pioarduino looks for esptool.exe in "<python dir>\\Scripts", but on Windows the
PlatformIO venv's python.exe already lives in penv\\Scripts, so it misses and
falls back to the bare command "esptool". If PATHEXT contains .PY, Windows then
resolves that to tool-esptoolpy\\esptool.py and opens it with whatever app owns
.py files (e.g. Notepad++), and the build hangs at "Building ...bin".
"""

Import("env")  # noqa: F821  (PlatformIO injects this)
import os
import sys

exe = "esptool.exe" if sys.platform == "win32" else "esptool"
candidate = os.path.join(os.path.dirname(env.subst("$PYTHONEXE")), exe)  # noqa: F821
if os.path.isfile(candidate):
    quoted = f'"{candidate}"' if " " in candidate else candidate
    # OBJCOPY builds the .bin images; UPLOADER flashes over USB (espota is unaffected)
    for var in ("OBJCOPY", "UPLOADER"):
        if env.subst(f"${var}").strip('"') == "esptool":  # noqa: F821
            env.Replace(**{var: quoted})  # noqa: F821
            print(f"[fix_esptool] {var} -> {candidate}")
