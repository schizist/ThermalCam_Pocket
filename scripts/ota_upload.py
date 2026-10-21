"""
Pre-build script for the OTA environment (runs after load_env.py).

- Passes OTA_PASSWORD from .env to espota. `upload_flags = --auth=${sysenv...}`
  can't do this: platformio.ini is resolved before load_env.py loads .env.
- Ignores a serial port (e.g. COM4) picked in VS Code's status bar, which
  PlatformIO passes as --upload-port and which would override upload_port.
"""

Import("env")  # noqa: F821  (PlatformIO injects this)
import os
import re

password = os.environ.get("OTA_PASSWORD", "")
if password:
    env.Replace(UPLOAD_FLAGS=[f"--auth={password}"])  # noqa: F821

port = env.subst("$UPLOAD_PORT")  # noqa: F821
if re.match(r"^(COM\d+|/dev/)", port, re.IGNORECASE):
    ota_port = env.GetProjectOption("upload_port", "dollhead.local")  # noqa: F821
    print(f"[ota_upload] Ignoring serial port {port}, uploading to {ota_port}")
    env.Replace(UPLOAD_PORT=ota_port)  # noqa: F821
