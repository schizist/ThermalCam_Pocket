"""
Pre-build script: reads .env from project root and injects each KEY=VALUE
pair as a C preprocessor define so firmware can use #ifdef / #ifndef guards.

String values are automatically wrapped in escaped quotes so they compile as
C string literals:  -DWIFI_PASSWORD=\"your_password\"
"""

Import("env")  # noqa: F821  (PlatformIO injects this)
import os

env_path = os.path.join(env.subst("$PROJECT_DIR"), ".env")  # noqa: F821
if not os.path.exists(env_path):
    print("[load_env] No .env file found — using defaults from source code")
else:
    with open(env_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, val = line.split("=", 1)
            key, val = key.strip(), val.strip()
            # Inject as C define and into OS env (for upload_flags interpolation)
            env.Append(CPPDEFINES=[(key, f'\\"{val}\\"')])  # noqa: F821
            os.environ[key] = val
            print(f"[load_env] Loaded {key}")
