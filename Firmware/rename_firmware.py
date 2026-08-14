"""PlatformIO extra script: name the build artifacts after the board + version.

Instead of the default ``firmware.bin`` this reads the single source of truth
for the image version — ``HIVETRAFFIC_FW_VERSION`` in ``include/version.h`` —
and combines it with the board label and the build variant, producing

    hivetraffic_esp32-c6_<version>.bin           (production)

That name is what makes an image self-describing to HiveHub. Its firmware
upload form parses the filename and pre-fills the release for you:

  * the leading ``hivetraffic`` token selects target *HiveTraffic counter*
    (``beecounter`` server-side) instead of leaving the default main unit.
    This is a convenience in the dashboard form only — the backend never infers
    the target from a filename, it reads the explicit ``target`` field — so the
    token saves an operator a mis-click, it does not enforce anything. (Its
    regex also matches a ``beecounter`` prefix, so an image named that way is
    targeted correctly too rather than silently mis-filed.)
  * the ``esp32-c6`` token is the board the release is stamped with — the
    backend refuses to publish a release whose declared board disagrees with
    its filename, which is what stops a cross-architecture image from ever
    reaching a counter,
  * the trailing dotted token pre-fills the Version field.

The label and layout therefore are not cosmetic: they must stay in sync with
HiveHub's ``rename_firmware.py`` (BOARD_LABELS), ``server/firmware.py``
(``board_from_filename``, ``BEECOUNTER_BOARDS``) and the dashboard's
``versionFromFilename`` / ``targetFromFilename``.

Any future variant is stamped *after* the version rather than into the board
label on purpose. That keeps the board token exactly ``esp32-c6`` (so the name
still parses), it is what the operator sees in the pre-filled Version field —
"0.1.0-something" is hard to publish by accident — and HiveHub compares versions
component-wise with non-digits stripped, so such a build never out-ranks the
production image of the same version and cannot be relayed over one.

Referenced from platformio.ini via ``extra_scripts = pre:rename_firmware.py``.
"""

import os
import re

Import("env")  # noqa: F821 - provided by PlatformIO

# The counter is a single-board product (Seeed XIAO ESP32C6, PlatformIO board
# seeed_xiao_esp32c6), so every environment carries the same board label; the
# environments differ only in the variant suffix below.
BOARD_LABEL = "esp32-c6"

# Suffix appended after the version, per PlatformIO env name. The production
# env gets none. Anything unlisted falls back to its raw env name, so a new
# environment is still named distinctly instead of silently overwriting the
# production artifact.
VARIANT_SUFFIXES = {
    "seeed_xiao_esp32c6": "",
}


def variant_suffix():
    env_name = env.subst("$PIOENV")  # noqa: F821 - provided by PlatformIO
    suffix = VARIANT_SUFFIXES.get(env_name)
    if suffix is None:
        suffix = "-" + env_name
    return re.sub(r"[^0-9A-Za-z._-]", "_", suffix)


def read_firmware_version():
    include_dir = env.subst("$PROJECT_INCLUDE_DIR")  # noqa: F821
    if not include_dir:
        include_dir = os.path.join(env.subst("$PROJECT_DIR"), "include")  # noqa: F821
    version_path = os.path.join(include_dir, "version.h")
    try:
        with open(version_path, "r", encoding="utf-8") as handle:
            source = handle.read()
    except OSError as exc:
        print("[rename_firmware] Could not read %s: %s" % (version_path, exc))
        return None

    match = re.search(r'#\s*define\s+HIVETRAFFIC_FW_VERSION\s+"([^"]+)"', source)
    if not match:
        print("[rename_firmware] HIVETRAFFIC_FW_VERSION not found in version.h")
        return None
    return match.group(1)


version = read_firmware_version()
if version:
    # Sanitize so the version can't produce an invalid file name.
    safe_version = re.sub(r"[^0-9A-Za-z._-]", "_", version)
    progname = "hivetraffic_%s_%s%s" % (BOARD_LABEL, safe_version, variant_suffix())
    env.Replace(PROGNAME=progname)  # noqa: F821
    print("[rename_firmware] Build artifacts will be named %s.bin / .elf" % progname)
else:
    print("[rename_firmware] Falling back to default firmware.bin name")
