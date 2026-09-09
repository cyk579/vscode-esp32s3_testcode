"""Prepare mono PCM16 WAV files and one shared phone/car catalog. No network access."""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import wave
import zlib

PROJECT = Path(__file__).resolve().parents[1]
APP_ASSET = PROJECT.parent / "gesture-android/app/src/main/assets/music_catalog.json"


def tone(rate):
    """A quiet generated test tone, explicitly not a named commercial song."""
    out = io.BytesIO()
    with wave.open(out, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(rate)
        frames = bytearray()
        for i in range(rate * 6):
            t = i / rate
            fade = min(1.0, t * 10, (6 - t) * 10)
            frequency = (262, 330, 392)[int(t) % 3]
            frames.extend(struct.pack("<h", int(5000 * fade * math.sin(2 * math.pi * frequency * t))))
        wav.writeframes(frames)
    return out.getvalue()


def prepare(manifest, output, app_asset):
    config = json.loads(manifest.read_text(encoding="utf-8"))
    rate = config["sample_rate"]
    if type(rate) is not int or not 8000 <= rate <= 48000:
        raise ValueError("sample_rate must be 8000..48000 and supported by the USB speaker")
    tracks, files, ids, names = [], {}, set(), set()
    for entry in config["tracks"]:
        track_id, title = entry["id"], entry["title"]
        aliases = entry.get("aliases", [])
        if type(track_id) is not int or not 1 <= track_id <= 65535 or track_id in ids:
            raise ValueError("Track IDs must be unique integers 1..65535")
        if not isinstance(aliases, list):
            raise ValueError("Track aliases must be a JSON array")
        for name in [title, *aliases]:
            if not isinstance(name, str) or not name or name != name.strip() or name in names:
                raise ValueError("Titles/aliases must be nonempty, trimmed and unique")
            names.add(name)
        ids.add(track_id)
        data = tone(rate) if entry["file"] == "@test-tone" else (manifest.parent / entry["file"]).read_bytes()
        with wave.open(io.BytesIO(data), "rb") as wav:
            if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate(), wav.getcomptype()) != (1, 2, rate, "NONE"):
                raise ValueError(f"{title}: require mono PCM16 WAV at {rate} Hz. Convert with ffmpeg first.")
            pcm = wav.readframes(wav.getnframes())
            if not pcm or len(pcm) != wav.getnframes() * 2:
                raise ValueError(f"{title}: empty or truncated PCM")
        # Repack to canonical RIFF PCM; metadata and unusual chunks stay out of firmware files.
        canonical = io.BytesIO()
        with wave.open(canonical, "wb") as wav:
            wav.setnchannels(1); wav.setsampwidth(2); wav.setframerate(rate); wav.writeframes(pcm)
        data = canonical.getvalue()
        files[f"{track_id}.wav"] = data
        tracks.append({"id": track_id, "title": title, "aliases": aliases, "sha256": hashlib.sha256(data).hexdigest()})
    if not tracks:
        raise ValueError("Include at least one track")
    # SPIFFS overhead is significant. Keep content below 70% of the 24 MiB partition.
    if sum(map(len, files.values())) > int(24 * 1024 * 1024 * 0.70):
        raise ValueError("Library exceeds 70% of the 24 MiB music partition")
    tracks.sort(key=lambda item: item["id"])
    fingerprint = zlib.crc32(json.dumps({"rate": rate, "tracks": tracks}, ensure_ascii=True, sort_keys=True).encode())
    catalog = {"version": 1, "catalog": fingerprint, "sample_rate": rate, "tracks": tracks}
    output.mkdir(parents=True, exist_ok=True)
    # Only numeric WAVs in this explicit generated directory are managed by this script.
    for old in output.glob("*.wav"):
        if old.stem.isdecimal() and old.name not in files:
            old.unlink()
    for name, data in files.items():
        (output / name).write_bytes(data)
    (output / "catalog.id").write_text(f"{fingerprint:08x}\n", encoding="ascii")
    app_asset.parent.mkdir(parents=True, exist_ok=True)
    app_asset.write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared {len(tracks)} tracks, catalog={fingerprint:08x}, rate={rate} Hz; set the same speaker rate in menuconfig.")
    return catalog


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=PROJECT / "music/library.json")
    args = parser.parse_args()
    prepare(args.manifest.resolve(), PROJECT / "musicfs", APP_ASSET)
