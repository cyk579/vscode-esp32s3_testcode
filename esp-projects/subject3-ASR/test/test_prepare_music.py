"""The phone and car must agree on the exact prepared library before selecting songs."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
import wave

spec = importlib.util.spec_from_file_location("prepare_music", Path(__file__).resolve().parents[1] / "tools/prepare_music.py")
music = importlib.util.module_from_spec(spec)
spec.loader.exec_module(music)


class PrepareMusicTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.manifest = self.root / "library.json"
        self.output = self.root / "musicfs"
        self.asset = self.root / "assets/music_catalog.json"
        self.config = {"sample_rate": 16000, "tracks": [
            {"id": 1, "title": "First", "aliases": ["One"], "file": "input.wav"}
        ]}
        self.wav()

    def wav(self, rate=16000, channels=1, frames=b"\x01\x00" * 160):
        buf = io.BytesIO()
        with wave.open(buf, "wb") as w:
            w.setnchannels(channels)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes(frames)
        (self.root / "input.wav").write_bytes(buf.getvalue())

    def prepare(self):
        self.manifest.write_text(json.dumps(self.config), encoding="utf-8")
        return music.prepare(self.manifest, self.output, self.asset)

    def test_catalog_matches_both_outputs_and_changes_with_audio_or_names(self):
        first = self.prepare()
        self.assertEqual(first, self.prepare())
        self.assertEqual(first, json.loads(self.asset.read_text(encoding="utf-8")))
        self.assertEqual(first["catalog"], int((self.output / "catalog.id").read_text(), 16))
        self.wav(frames=b"\x02\x00" * 160)
        second = self.prepare()
        self.assertNotEqual(first["catalog"], second["catalog"])
        self.config["tracks"][0]["aliases"] = ["New alias"]
        self.assertNotEqual(second["catalog"], self.prepare()["catalog"])

    def test_duplicate_and_invalid_metadata_fail_before_overwriting_outputs(self):
        first = self.prepare()
        entry = self.config["tracks"][0]
        for field, value in [("id", True), ("id", 0), ("title", " First"),
                             ("aliases", "One"), ("aliases", ["First"])]:
            with self.subTest(field=field, value=value):
                old = entry[field]
                entry[field] = value
                with self.assertRaises(ValueError):
                    self.prepare()
                entry[field] = old
                self.assertEqual(first, json.loads(self.asset.read_text(encoding="utf-8")))
        self.config["tracks"].append(dict(entry, title="Second", aliases=[]))
        with self.assertRaises(ValueError):
            self.prepare()

    def test_wrong_rate_stereo_empty_and_truncated_files_are_rejected(self):
        for args in [{"rate": 48000}, {"channels": 2}, {"frames": b""}]:
            with self.subTest(args=args):
                self.wav(**args)
                with self.assertRaises(ValueError):
                    self.prepare()
        self.wav()
        source = self.root / "input.wav"
        source.write_bytes(source.read_bytes()[:-2])
        with self.assertRaises(ValueError):
            self.prepare()
        self.assertFalse(self.asset.exists())

    def test_rebuild_removes_only_obsolete_numeric_wavs(self):
        self.prepare()
        (self.output / "notes.wav").write_bytes(b"keep")
        self.config["tracks"][0]["id"] = 2
        self.prepare()
        self.assertFalse((self.output / "1.wav").exists())
        self.assertTrue((self.output / "2.wav").exists())
        self.assertEqual(b"keep", (self.output / "notes.wav").read_bytes())


if __name__ == "__main__":
    unittest.main()
