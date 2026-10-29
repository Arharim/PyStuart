import json
import math
from pathlib import Path
from typing import Dict, Optional


class PosePresets:
    def __init__(self, config_path: Optional[Path] = None):
        if config_path is None:
            config_dir = Path.home() / ".pystewart"
            config_dir.mkdir(exist_ok=True)
            config_path = config_dir / "presets.json"
        self.config_path = config_path
        self.presets: Dict[str, dict] = {}
        self._load()

    def _load(self):
        if self.config_path.exists():
            try:
                with open(self.config_path, 'r', encoding='utf-8') as f:
                    self.presets = json.load(f)
            except (json.JSONDecodeError, IOError):
                self.presets = {}
        else:
            self.presets = {}

    def _save(self):
        with open(self.config_path, 'w', encoding='utf-8') as f:
            json.dump(self.presets, f, indent=2)

    def get_preset_names(self) -> list:
        return sorted(self.presets.keys())

    def save_preset(self, name: str, pose: dict) -> bool:
        if not name or not name.strip():
            return False
        name = name.strip()
        stored_pose = {
            'pos_x': pose.get('pos_x', 0),
            'pos_y': pose.get('pos_y', 0),
            'pos_z': pose.get('pos_z', 0),
            'rot_x_deg': math.degrees(pose.get('rot_x', 0)),
            'rot_y_deg': math.degrees(pose.get('rot_y', 0)),
            'rot_z_deg': math.degrees(pose.get('rot_z', 0)),
        }
        self.presets[name] = stored_pose
        self._save()
        return True

    def load_preset(self, name: str) -> Optional[dict]:
        if name not in self.presets:
            return None
        stored = self.presets[name]
        return {
            'pos_x': stored.get('pos_x', 0),
            'pos_y': stored.get('pos_y', 0),
            'pos_z': stored.get('pos_z', 0),
            'rot_x': math.radians(stored.get('rot_x_deg', 0)),
            'rot_y': math.radians(stored.get('rot_y_deg', 0)),
            'rot_z': math.radians(stored.get('rot_z_deg', 0)),
        }

    def delete_preset(self, name: str) -> bool:
        if name in self.presets:
            del self.presets[name]
            self._save()
            return True
        return False

    def preset_exists(self, name: str) -> bool:
        return name in self.presets
