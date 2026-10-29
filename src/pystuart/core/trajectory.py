import csv
import math
from typing import List, Optional, Tuple


class TrajectoryPoint:
    __slots__ = ('pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z')

    def __init__(self, pos_x=0.0, pos_y=0.0, pos_z=0.0, rot_x=0.0, rot_y=0.0, rot_z=0.0):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.rot_x = rot_x
        self.rot_y = rot_y
        self.rot_z = rot_z

    def to_pose_dict(self) -> dict:
        return {
            'pos_x': self.pos_x,
            'pos_y': self.pos_y,
            'pos_z': self.pos_z,
            'rot_x': math.radians(self.rot_x),
            'rot_y': math.radians(self.rot_y),
            'rot_z': math.radians(self.rot_z),
        }

    @staticmethod
    def lerp(a: 'TrajectoryPoint', b: 'TrajectoryPoint', t: float) -> 'TrajectoryPoint':
        return TrajectoryPoint(
            pos_x=a.pos_x + (b.pos_x - a.pos_x) * t,
            pos_y=a.pos_y + (b.pos_y - a.pos_y) * t,
            pos_z=a.pos_z + (b.pos_z - a.pos_z) * t,
            rot_x=a.rot_x + (b.rot_x - a.rot_x) * t,
            rot_y=a.rot_y + (b.rot_y - a.rot_y) * t,
            rot_z=a.rot_z + (b.rot_z - a.rot_z) * t,
        )


class Trajectory:
    def __init__(self):
        self.waypoints: List[TrajectoryPoint] = []
        self.interpolated: List[TrajectoryPoint] = []
        self._steps_per_segment: int = 50

    def load_csv(self, filepath: str) -> Tuple[int, Optional[str]]:
        waypoints = []
        try:
            with open(filepath, 'r', newline='', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row_num, row in enumerate(reader, start=2):
                    try:
                        pt = TrajectoryPoint(
                            pos_x=float(row.get('pos_x', 0)),
                            pos_y=float(row.get('pos_y', 0)),
                            pos_z=float(row.get('pos_z', 0)),
                            rot_x=float(row.get('rot_x', 0)),
                            rot_y=float(row.get('rot_y', 0)),
                            rot_z=float(row.get('rot_z', 0)),
                        )
                        waypoints.append(pt)
                    except (ValueError, TypeError) as e:
                        return 0, f"Row {row_num}: {e}"
        except FileNotFoundError:
            return 0, "File not found"
        except Exception as e:
            return 0, str(e)

        if len(waypoints) < 2:
            return 0, "Need at least 2 waypoints"

        self.waypoints = waypoints
        self._build_interpolated()
        return len(self.waypoints), None

    def set_steps_per_segment(self, steps: int):
        self._steps_per_segment = max(1, steps)
        if self.waypoints:
            self._build_interpolated()

    def _build_interpolated(self):
        self.interpolated = []
        n = len(self.waypoints)
        for i in range(n - 1):
            a = self.waypoints[i]
            b = self.waypoints[i + 1]
            for s in range(self._steps_per_segment):
                t = s / self._steps_per_segment
                self.interpolated.append(TrajectoryPoint.lerp(a, b, t))
        self.interpolated.append(self.waypoints[-1])

    def total_frames(self) -> int:
        return len(self.interpolated)

    def get_frame(self, index: int) -> Optional[TrajectoryPoint]:
        if 0 <= index < len(self.interpolated):
            return self.interpolated[index]
        return None

    def is_empty(self) -> bool:
        return len(self.interpolated) == 0
