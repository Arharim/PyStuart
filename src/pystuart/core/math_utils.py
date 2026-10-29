"""Mathematical utilities for Stewart Platform."""
import numpy as np


def rotation_matrix(rotation):
    """
    Create a rotation matrix from Euler angles (XYZ order).
    
    Args:
        rotation: Array of [rx, ry, rz] angles in radians.
        
    Returns:
        3x3 rotation matrix.
    """
    rx, ry, rz = rotation
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])
    
    return Rz @ Ry @ Rx


def degrees_to_radians(degrees):
    """Convert degrees to radians."""
    return np.radians(degrees)


def radians_to_degrees(radians):
    """Convert radians to degrees."""
    return np.degrees(radians)


def clamp(value, min_val, max_val):
    """Clamp a value between min and max."""
    return max(min_val, min(max_val, value))


def safe_arcsin(value):
    """
    Compute arcsin with clamping to avoid NaN from values outside [-1, 1].
    
    Args:
        value: Input value.
        
    Returns:
        arcsin result, or NaN if value is significantly out of range.
    """
    if value < -1.0 or value > 1.0:
        return np.nan
    return np.arcsin(value)
