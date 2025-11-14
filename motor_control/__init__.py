"""
OpenHyperspectral Motor Control Package

This package provides Python interface for controlling the OpenHyperspectral
motor controller via USB serial communication.
"""

from .controller import MotorController, ControlMode
from .scanner import HyperspectralScanner

__version__ = "1.0.0"
__all__ = ['MotorController', 'ControlMode', 'HyperspectralScanner']
