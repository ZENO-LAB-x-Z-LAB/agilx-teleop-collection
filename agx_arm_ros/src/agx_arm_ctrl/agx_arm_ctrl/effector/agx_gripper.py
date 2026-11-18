#!/usr/bin/env python3
# -*-coding:utf8-*-
import math
from typing import Optional, TYPE_CHECKING, Tuple
from dataclasses import dataclass

from pyAgxArm.protocols.can_protocol.msgs.effector.agx_gripper.default import (
    ArmMsgGripperCtrl,
)

if TYPE_CHECKING:
    from pyAgxArm.protocols.can_protocol.drivers.core.arm_driver_abstract import ArmDriverAbstract
    from pyAgxArm.protocols.can_protocol.drivers import AgxGripperDriverDefault


@dataclass
class GripperStatus:
    position: float = 0.0               # Current gripper position in active mode units (m/deg)
    mode: str = "width"                 # Active gripper mode: width / angle
    width: float = math.nan             # Current gripper opening width, unit: m; NaN in angle mode
    force: float = 0.0                  # Current gripping force, unit: N
    voltage_too_low: bool = False       # Voltage too low
    motor_overheating: bool = False     # Motor overheating
    driver_overcurrent: bool = False    # Driver overcurrent
    driver_overheating: bool = False    # Driver overheating
    sensor_status: bool = False         # Sensor status
    driver_error_status: bool = False   # Driver error status
    driver_enable_status: bool = False  # Driver enable status
    homing_status: bool = False         # Homing/zero position status
    hz: float = 0.0                     # Message receiving frequency
    timestamp: float = 0.0              # Timestamp


@dataclass
class GripperCtrlStatus:
    width: float = 0.0          # Current gripper opening width, unit: m
    force: float = 0.0          # Current gripping force, unit: N
    status_code: int = 0        # Status code
    set_zero: int = 0           # Homing/zeroing field
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp


@dataclass
class GripperTeachingParam:
    teaching_range_per: int = 100   # Teaching range (percentage), range: [100, 200]
    max_range_config: float = 0.0   # Maximum stroke configuration, unit: m, only supports: 0/0.07/0.1
    teaching_friction: int = 1      # Teaching friction, range: 1..10
    hz: float = 0.0
    timestamp: float = 0.0


class AgxGripperWrapper:
    
    # Gripper control parameter ranges
    WIDTH_MIN: float = 0.0      # minimum width, unit: m
    WIDTH_MAX: float = 0.1      # maximum width, unit: m
    FORCE_MIN: float = 0.0      # minimum force, unit: N
    FORCE_MAX: float = 3.0      # maximum force, unit: N
    ANGLE_MIN: float = 0.0      # minimum angle, unit: deg
    ANGLE_MAX: float = 180.0    # conservative upper bound, unit: deg
    
    def __init__(self, agx_arm):
        self._agx_arm: Optional[ArmDriverAbstract] = agx_arm
        self._effector: Optional[AgxGripperDriverDefault] = None
        self._initialized = False
    
    def initialize(self) -> bool:
        if self._initialized:
            return True
        
        try:
            self._effector = self._agx_arm.init_effector(
                self._agx_arm.OPTIONS.EFFECTOR.AGX_GRIPPER
            )
            self._initialized = True
            return True
        except Exception as e:
            print(f"[AgxGripperWrapper] initialize failed: {e}")
            return False
    
    def is_ok(self) -> bool:
        if not self._initialized or self._effector is None:
            return False
        return self._effector.is_ok()
    
    def get_fps(self) -> float:
        if not self._initialized or self._effector is None:
            return 0.0
        return self._effector.get_fps()
    
    def get_status(self) -> Optional[GripperStatus]:
        if not self._initialized or self._effector is None:
            return None
        
        gs = self._effector.get_gripper_status()
        if gs is None:
            return None
        
        mode = str(getattr(gs.msg, "mode", "width"))
        position = float(gs.msg.value)
        status = GripperStatus(
            position=position,
            mode=mode,
            width=position if mode == "width" else math.nan,
            force=gs.msg.force,
            voltage_too_low=gs.msg.foc_status.voltage_too_low,
            motor_overheating=gs.msg.foc_status.motor_overheating,
            driver_overcurrent=gs.msg.foc_status.driver_overcurrent,
            driver_overheating=gs.msg.foc_status.driver_overheating,
            sensor_status=gs.msg.foc_status.sensor_status,
            driver_error_status=gs.msg.foc_status.driver_error_status,
            driver_enable_status=gs.msg.foc_status.driver_enable_status,
            homing_status=gs.msg.foc_status.homing_status,
            hz=gs.hz,
            timestamp=gs.timestamp
        )
        return status
    
    def get_ctrl_states(self) -> Optional[GripperCtrlStatus]:
        if not self._initialized or self._effector is None:
            return None
        
        gcs = self._effector.get_gripper_ctrl_states()
        if gcs is None:
            return None
        
        ctrl_status = GripperCtrlStatus(
            width=gcs.msg.width,
            force=gcs.msg.force,
            status_code=gcs.msg.status_code,
            set_zero=gcs.msg.set_zero,
            hz=gcs.hz,
            timestamp=gcs.timestamp
        )
        return ctrl_status
    
    def get_teaching_param(
        self, 
        timeout: float = 1.0, 
        min_interval: float = 1.0
    ) -> Optional[GripperTeachingParam]:
        if not self._initialized or self._effector is None:
            return None
        
        param = self._effector.get_gripper_teaching_pendant_param(
            timeout=timeout, 
            min_interval=min_interval
        )
        if param is None:
            return None
        
        teaching_pendant_param = GripperTeachingParam(
            teaching_range_per=param.msg.teaching_range_per,
            max_range_config=param.msg.max_range_config,
            teaching_friction=param.msg.teaching_friction,
            hz=param.hz,
            timestamp=param.timestamp
        )
        return teaching_pendant_param

    def move_position(
        self,
        position: float = 0.0,
        force: float = 1.0,
        mode: Optional[str] = None,
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False

        if not (self.FORCE_MIN <= force <= self.FORCE_MAX):
            raise ValueError(
                f"force must be in range [{self.FORCE_MIN}, {self.FORCE_MAX}], current value: {force}"
            )

        try:
            value_i, force_i, command_mode = self._encode_position_command(
                position=position,
                force=force,
                mode=mode,
            )
            status_code = 0x05 if command_mode == "angle" else 0x01
            self._effector._send_msg(
                ArmMsgGripperCtrl(value=value_i, force=force_i, status_code=status_code)
            )
            return True
        except Exception as e:
            print(f"[AgxGripperWrapper] Control gripper failed: {e}")
            return False

    def command_position(
        self,
        position: float = 0.0,
        force: float = 0.0,
        mode: Optional[str] = None,
        enabled: bool = True,
        clear_error: bool = False,
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False

        try:
            value_i, force_i, command_mode = self._encode_position_command(
                position=position,
                force=force,
                mode=mode,
            )
            if command_mode == "angle":
                status_code = 0x05 if enabled else 0x04
                if clear_error:
                    status_code = 0x07 if enabled else 0x06
            else:
                status_code = 0x01 if enabled else 0x00
                if clear_error:
                    status_code = 0x03 if enabled else 0x02
            self._effector._send_msg(
                ArmMsgGripperCtrl(value=value_i, force=force_i, status_code=status_code)
            )
            return True
        except Exception as e:
            print(f"[AgxGripperWrapper] Raw gripper command failed: {e}")
            return False
    
    def move(self, width: float = 0.0, force: float = 1.0) -> bool:
        return self.move_position(position=width, force=force, mode="width")
    
    def open(self, width: float = 0.1, force: float = 1.0) -> bool:
        return self.move(width=width, force=force)
    
    def close(self, force: float = 1.0) -> bool:
        return self.move(width=0.0, force=force)
    
    def disable(self) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.disable_gripper()

    def reset(self) -> bool:
        if not self._initialized or self._effector is None:
            return False

        return self._effector.reset_gripper()

    def _encode_position_command(
        self,
        position: float,
        force: float,
        mode: Optional[str],
    ) -> Tuple[int, int, str]:
        if not (self.FORCE_MIN <= force <= self.FORCE_MAX):
            raise ValueError(
                f"force must be in range [{self.FORCE_MIN}, {self.FORCE_MAX}], current value: {force}"
            )

        command_mode = str(mode or "").strip().lower()
        if command_mode not in ("width", "angle"):
            command_mode = "angle" if abs(float(position)) > self.WIDTH_MAX else "width"

        if command_mode == "angle":
            angle = abs(float(position))
            if not (self.ANGLE_MIN <= angle <= self.ANGLE_MAX):
                raise ValueError(
                    f"angle must be in range [{self.ANGLE_MIN}, {self.ANGLE_MAX}], current value: {angle}"
                )
            return round(angle * 1e3), round(force * 1e3), "angle"

        width = abs(float(position))
        if not (self.WIDTH_MIN <= width <= self.WIDTH_MAX):
            raise ValueError(
                f"width must be in range [{self.WIDTH_MIN}, {self.WIDTH_MAX}], current value: {width}"
            )
        return round(width * 1e6), round(force * 1e3), "width"
    
    def calibrate(self, timeout: float = 1.0) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.calibrate_gripper(timeout=timeout)
    
    def set_teaching_param(
        self,
        teaching_range_per: int = 100,
        max_range_config: float = 0.0,
        teaching_friction: int = 1,
        timeout: float = 1.0
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.set_gripper_teaching_pendant_param(
            teaching_range_per=teaching_range_per,
            max_range_config=max_range_config,
            teaching_friction=teaching_friction,
            timeout=timeout
        )
