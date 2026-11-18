#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from typing_extensions import Final, Literal


class ArmModel:
    """Robot arm model identifiers.

    Use::

        from pyAgxArm import ArmModel
        create_agx_arm_config(robot=ArmModel.PIPER, ...)
    """

    NERO: Final[Literal["nero"]] = "nero"


class NeroFW:
    """Nero series firmware versions.

    ======== ========================
    Constant Firmware range
    ======== ========================
    DEFAULT  <= 1.10
    V111     == 1.11
    V112     >= 1.12
    ======== ========================

    Use::

        from pyAgxArm import NeroFW
        create_agx_arm_config(..., firmeware_version=NeroFW.V111)
    """

    DEFAULT: Final[Literal["default"]] = "default"
    V111: Final[Literal["v111"]] = "v111"
    V112: Final[Literal["v112"]] = "v112"
