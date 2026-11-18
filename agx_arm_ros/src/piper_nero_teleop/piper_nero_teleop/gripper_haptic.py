from dataclasses import dataclass
import math
from typing import Optional


HAPTIC_FREE_FOLLOW = "FREE_FOLLOW"
HAPTIC_ACTIVE = "ACTIVE"
HAPTIC_OPENING_RELEASE = "OPENING_RELEASE"
HAPTIC_STALE = "STALE"
HAPTIC_MISMATCH_PULSE = "MISMATCH_PULSE"
HAPTIC_SPRING_ALIGN = "SPRING_ALIGN"
HAPTIC_SPRING_HOLD = "SPRING_HOLD"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _finite(value: float) -> bool:
    return math.isfinite(float(value))


@dataclass(frozen=True)
class CueHapticConfig:
    enabled: bool
    effort_sign: float
    effort_bias: float
    effort_alpha: float
    effort_deadband: float
    effort_max: float
    cmd_max: int
    cmd_enable_threshold: int
    release_when_opening: bool
    opening_relief: float
    opening_sign: float
    opening_direction_eps: float
    force_max_n: float
    force_ramp_up_n_per_sec: float
    force_ramp_down_n_per_sec: float
    max_stale_dt_sec: float = 0.25


@dataclass(frozen=True)
class CueHapticResult:
    force: float
    enabled: bool
    cmd_raw: int
    state: str
    filtered_effort: float
    opening_detected: bool
    target_opening: Optional[float] = None


def constant_resistance_result(
    *,
    force_n: float,
    force_max_n: float,
    cmd_max: int,
    leader_opening_delta: float,
    directional: bool,
    opening_direction_eps: float,
    directional_active: Optional[bool] = None,
) -> CueHapticResult:
    force = max(0.0, float(force_n))
    if force <= 0.0:
        return CueHapticResult(
            force=0.0,
            enabled=False,
            cmd_raw=0,
            state=HAPTIC_FREE_FOLLOW,
            filtered_effort=0.0,
            opening_detected=False,
        )

    if bool(directional):
        if not _finite(leader_opening_delta):
            return CueHapticResult(
                force=0.0,
                enabled=False,
                cmd_raw=0,
                state=HAPTIC_STALE,
                filtered_effort=0.0,
                opening_detected=False,
            )
        eps = max(0.0, float(opening_direction_eps))
        closing = (
            bool(directional_active)
            if directional_active is not None
            else float(leader_opening_delta) < -eps
        )
        if not closing:
            opening_release = float(leader_opening_delta) > eps
            return CueHapticResult(
                force=0.0,
                enabled=False,
                cmd_raw=0,
                state=HAPTIC_OPENING_RELEASE if opening_release else HAPTIC_FREE_FOLLOW,
                filtered_effort=0.0,
                opening_detected=opening_release,
            )

    max_force = max(1e-6, float(force_max_n))
    cmd_raw = int(round(clamp(force / max_force, 0.0, 1.0) * max(0, int(cmd_max))))
    return CueHapticResult(
        force=force,
        enabled=True,
        cmd_raw=cmd_raw,
        state=HAPTIC_ACTIVE,
        filtered_effort=0.0,
        opening_detected=False,
    )


def mismatch_pulse_result(
    *,
    leader_opening: float,
    follower_opening: float,
    fresh_follower_opening: bool,
    threshold_m: float,
    reset_threshold_m: float,
    pulse_sec: float,
    force_n: float,
    force_max_n: float,
    cmd_max: int,
    time_sec: float,
    active_until_sec: float,
    armed: bool,
    follower_force: float = 0.0,
    fresh_follower_force: bool = True,
    force_threshold_n: float = 0.0,
    closing_only: bool = False,
    retrigger_sec: float = 0.0,
):
    if not fresh_follower_opening:
        return (
            CueHapticResult(
                force=0.0,
                enabled=False,
                cmd_raw=0,
                state=HAPTIC_STALE,
                filtered_effort=0.0,
                opening_detected=False,
            ),
            0.0,
            True,
            float("nan"),
        )

    if not all(
        _finite(value)
        for value in (
            leader_opening,
            follower_opening,
            follower_force,
            force_threshold_n,
            threshold_m,
            reset_threshold_m,
            pulse_sec,
            force_n,
            force_max_n,
            time_sec,
            active_until_sec,
        )
    ):
        return (
            CueHapticResult(
                force=0.0,
                enabled=False,
                cmd_raw=0,
                state=HAPTIC_STALE,
                filtered_effort=0.0,
                opening_detected=False,
            ),
            0.0,
            True,
            float("nan"),
        )

    now = float(time_sec)
    threshold = max(0.0, float(threshold_m))
    reset_threshold = min(threshold, max(0.0, float(reset_threshold_m)))
    force_threshold = max(0.0, float(force_threshold_n))
    pulse_duration = max(0.0, float(pulse_sec))
    signed_error = float(follower_opening) - float(leader_opening)
    abs_error = abs(signed_error)
    abs_follower_force = abs(float(follower_force))
    next_armed = bool(armed)
    next_active_until = max(0.0, float(active_until_sec))

    if now < next_active_until:
        force = clamp(float(force_n), 0.0, float(force_max_n))
        max_force = max(1e-6, float(force_max_n))
        cmd_raw = int(round(clamp(force / max_force, 0.0, 1.0) * max(0, int(cmd_max))))
        return (
            CueHapticResult(
                force=force,
                enabled=force > 0.0,
                cmd_raw=cmd_raw,
                state=HAPTIC_MISMATCH_PULSE,
                filtered_effort=abs_error,
                opening_detected=False,
            ),
            next_active_until,
            next_armed,
            abs_error,
        )

    error_ready = (
        signed_error >= threshold if bool(closing_only) else abs_error >= threshold
    )
    error_recovered = (
        signed_error <= reset_threshold if bool(closing_only) else abs_error <= reset_threshold
    )

    if error_recovered:
        next_armed = True

    force_ready = force_threshold <= 0.0 or (
        bool(fresh_follower_force) and abs_follower_force >= force_threshold
    )
    cooldown_ready = (
        max(0.0, float(retrigger_sec)) > 0.0
        and next_active_until > 0.0
        and now >= next_active_until + max(0.0, float(retrigger_sec))
    )
    if (
        threshold > 0.0
        and pulse_duration > 0.0
        and (next_armed or cooldown_ready)
        and error_ready
        and force_ready
    ):
        next_active_until = now + pulse_duration
        next_armed = False

    if now < next_active_until:
        force = clamp(float(force_n), 0.0, float(force_max_n))
        max_force = max(1e-6, float(force_max_n))
        cmd_raw = int(round(clamp(force / max_force, 0.0, 1.0) * max(0, int(cmd_max))))
        return (
            CueHapticResult(
                force=force,
                enabled=force > 0.0,
                cmd_raw=cmd_raw,
                state=HAPTIC_MISMATCH_PULSE,
                filtered_effort=abs_error,
                opening_detected=False,
            ),
            next_active_until,
            next_armed,
            abs_error,
        )

    return (
        CueHapticResult(
            force=0.0,
            enabled=False,
            cmd_raw=0,
            state=HAPTIC_FREE_FOLLOW,
            filtered_effort=abs_error,
            opening_detected=False,
        ),
        next_active_until,
        next_armed,
        abs_error,
    )


def mismatch_force_ready(
    *,
    follower_force: float,
    fresh_follower_force: bool,
    force_threshold_n: float,
    confirm_sec: float,
    time_sec: float,
    force_ready_since_sec: float,
    force_sign: float = 0.0,
):
    threshold = max(0.0, float(force_threshold_n))
    if threshold <= 0.0:
        return True, 0.0
    if not bool(fresh_follower_force):
        return False, 0.0
    if not all(_finite(value) for value in (follower_force, confirm_sec, time_sec)):
        return False, 0.0
    force = float(follower_force)
    sign = float(force_sign) if _finite(force_sign) else 0.0
    if sign < 0.0:
        force_active = force <= -threshold
    elif sign > 0.0:
        force_active = force >= threshold
    else:
        force_active = abs(force) >= threshold
    if not force_active:
        return False, 0.0

    now = float(time_sec)
    since = float(force_ready_since_sec) if _finite(force_ready_since_sec) else 0.0
    if since <= 0.0 or since > now:
        since = now
    ready = (now - since) >= max(0.0, float(confirm_sec))
    return ready, since


def spring_align_result(
    *,
    leader_opening: float,
    follower_opening: float,
    fresh_follower_opening: bool,
    force_ready: bool,
    threshold_m: float,
    reset_threshold_m: float,
    spring_k_n_per_m: float,
    min_force_n: float,
    force_max_n: float,
    cmd_max: int,
    active: bool,
):
    if not fresh_follower_opening:
        return CueHapticResult(
            force=0.0,
            enabled=False,
            cmd_raw=0,
            state=HAPTIC_STALE,
            filtered_effort=0.0,
            opening_detected=False,
        )

    if not all(
        _finite(value)
        for value in (
            leader_opening,
            follower_opening,
            threshold_m,
            reset_threshold_m,
            spring_k_n_per_m,
            min_force_n,
            force_max_n,
        )
    ):
        return CueHapticResult(
            force=0.0,
            enabled=False,
            cmd_raw=0,
            state=HAPTIC_STALE,
            filtered_effort=0.0,
            opening_detected=False,
        )

    signed_error = float(follower_opening) - float(leader_opening)
    abs_error = abs(signed_error)
    threshold = max(0.0, float(threshold_m))
    reset_threshold = min(threshold, max(0.0, float(reset_threshold_m)))
    error_ready = abs_error >= (reset_threshold if bool(active) else threshold)

    if not (bool(force_ready) and threshold > 0.0 and error_ready):
        return CueHapticResult(
            force=0.0,
            enabled=False,
            cmd_raw=0,
            state=HAPTIC_FREE_FOLLOW,
            filtered_effort=abs_error,
            opening_detected=False,
        )

    max_force = max(1e-6, float(force_max_n))
    force = clamp(
        max(0.0, float(min_force_n)) + max(0.0, float(spring_k_n_per_m)) * abs_error,
        0.0,
        max_force,
    )
    cmd_raw = int(round(clamp(force / max_force, 0.0, 1.0) * max(0, int(cmd_max))))
    return CueHapticResult(
        force=force,
        enabled=force > 0.0,
        cmd_raw=cmd_raw,
        state=HAPTIC_SPRING_ALIGN,
        filtered_effort=abs_error,
        opening_detected=False,
        target_opening=float(follower_opening),
    )


def spring_hold_result(
    *,
    leader_opening: float,
    follower_opening: float,
    hold_opening: float,
    fresh_follower_opening: bool,
    force_n: float,
    force_max_n: float,
    cmd_max: int,
):
    if not fresh_follower_opening:
        return CueHapticResult(
            force=0.0,
            enabled=False,
            cmd_raw=0,
            state=HAPTIC_STALE,
            filtered_effort=0.0,
            opening_detected=False,
        )

    if not all(
        _finite(value)
        for value in (leader_opening, follower_opening, hold_opening, force_n, force_max_n)
    ):
        return CueHapticResult(
            force=0.0,
            enabled=False,
            cmd_raw=0,
            state=HAPTIC_STALE,
            filtered_effort=0.0,
            opening_detected=False,
        )

    abs_error = abs(float(follower_opening) - float(leader_opening))
    max_force = max(1e-6, float(force_max_n))
    force = clamp(float(force_n), 0.0, max_force)
    cmd_raw = int(round(clamp(force / max_force, 0.0, 1.0) * max(0, int(cmd_max))))
    return CueHapticResult(
        force=force,
        enabled=force > 0.0,
        cmd_raw=cmd_raw,
        state=HAPTIC_SPRING_HOLD,
        filtered_effort=abs_error,
        opening_detected=False,
        target_opening=float(hold_opening),
    )


class CueHapticController:
    """ROS1-structure cue-only gripper haptic for Nero.

    The leader gripper never tracks follower opening. The cue only modulates
    local leader gripper force/enable while leader position remains a self-loop
    based on the leader's own measured opening.
    """

    def __init__(self, config: CueHapticConfig):
        self.config = config
        self.reset()

    def reset(self):
        self._last_time: Optional[float] = None
        self._filtered_effort = 0.0
        self._filter_initialized = False
        self._last_force_cmd = 0.0

    def update(
        self,
        *,
        leader_opening: float,
        leader_opening_delta: float,
        follower_force: float,
        time_sec: float,
        fresh_follower_force: bool,
    ) -> CueHapticResult:
        if not self.config.enabled:
            self.reset()
            return self._result(
                force=0.0,
                enabled=False,
                cmd_raw=0,
                state=HAPTIC_FREE_FOLLOW,
                filtered_effort=0.0,
                opening_release=False,
            )

        if not all(
            _finite(value)
            for value in (leader_opening, leader_opening_delta, follower_force, time_sec)
        ):
            self.reset()
            return self._result(
                force=0.0,
                enabled=False,
                cmd_raw=0,
                state=HAPTIC_STALE,
                filtered_effort=0.0,
                opening_release=False,
            )

        if not fresh_follower_force:
            self.reset()
            self._last_time = float(time_sec)
            return self._result(
                force=0.0,
                enabled=False,
                cmd_raw=0,
                state=HAPTIC_STALE,
                filtered_effort=0.0,
                opening_release=False,
            )

        dt = 0.0
        if self._last_time is not None:
            dt = float(time_sec) - float(self._last_time)
            if dt <= 1e-6 or dt > max(0.05, float(self.config.max_stale_dt_sec)):
                self.reset()
                self._last_time = float(time_sec)
                return self._result(
                    force=0.0,
                    enabled=False,
                    cmd_raw=0,
                    state=HAPTIC_STALE,
                    filtered_effort=0.0,
                    opening_release=False,
                )

        signed_effort = float(follower_force) * float(self.config.effort_sign)
        active_effort = max(0.0, signed_effort - float(self.config.effort_bias))

        if not self._filter_initialized:
            filtered_effort = active_effort
            self._filter_initialized = True
        else:
            alpha = clamp(float(self.config.effort_alpha), 0.0, 1.0)
            filtered_effort = alpha * active_effort + (1.0 - alpha) * self._filtered_effort
        self._filtered_effort = filtered_effort
        self._last_time = float(time_sec)

        cmd_raw = 0
        if (
            filtered_effort > float(self.config.effort_deadband)
            and int(self.config.cmd_max) > 0
        ):
            clipped_effort = min(filtered_effort, float(self.config.effort_max))
            span = max(
                1e-6,
                float(self.config.effort_max) - float(self.config.effort_deadband),
            )
            normalized = clamp(
                (clipped_effort - float(self.config.effort_deadband)) / span,
                0.0,
                1.0,
            )
            cmd_raw = int(round(normalized * int(self.config.cmd_max)))

        opening_release = (
            bool(self.config.release_when_opening)
            and leader_opening_delta * float(self.config.opening_sign)
            > float(self.config.opening_direction_eps)
        )
        if opening_release and cmd_raw > 0:
            cmd_raw = int(round(cmd_raw * float(self.config.opening_relief)))

        above_threshold = (
            cmd_raw >= int(self.config.cmd_enable_threshold)
            and int(self.config.cmd_max) > 0
            and float(self.config.force_max_n) > 0.0
        )
        if above_threshold:
            target_force = (
                clamp(float(cmd_raw) / float(int(self.config.cmd_max)), 0.0, 1.0)
                * float(self.config.force_max_n)
            )
        else:
            # Below threshold (e.g., opening_release just dropped cmd_raw).
            # Target zero, but ramp force_cmd down from its current value so
            # the motor releases smoothly instead of snapping off.
            target_force = 0.0

        force_cmd = self._apply_force_ramp(force_cmd=target_force, dt=dt)

        if force_cmd <= 1e-4:
            self._last_force_cmd = 0.0
            return self._result(
                force=0.0,
                enabled=False,
                cmd_raw=int(cmd_raw),
                state=HAPTIC_OPENING_RELEASE if opening_release else HAPTIC_FREE_FOLLOW,
                filtered_effort=filtered_effort,
                opening_release=opening_release,
            )

        return self._result(
            force=force_cmd,
            enabled=True,
            cmd_raw=int(cmd_raw),
            state=HAPTIC_ACTIVE,
            filtered_effort=filtered_effort,
            opening_release=opening_release,
        )

    def _apply_force_ramp(self, *, force_cmd: float, dt: float) -> float:
        target_force = clamp(float(force_cmd), 0.0, float(self.config.force_max_n))
        previous_force = clamp(float(self._last_force_cmd), 0.0, float(self.config.force_max_n))
        if dt <= 0.0:
            previous_force = target_force
        elif target_force > previous_force:
            ramp_up = max(0.0, float(self.config.force_ramp_up_n_per_sec))
            if ramp_up > 0.0:
                previous_force = min(target_force, previous_force + ramp_up * dt)
            else:
                previous_force = target_force
        elif target_force < previous_force:
            ramp_down = max(0.0, float(self.config.force_ramp_down_n_per_sec))
            if ramp_down > 0.0:
                previous_force = max(target_force, previous_force - ramp_down * dt)
            else:
                previous_force = target_force
        self._last_force_cmd = previous_force
        return previous_force

    def _result(
        self,
        *,
        force: float,
        enabled: bool,
        cmd_raw: int,
        state: str,
        filtered_effort: float,
        opening_release: bool,
    ) -> CueHapticResult:
        return CueHapticResult(
            force=float(force),
            enabled=bool(enabled),
            cmd_raw=int(cmd_raw),
            state=str(state),
            filtered_effort=float(filtered_effort),
            opening_detected=bool(opening_release),
        )


# Compatibility aliases for nearby code/tests while the runtime is being
# simplified from SpringAlign to cue-only haptic.
GripperHapticConfig = CueHapticConfig
GripperHapticCommandResult = CueHapticResult
PiperStyleGripperHapticController = CueHapticController
