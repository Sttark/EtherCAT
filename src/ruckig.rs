use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use thiserror::Error;

const POSITION_REBASE_THRESHOLD: f64 = 1_000_000_000.0;
const POSITION_EPSILON: f64 = 0.5;
const VELOCITY_EPSILON: f64 = 1.0e-3;
const ACCELERATION_EPSILON: f64 = 1.0e-2;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct RuckigConfig {
    pub enabled: bool,
    pub dt_ms: Option<f64>,
    pub max_velocity: Option<f64>,
    pub max_acceleration: Option<f64>,
    pub max_jerk: Option<f64>,
    pub velocity_lookahead_s: f64,
    pub hold_last_commanded_position: bool,
}

impl Default for RuckigConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            dt_ms: None,
            max_velocity: None,
            max_acceleration: None,
            max_jerk: None,
            velocity_lookahead_s: 0.5,
            hold_last_commanded_position: true,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct MotionLimitsOverrides {
    pub max_velocity: Option<f64>,
    pub max_acceleration: Option<f64>,
    pub max_jerk: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RuckigStep {
    pub position: i64,
    pub velocity: f64,
    pub acceleration: f64,
    pub done: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlannerDescription {
    pub active: bool,
    pub mode: Option<&'static str>,
    pub target_position: Option<i64>,
    pub target_velocity: Option<f64>,
    pub error: Option<String>,
    pub backend: &'static str,
}

#[derive(Debug, Clone, Error, PartialEq, Eq)]
#[error("{0}")]
pub struct RuckigUnavailable(pub String);

#[derive(Debug, Clone, Error, PartialEq, Eq)]
pub enum PlannerError {
    #[error(transparent)]
    Unavailable(#[from] RuckigUnavailable),
    #[error("{0}")]
    InvalidConfig(String),
    #[error("{0}")]
    Backend(String),
}

pub trait TrajectoryBackend: Send + Sync + 'static {
    fn backend_name(&self) -> &'static str;

    fn available(&self) -> bool {
        true
    }

    fn step(&mut self, state: &mut ActiveTrajectory) -> Result<RuckigStep, PlannerError>;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct PureRustTrajectoryBackend;

impl TrajectoryBackend for PureRustTrajectoryBackend {
    fn backend_name(&self) -> &'static str {
        "pure-rust-fallback"
    }

    fn step(&mut self, state: &mut ActiveTrajectory) -> Result<RuckigStep, PlannerError> {
        match state.mode {
            PlannerMode::Position { target_position } => step_position(state, target_position),
            PlannerMode::Velocity { target_velocity } => Ok(step_velocity(state, target_velocity)),
        }
    }
}

pub struct RuckigCspPlanner<B = PureRustTrajectoryBackend>
where
    B: TrajectoryBackend,
{
    backend: B,
    state: HashMap<i32, ActiveTrajectory>,
    last_error: HashMap<i32, String>,
}

impl Default for RuckigCspPlanner<PureRustTrajectoryBackend> {
    fn default() -> Self {
        Self::new()
    }
}

impl RuckigCspPlanner<PureRustTrajectoryBackend> {
    pub fn new() -> Self {
        Self::with_backend(PureRustTrajectoryBackend)
    }
}

impl<B> RuckigCspPlanner<B>
where
    B: TrajectoryBackend,
{
    pub fn with_backend(backend: B) -> Self {
        Self {
            backend,
            state: HashMap::new(),
            last_error: HashMap::new(),
        }
    }

    pub fn available(&self) -> bool {
        self.backend.available()
    }

    pub fn stop(&mut self, slave_pos: i32) {
        self.state.remove(&slave_pos);
    }

    pub fn is_active(&self, slave_pos: i32) -> bool {
        self.state.contains_key(&slave_pos)
    }

    pub fn describe(&self, slave_pos: i32) -> PlannerDescription {
        let state = self.state.get(&slave_pos);
        PlannerDescription {
            active: state.is_some(),
            mode: state.map(|trajectory| trajectory.mode.as_str()),
            target_position: state.and_then(|trajectory| trajectory.mode.target_position()),
            target_velocity: state.and_then(|trajectory| trajectory.mode.target_velocity()),
            error: state.and_then(|trajectory| trajectory.error.clone()),
            backend: self.backend.backend_name(),
        }
    }

    pub fn consume_last_error(&mut self, slave_pos: i32) -> Option<String> {
        self.last_error.remove(&slave_pos)
    }

    pub fn is_velocity_mode(&self, slave_pos: i32) -> bool {
        self.state
            .get(&slave_pos)
            .is_some_and(|trajectory| matches!(trajectory.mode, PlannerMode::Velocity { .. }))
    }

    pub fn update_target_velocity(&mut self, slave_pos: i32, target_velocity: f64) -> bool {
        let Some(state) = self.state.get_mut(&slave_pos) else {
            return false;
        };

        match &mut state.mode {
            PlannerMode::Velocity {
                target_velocity: current,
            } => {
                *current = target_velocity;
                true
            }
            PlannerMode::Position { .. } => false,
        }
    }

    pub fn start_position(
        &mut self,
        slave_pos: i32,
        actual_position: i64,
        actual_velocity: f64,
        target_position: i64,
        cfg: RuckigConfig,
        dt_s_fallback: f64,
        overrides: MotionLimitsOverrides,
        actual_acceleration: f64,
    ) -> Result<(), PlannerError> {
        self.require_backend()?;
        self.last_error.remove(&slave_pos);

        let dt = dt_s(&cfg, dt_s_fallback)?;
        let limits = resolve_limits(&cfg, overrides)?;

        self.state.insert(
            slave_pos,
            ActiveTrajectory {
                mode: PlannerMode::Position { target_position },
                state: KinematicState {
                    position: actual_position as f64,
                    velocity: actual_velocity,
                    acceleration: actual_acceleration,
                },
                limits,
                dt,
                lookahead_s: cfg.velocity_lookahead_s,
                position_offset: 0,
                error: None,
            },
        );

        Ok(())
    }

    pub fn start_velocity(
        &mut self,
        slave_pos: i32,
        actual_position: i64,
        actual_velocity: f64,
        target_velocity: f64,
        cfg: RuckigConfig,
        dt_s_fallback: f64,
        overrides: MotionLimitsOverrides,
        actual_acceleration: f64,
    ) -> Result<(), PlannerError> {
        self.require_backend()?;
        self.last_error.remove(&slave_pos);

        let dt = dt_s(&cfg, dt_s_fallback)?;
        let limits = resolve_limits(&cfg, overrides)?;
        let lookahead_s = cfg.velocity_lookahead_s;
        if !(lookahead_s.is_finite() && lookahead_s > 0.0) {
            return Err(PlannerError::InvalidConfig(
                "velocity_lookahead_s must be > 0.".to_string(),
            ));
        }

        self.state.insert(
            slave_pos,
            ActiveTrajectory {
                mode: PlannerMode::Velocity { target_velocity },
                state: KinematicState {
                    position: actual_position as f64,
                    velocity: actual_velocity,
                    acceleration: actual_acceleration,
                },
                limits,
                dt,
                lookahead_s,
                position_offset: 0,
                error: None,
            },
        );

        Ok(())
    }

    pub fn step(
        &mut self,
        slave_pos: i32,
        _actual_position: Option<i64>,
        _actual_velocity: Option<f64>,
    ) -> Option<RuckigStep> {
        let mut done_position_move = false;
        let result = {
            let state = self.state.get_mut(&slave_pos)?;
            state.error = None;
            maybe_rebase_position(state);

            match self.backend.step(state) {
                Ok(mut step) => {
                    maybe_rebase_position(state);
                    step.position = step.position.saturating_add(state.position_offset);
                    done_position_move =
                        step.done && matches!(state.mode, PlannerMode::Position { .. });
                    Some(step)
                }
                Err(error) => {
                    let message = error.to_string();
                    state.error = Some(message.clone());
                    self.last_error.insert(slave_pos, message);
                    None
                }
            }
        };

        match result {
            Some(step) => {
                if done_position_move {
                    self.state.remove(&slave_pos);
                }
                Some(step)
            }
            None => {
                self.state.remove(&slave_pos);
                None
            }
        }
    }

    fn require_backend(&self) -> Result<(), PlannerError> {
        if self.available() {
            Ok(())
        } else {
            Err(RuckigUnavailable(format!(
                "{} backend is not available in this build.",
                self.backend.backend_name()
            ))
            .into())
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct MotionLimits {
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
}

#[derive(Debug, Clone, Copy)]
struct KinematicState {
    position: f64,
    velocity: f64,
    acceleration: f64,
}

#[derive(Debug, Clone)]
pub struct ActiveTrajectory {
    mode: PlannerMode,
    state: KinematicState,
    limits: MotionLimits,
    dt: f64,
    lookahead_s: f64,
    position_offset: i64,
    error: Option<String>,
}

#[derive(Debug, Clone, Copy)]
enum PlannerMode {
    Position { target_position: i64 },
    Velocity { target_velocity: f64 },
}

impl PlannerMode {
    fn as_str(self) -> &'static str {
        match self {
            Self::Position { .. } => "position",
            Self::Velocity { .. } => "velocity",
        }
    }

    fn target_position(self) -> Option<i64> {
        match self {
            Self::Position { target_position } => Some(target_position),
            Self::Velocity { .. } => None,
        }
    }

    fn target_velocity(self) -> Option<f64> {
        match self {
            Self::Position { .. } => None,
            Self::Velocity { target_velocity } => Some(target_velocity),
        }
    }
}

fn dt_s(cfg: &RuckigConfig, fallback_dt_s: f64) -> Result<f64, PlannerError> {
    let dt = cfg
        .dt_ms
        .map(|value| value / 1000.0)
        .unwrap_or(fallback_dt_s);
    if !(dt.is_finite() && dt > 0.0) {
        return Err(PlannerError::InvalidConfig(
            "Ruckig timestep must be > 0 seconds.".to_string(),
        ));
    }
    Ok(dt)
}

fn resolve_limits(
    cfg: &RuckigConfig,
    overrides: MotionLimitsOverrides,
) -> Result<MotionLimits, PlannerError> {
    let max_velocity = overrides.max_velocity.or(cfg.max_velocity).unwrap_or(0.0);
    let max_acceleration = overrides
        .max_acceleration
        .or(cfg.max_acceleration)
        .unwrap_or(0.0);
    let max_jerk = overrides.max_jerk.or(cfg.max_jerk).unwrap_or(0.0);

    if !(max_velocity.is_finite() && max_velocity > 0.0) {
        return Err(PlannerError::InvalidConfig(
            "Ruckig limits must be > 0 (max_velocity/max_acceleration/max_jerk).".to_string(),
        ));
    }
    if !(max_acceleration.is_finite() && max_acceleration > 0.0) {
        return Err(PlannerError::InvalidConfig(
            "Ruckig limits must be > 0 (max_velocity/max_acceleration/max_jerk).".to_string(),
        ));
    }
    if !(max_jerk.is_finite() && max_jerk > 0.0) {
        return Err(PlannerError::InvalidConfig(
            "Ruckig limits must be > 0 (max_velocity/max_acceleration/max_jerk).".to_string(),
        ));
    }

    Ok(MotionLimits {
        max_velocity,
        max_acceleration,
        max_jerk,
    })
}

fn maybe_rebase_position(state: &mut ActiveTrajectory) {
    if matches!(state.mode, PlannerMode::Velocity { .. })
        && state.state.position.abs() > POSITION_REBASE_THRESHOLD
    {
        let shift = state.state.position.round() as i64;
        state.state.position -= shift as f64;
        state.position_offset = state.position_offset.saturating_add(shift);
    }
}

fn step_velocity(state: &mut ActiveTrajectory, target_velocity: f64) -> RuckigStep {
    let desired_velocity = clamp(
        target_velocity,
        -state.limits.max_velocity,
        state.limits.max_velocity,
    );
    let jerk_window = state.limits.max_jerk * state.dt;
    let velocity_horizon = state.lookahead_s.max(state.dt);
    let desired_acceleration = clamp(
        (desired_velocity - state.state.velocity) / velocity_horizon,
        -state.limits.max_acceleration,
        state.limits.max_acceleration,
    );

    state.state.acceleration += clamp(
        desired_acceleration - state.state.acceleration,
        -jerk_window,
        jerk_window,
    );
    state.state.acceleration = clamp(
        state.state.acceleration,
        -state.limits.max_acceleration,
        state.limits.max_acceleration,
    );

    integrate_state(&mut state.state, state.dt, state.limits.max_velocity);

    let done = (state.state.velocity - desired_velocity).abs() <= VELOCITY_EPSILON
        && state.state.acceleration.abs() <= ACCELERATION_EPSILON;

    RuckigStep {
        position: state.state.position.round() as i64,
        velocity: state.state.velocity,
        acceleration: state.state.acceleration,
        done,
    }
}

fn step_position(
    state: &mut ActiveTrajectory,
    target_position: i64,
) -> Result<RuckigStep, PlannerError> {
    let target_position = target_position as f64;
    let error = target_position - state.state.position;
    let prior_error_sign = signum_nonzero(error);

    if error.abs() <= POSITION_EPSILON
        && state.state.velocity.abs() <= VELOCITY_EPSILON
        && state.state.acceleration.abs() <= ACCELERATION_EPSILON
    {
        state.state.position = target_position;
        state.state.velocity = 0.0;
        state.state.acceleration = 0.0;
        return Ok(RuckigStep {
            position: target_position.round() as i64,
            velocity: 0.0,
            acceleration: 0.0,
            done: true,
        });
    }

    if !error.is_finite() {
        return Err(PlannerError::Backend(
            "non-finite position state encountered".to_string(),
        ));
    }

    let stopping_distance = (state.state.velocity * state.state.velocity)
        / (2.0 * state.limits.max_acceleration)
        + state.state.velocity.abs() * state.dt;

    let travel_direction = if error.abs() > POSITION_EPSILON {
        error.signum()
    } else {
        -state.state.velocity.signum()
    };

    let approach_speed = if error.abs() <= stopping_distance {
        (2.0 * state.limits.max_acceleration * error.abs()).sqrt()
    } else {
        state.limits.max_velocity
    };

    let desired_velocity = clamp(
        travel_direction * approach_speed,
        -state.limits.max_velocity,
        state.limits.max_velocity,
    );
    let desired_acceleration = clamp(
        (desired_velocity - state.state.velocity) / state.dt,
        -state.limits.max_acceleration,
        state.limits.max_acceleration,
    );
    let jerk_window = state.limits.max_jerk * state.dt;

    state.state.acceleration += clamp(
        desired_acceleration - state.state.acceleration,
        -jerk_window,
        jerk_window,
    );
    state.state.acceleration = clamp(
        state.state.acceleration,
        -state.limits.max_acceleration,
        state.limits.max_acceleration,
    );

    let previous_position = state.state.position;
    integrate_state(&mut state.state, state.dt, state.limits.max_velocity);
    let next_error = target_position - state.state.position;

    let crossed_target = prior_error_sign != 0.0
        && signum_nonzero(next_error) != 0.0
        && signum_nonzero(next_error) != prior_error_sign;

    if crossed_target
        || (next_error.abs() <= POSITION_EPSILON
            && state.state.velocity.abs() <= state.limits.max_velocity.min(1.0))
        || (previous_position - target_position).abs() <= POSITION_EPSILON
    {
        state.state.position = target_position;
        state.state.velocity = 0.0;
        state.state.acceleration = 0.0;
        return Ok(RuckigStep {
            position: target_position.round() as i64,
            velocity: 0.0,
            acceleration: 0.0,
            done: true,
        });
    }

    Ok(RuckigStep {
        position: state.state.position.round() as i64,
        velocity: state.state.velocity,
        acceleration: state.state.acceleration,
        done: false,
    })
}

fn integrate_state(state: &mut KinematicState, dt: f64, max_velocity: f64) {
    state.position += state.velocity * dt + 0.5 * state.acceleration * dt * dt;
    state.velocity = clamp(
        state.velocity + state.acceleration * dt,
        -max_velocity,
        max_velocity,
    );
}

fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

fn signum_nonzero(value: f64) -> f64 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> RuckigConfig {
        RuckigConfig {
            enabled: true,
            dt_ms: Some(10.0),
            max_velocity: Some(2_000.0),
            max_acceleration: Some(5_000.0),
            max_jerk: Some(50_000.0),
            velocity_lookahead_s: 0.5,
            hold_last_commanded_position: true,
        }
    }

    #[test]
    fn position_mode_converges() {
        let mut planner = RuckigCspPlanner::new();
        planner
            .start_position(
                1,
                0,
                0.0,
                10_000,
                cfg(),
                0.01,
                MotionLimitsOverrides::default(),
                0.0,
            )
            .unwrap();

        let mut last = None;
        for _ in 0..2_000 {
            last = planner.step(1, None, None);
            if last.is_some_and(|step| step.done) {
                break;
            }
        }

        let step = last.expect("planner should emit steps");
        assert!(step.done);
        assert_eq!(step.position, 10_000);
        assert!(!planner.is_active(1));
    }

    #[test]
    fn velocity_mode_updates_target() {
        let mut planner = RuckigCspPlanner::new();
        planner
            .start_velocity(
                2,
                0,
                0.0,
                500.0,
                cfg(),
                0.01,
                MotionLimitsOverrides::default(),
                0.0,
            )
            .unwrap();

        assert!(planner.is_velocity_mode(2));
        assert!(planner.update_target_velocity(2, 750.0));

        let mut latest = None;
        for _ in 0..50 {
            latest = planner.step(2, None, None);
        }

        let step = latest.expect("planner should emit steps");
        assert!(step.velocity > 0.0);
        assert!(planner.is_active(2));
    }
}
