use pid::Pid;

pub enum Direction {
    Right,
    Left,
    Straight,
}

type TargetAltitude = f32;
type AutoPilotMode = (Direction, TargetAltitude);

pub struct AutoPilot {
    mode: Option<AutoPilotMode>,
    roll_controller: Pid<f32>,
    pitch_controller: Pid<f32>,
    pitch_output: f32,
    prev_altitude: f32,
}

impl AutoPilot {
    pub fn new() -> Self {
        let roll_controller = Pid::<f32>::new(1.5, 1.0, 20.0, 1.0, 0.7, 0.3, 1.0, 0.0);
        let pitch_controller = Pid::<f32>::new(0.5, 0.1, 50.0, 0.5, 0.3, 0.5, 1.0, 0.0);
        Self { mode: None, roll_controller, pitch_controller, pitch_output: 0.0, prev_altitude: 0.0 }
    }

    pub fn stop(&mut self) {
        self.mode = None;
    }

    pub fn set_mode(&mut self, mode: AutoPilotMode) {
        if self.mode.is_none() {
            self.roll_controller.reset_integral_term();
            self.roll_controller.reset_derivative_term();
            self.pitch_controller.reset_integral_term();
            self.pitch_controller.reset_derivative_term();
        }
        self.roll_controller.setpoint = match &mode.0 {
            Direction::Right => core::f32::consts::FRAC_PI_4,
            Direction::Left => -core::f32::consts::FRAC_PI_4,
            Direction::Straight => 0.0,
        };
        self.mode = Some(mode);
    }

    pub fn control(&mut self, roll: f32, pitch: Option<f32>, yaw: f32, altitude: f32) -> (f32, f32, f32, f32) {
        if self.mode.is_none() {
            return (0.0, 0.0, 0.0, 0.0);
        }
        let (target_direction, target_altitude) = self.mode.as_ref().unwrap();
        let roll_output = self.roll_controller.next_control_output(roll).output;
        let altitude_diff = altitude - target_altitude;
        if altitude_diff > 0.05 {
            self.pitch_controller.setpoint = 0.0;
        } else if altitude_diff < -0.05 {
            self.pitch_controller.setpoint = core::f32::consts::FRAC_PI_6;
        }
        if let Some(pitch) = pitch {
            self.pitch_output = self.pitch_controller.next_control_output(pitch).output;
            if (altitude_diff < -0.05) & (altitude < self.prev_altitude) {
                self.pitch_output = (self.pitch_output + (self.prev_altitude - altitude) * 10.0).min(1.0);
            }
            self.prev_altitude = altitude;
        }
        (roll_output, self.pitch_output, 0.275 + self.pitch_output * 0.05, roll_output)
    }
}
