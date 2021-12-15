pub struct DumbFilter {
    value: f32,
    duration_cnt: usize,
}

impl DumbFilter {
    pub fn new(value: f32) -> Self {
        Self { value, duration_cnt: 0 }
    }

    pub fn update(&mut self, value: f32) {
        const DURATION_THRESHOLD: usize = 5;
        const NOISE_THRESHOLD: f32 = core::f32::consts::FRAC_PI_8;

        let diff = abs(self.value - value);
        if (diff > NOISE_THRESHOLD) & (diff < (core::f32::consts::TAU - NOISE_THRESHOLD)) {
            self.duration_cnt += 1;
            if self.duration_cnt < DURATION_THRESHOLD {
                return;
            }
        }
        self.duration_cnt = 0;
        self.value = value;
    }

    pub fn value(&self) -> f32 {
        self.value
    }
}

fn abs(value: f32) -> f32 {
    if value.is_sign_negative() { -value } else { value }
}
