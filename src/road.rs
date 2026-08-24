use crate::math::{Vec3, clamp01};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoadCell {
    pub temperature_k: f64,
    pub rubber: f64,
    pub water_depth_m: f64,
    pub contamination: f64,
}

impl Default for RoadCell {
    fn default() -> Self {
        Self { temperature_k: 298.15, rubber: 0.05, water_depth_m: 0.0, contamination: 0.0 }
    }
}

impl RoadCell {
    pub fn grip_scale(self) -> f64 {
        let rubber_gain = 1.0 + 0.22 * self.rubber.sqrt();
        let wet_loss = 1.0 - 0.55 * clamp01(self.water_depth_m / 0.003);
        rubber_gain * wet_loss * (1.0 - 0.45 * self.contamination)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct DynamicRoad {
    pub origin_x: f64,
    pub origin_z: f64,
    pub cell_size_m: f64,
    pub width: usize,
    pub height: usize,
    pub ambient_temperature_k: f64,
    pub solar_heating_w_m2: f64,
    cells: Vec<RoadCell>,
}

impl DynamicRoad {
    pub fn new(width: usize, height: usize, cell_size_m: f64) -> Self {
        Self {
            origin_x: -(width as f64) * cell_size_m * 0.5,
            origin_z: -(height as f64) * cell_size_m * 0.5,
            cell_size_m,
            width,
            height,
            ambient_temperature_k: 293.15,
            solar_heating_w_m2: 250.0,
            cells: vec![RoadCell::default(); width * height],
        }
    }
    fn index(&self, p: Vec3) -> Option<usize> {
        let x = ((p.x - self.origin_x) / self.cell_size_m).floor() as isize;
        let z = ((p.z - self.origin_z) / self.cell_size_m).floor() as isize;
        if x >= 0 && z >= 0 && x < self.width as isize && z < self.height as isize {
            Some(z as usize * self.width + x as usize)
        } else {
            None
        }
    }
    pub fn sample(&self, p: Vec3) -> RoadCell {
        self.index(p).map_or_else(RoadCell::default, |i| self.cells[i])
    }
    pub fn interact(&mut self, p: Vec3, slip_energy_j: f64, tire_temp_k: f64, dt: f64) {
        let Some(i) = self.index(p) else {
            return;
        };
        let c = &mut self.cells[i];
        c.rubber = (c.rubber + slip_energy_j * 1.0e-8).clamp(0.0, 1.0);
        c.temperature_k += (tire_temp_k - c.temperature_k) * 0.0008 * dt + slip_energy_j * 2.0e-6;
        c.water_depth_m = (c.water_depth_m - (0.00002 + slip_energy_j * 2.0e-10) * dt).max(0.0);
    }
    pub fn update_weather(&mut self, rain_rate_m_s: f64, dt: f64) {
        for c in &mut self.cells {
            c.water_depth_m = (c.water_depth_m + rain_rate_m_s * dt).clamp(0.0, 0.02);
            let solar = self.solar_heating_w_m2 * 3.0e-7;
            c.temperature_k += (self.ambient_temperature_k - c.temperature_k) * 0.002 * dt + solar * dt;
        }
    }
    pub fn set_uniform_water(&mut self, depth_m: f64) {
        for c in &mut self.cells {
            c.water_depth_m = depth_m.max(0.0);
        }
    }
    pub fn cells(&self) -> &[RoadCell] {
        &self.cells
    }
    pub(crate) fn replace_cells(&mut self, cells: Vec<RoadCell>) -> bool {
        if cells.len() != self.width * self.height {
            return false;
        }
        self.cells = cells;
        true
    }
}
