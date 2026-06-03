/// Extended Kalman Filter for blind drone dead-reckoning.
///
/// State vector: [x, y, z, yaw] in cave-grid coordinates.
/// There is no measurement update step — the filter runs prediction-only,
/// so the state estimate equals dead-reckoning but the covariance P grows
/// over time, giving a principled measure of how lost the drone is.
pub struct Ekf {
    /// [x, y, z, yaw]
    pub state: [f64; 4],
    /// 4×4 covariance matrix (row-major)
    pub cov: [[f64; 4]; 4],
}

impl Ekf {
    /// Create a new EKF initialised at the given pose.
    /// `pos_std`: initial 1-σ position uncertainty (m).
    /// `yaw_std`: initial 1-σ heading uncertainty (rad).
    pub fn new(x: f64, y: f64, z: f64, yaw: f64, pos_std: f64, yaw_std: f64) -> Self {
        let p2 = pos_std * pos_std;
        let y2 = yaw_std * yaw_std;
        Self {
            state: [x, y, z, yaw],
            cov: [
                [p2,  0.0, 0.0, 0.0],
                [0.0, p2,  0.0, 0.0],
                [0.0, 0.0, p2,  0.0],
                [0.0, 0.0, 0.0, y2 ],
            ],
        }
    }

    /// EKF prediction step (no measurement update — blind navigation).
    ///
    /// Integrates the kinematic motion model and propagates the covariance
    /// through the linearised state-transition Jacobian F:
    ///
    ///   F = I  +  ∂f/∂x  evaluated at (v, yaw)
    ///
    /// Process noise is proportional to the commanded velocity so the
    /// covariance grows faster when the drone moves quickly.
    pub fn predict(&mut self, v: f64, vz: f64, omega: f64, dt: f64) {
        let [x, y, z, yaw] = self.state;

        // Motion model
        self.state = [
            x   + v * yaw.cos() * dt,
            y   + v * yaw.sin() * dt,
            z   + vz * dt,
            wrap_angle(yaw + omega * dt),
        ];

        // Linearised state-transition Jacobian
        let f = [
            [1.0, 0.0, 0.0, -v * yaw.sin() * dt],
            [0.0, 1.0, 0.0,  v * yaw.cos() * dt],
            [0.0, 0.0, 1.0,  0.0              ],
            [0.0, 0.0, 0.0,  1.0              ],
        ];

        // Diagonal process noise — ~5 % velocity error, ~2 % angular rate error
        let q_pos = (0.05 * v.abs() * dt).max(0.002);
        let q_yaw = (0.02 * omega.abs() * dt).max(0.001);
        let q = [
            [q_pos * q_pos, 0.0,           0.0,           0.0          ],
            [0.0,           q_pos * q_pos, 0.0,           0.0          ],
            [0.0,           0.0,           q_pos * q_pos, 0.0          ],
            [0.0,           0.0,           0.0,           q_yaw * q_yaw],
        ];

        // P ← F P Fᵀ + Q
        let fp   = mat4_mul(f, self.cov);
        let fpft = mat4_mul(fp, mat4_transpose(f));
        self.cov = mat4_add(fpft, q);
    }

    /// EKF measurement update with a full pose observation.
    ///
    /// Applies sequential scalar updates for each state dimension so no
    /// 4×4 matrix inversion is required.  The yaw innovation is wrapped to
    /// (−π, π] before being applied.
    ///
    /// * `pos_std` — 1-σ measurement noise for x/y/z (metres)
    /// * `yaw_std` — 1-σ measurement noise for yaw (radians)
    pub fn update(&mut self, x: f64, y: f64, z: f64, yaw: f64, pos_std: f64, yaw_std: f64) {
        let measurements = [x, y, z, yaw];
        let r = [
            pos_std * pos_std,
            pos_std * pos_std,
            pos_std * pos_std,
            yaw_std * yaw_std,
        ];

        for i in 0..4 {
            let s = self.cov[i][i] + r[i];
            if s < 1e-12 {
                continue;
            }
            // Scalar Kalman gain (column vector K = P[:,i] / S)
            let k = [
                self.cov[0][i] / s,
                self.cov[1][i] / s,
                self.cov[2][i] / s,
                self.cov[3][i] / s,
            ];
            // Innovation with yaw wrap-around for index 3
            let innov = if i == 3 {
                wrap_angle(measurements[3] - self.state[3])
            } else {
                measurements[i] - self.state[i]
            };
            for j in 0..4 {
                self.state[j] += k[j] * innov;
            }
            // Covariance update: P = (I - K·H)·P  →  subtract outer product K·P[i,:]
            let p_row = self.cov[i];
            for j in 0..4 {
                for l in 0..4 {
                    self.cov[j][l] -= k[j] * p_row[l];
                }
            }
        }
        self.state[3] = wrap_angle(self.state[3]);
    }

    /// RMS position uncertainty across x and y (metres, 1-σ).
    pub fn position_std(&self) -> f64 {
        ((self.cov[0][0] + self.cov[1][1]) * 0.5).sqrt()
    }

    pub fn x(&self)   -> f64 { self.state[0] }
    pub fn y(&self)   -> f64 { self.state[1] }
    pub fn z(&self)   -> f64 { self.state[2] }
    pub fn yaw(&self) -> f64 { self.state[3] }
}

fn wrap_angle(mut a: f64) -> f64 {
    while a > std::f64::consts::PI { a -= 2.0 * std::f64::consts::PI; }
    while a < -std::f64::consts::PI { a += 2.0 * std::f64::consts::PI; }
    a
}

// ── 4×4 matrix helpers ──────────────────────────────────────────────────────

fn mat4_mul(a: [[f64; 4]; 4], b: [[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut c = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

fn mat4_transpose(a: [[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut t = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            t[i][j] = a[j][i];
        }
    }
    t
}

fn mat4_add(a: [[f64; 4]; 4], b: [[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut c = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
    c
}
