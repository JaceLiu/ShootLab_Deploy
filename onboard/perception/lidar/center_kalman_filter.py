import numpy as np


class CenterKalmanFilter:
    """Simple 3D constant-velocity Kalman filter for ball center."""

    def __init__(self):
        # State: [x, y, z, vx, vy, vz]
        self.x = np.zeros((6, 1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * 1.0
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0
        self.Q_base = np.diag([0.01, 0.01, 0.01, 0.3, 0.3, 0.3]).astype(np.float64)
        self.R = np.diag([0.02, 0.02, 0.02]).astype(np.float64)
        self.initialized = False
        self.max_jump = 0.8  # measurement gating distance [m]
        self._max_predict_dt = 0.2  # cap integration step [s] after perception gaps
        # Missing-measurement handling
        self._miss_time = 0.0        # seconds accumulated without measurements
        self._freeze_after = 2     # seconds to tolerate before freezing velocity
        self._frozen = False

    def freeze_motion(self):
        """Zero velocity when perception is absent — avoids ballistic drift and huge
        single-step predicts when measurements resume."""
        self.x[3:6, 0] = 0.0
        self._frozen = True

    def _state_transition(self, dt: float):
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    def predict(self, dt: float):
        F = self._state_transition(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q_base

    def update(self, z: np.ndarray):
        z = np.asarray(z, dtype=np.float64).reshape(3, 1)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(6, dtype=np.float64)
        self.P = (I - K @ self.H) @ self.P

    def reset(self, z: np.ndarray):
        z = np.asarray(z, dtype=np.float64).reshape(3,)
        self.x[:] = 0.0
        self.x[0, 0], self.x[1, 0], self.x[2, 0] = z[0], z[1], z[2]
        self.P = np.eye(6, dtype=np.float64) * 0.5
        self.initialized = True

    def step(self, z: np.ndarray, dt: float):
        z = np.asarray(z, dtype=np.float64).reshape(3,)
        if not self.initialized:
            self.reset(z)
            self._miss_time = 0.0
            self._frozen = False
            return self.position

        dt = float(np.clip(dt, 1e-3, self._max_predict_dt))
        self.predict(dt)
        innov = np.linalg.norm(z - self.position)
        if innov <= self.max_jump:
            self.update(z)
        else:
            # Re-acquire after drift / outlier: snap to measurement (same idea as fused KF)
            self.reset(z)
        # Measurement received → reset missing timer / unfreeze
        self._miss_time = 0.0
        self._frozen = False
        return self.position

    def predict_only(self, dt: float):
        """Handle a cycle with no measurement.

        - If missing duration ≤ _freeze_after: keep normal predict (retain velocity).
        - If missing duration  > _freeze_after: freeze velocity once, then keep predicting
          (position will hold because v=0).
        """
        if not self.initialized:
            return self.position

        self._miss_time += max(dt, 0.0)
        dt_eff = float(np.clip(dt, 1e-3, self._max_predict_dt))
        if self._miss_time > self._freeze_after and not self._frozen:
            self.freeze_motion()
        self.predict(dt_eff)
        return self.position

    @property
    def miss_time(self) -> float:
        return float(self._miss_time)

    @property
    def position(self):
        return self.x[:3, 0].astype(np.float32)
