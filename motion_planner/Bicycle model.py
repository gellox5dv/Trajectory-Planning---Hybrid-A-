import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict


# ── DATACLASSES ───────────────────────────────────────────────────────────────

@dataclass
class VehicleParameters:
    max_steer:        float
    max_steer_rate:   float
    L:                float
    Lf:               float
    Lr:               float
    width:            float
    length:           float
    m:                float
    Iz:               float
    Cf:               float
    Cr:               float
    max_acceleration: float
    max_deceleration: float
    mu:               float


@dataclass
class EgoStateStamped:
    x:         float
    y:         float
    yaw:       float
    v:         float
    steer:     float
    timestamp: float = 0.0
    beta:      float = 0.0
    yaw_rate:  float = 0.0


@dataclass
class EgoInput:
    acceleration: float
    steer_rate:   float


@dataclass
class DynamicState:
    x:         float
    y:         float
    yaw:       float
    vx:        float
    vy:        float
    yaw_rate:  float
    steer:     float
    beta:      float = 0.0
    timestamp: float = 0.0


# ── BICYCLE MODEL ─────────────────────────────────────────────────────────────

def bicycle_model(
    state:   EgoStateStamped,
    control: EgoInput,
    params:  VehicleParameters,
    dt:      float
) -> EgoStateStamped:

    # Steering with rate limiting
    steer_new = state.steer + control.steer_rate * dt
    steer_new = np.clip(steer_new, -params.max_steer, params.max_steer)

    # Speed with acceleration limits
    acc_clamped = np.clip(control.acceleration, -params.max_deceleration, params.max_acceleration)
    v_new = max(state.v + acc_clamped * dt, 0.0)

    if v_new > 1e-3:
        Cf, Cr   = params.Cf, params.Cr
        lf, lr   = params.Lf, params.Lr
        m, Iz, v = params.m, params.Iz, v_new

        # A matrix
        A11 = -(Cf + Cr) / (m * v)
        A12 = (-lf * Cf + lr * Cr) / (m * v**2) - 1.0
        A21 = (-lf * Cf + lr * Cr) / Iz
        A22 = -(lf**2 * Cf + lr**2 * Cr) / (Iz * v)

        # B vector
        B1 = Cf / (m * v)
        B2 = lf * Cf / Iz

        # Read persistent state
        beta    = state.beta
        psi_dot = state.yaw_rate

        # Derivatives
        beta_dot = A11 * beta + A12 * psi_dot + B1 * steer_new
        psi_ddot = A21 * beta + A22 * psi_dot + B2 * steer_new

        # Integrate
        beta_new    = beta    + beta_dot * dt
        psi_dot_new = psi_dot + psi_ddot * dt
    else:
        beta_new    = 0.0
        psi_dot_new = 0.0

    yaw_new = state.yaw + psi_dot_new * dt
    yaw_new = (yaw_new + math.pi) % (2 * math.pi) - math.pi  # yaw betwwen -180 to 180

    x_new   = state.x + v_new * np.cos(beta_new + state.yaw) * dt
    y_new   = state.y + v_new * np.sin(beta_new + state.yaw) * dt

    return EgoStateStamped(
        x=x_new,
        y=y_new,
        yaw=yaw_new,
        v=v_new,
        steer=steer_new,
        timestamp=state.timestamp + dt,
        beta=beta_new,
        yaw_rate=psi_dot_new
    )


# ── VISUALIZER ────────────────────────────────────────────────────────────────

class Visualizer:

    def __init__(self, vehicle: 'Vehicle'):
        self.vehicle = vehicle

        self.trail_x:    List[float] = []
        self.trail_y:    List[float] = []
        self.steer_hist: List[float] = []
        self.yaw_hist:   List[float] = []
        self.beta_hist:  List[float] = []
        self.v_hist:     List[float] = []
        self.yr_hist:    List[float] = []
        self.t_hist:     List[float] = []

        self.fig, self.axes = plt.subplots(1, 2, figsize=(14, 6))
        self.ax_world = self.axes[0]
        self.ax_stats = self.axes[1]
        self.fig.suptitle('Bicycle model — vehicle visualizer', fontsize=12)
        plt.ion()

    def record(self, state: EgoStateStamped):
        self.trail_x.append(state.x)
        self.trail_y.append(state.y)
        self.steer_hist.append(math.degrees(state.steer))
        self.yaw_hist.append(math.degrees(state.yaw))
        self.beta_hist.append(math.degrees(state.beta))
        self.v_hist.append(state.v)
        self.yr_hist.append(math.degrees(state.yaw_rate))
        self.t_hist.append(state.timestamp)

    def _draw_wheel(self, ax, cx, cy, angle_rad):
        wl = self.vehicle.wheel_length
        ww = self.vehicle.wheel_width
        wheel = patches.Rectangle(
            (-wl / 2, -ww / 2), wl, ww,
            linewidth=0.8,
            edgecolor='#5F5E5A',
            facecolor='#2C2C2A'
        )
        t = (transforms.Affine2D()
             .rotate(angle_rad)
             .translate(cx, cy)
             + ax.transData)
        wheel.set_transform(t)
        ax.add_patch(wheel)

    def _draw_car(self, ax, state: EgoStateStamped):
        x, y, yaw = state.x, state.y, state.yaw
        p  = self.vehicle.params
        ht = self.vehicle.track / 2
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        def world(dx, dy):
            return (x + dx * cos_y - dy * sin_y,
                    y + dx * sin_y + dy * cos_y)

        # Car body
        body = patches.FancyBboxPatch(
            (-p.Lr, -p.width / 2), p.L, p.width,
            boxstyle="round,pad=0.05",
            linewidth=1.2,
            edgecolor='#185FA5',
            facecolor='#B5D4F4'
        )
        body.set_transform(
            transforms.Affine2D()
            .rotate(yaw).translate(x, y) + ax.transData
        )
        ax.add_patch(body)

        # Windscreen
        wscreen = patches.FancyBboxPatch(
            (p.Lf * 0.1, -p.width * 0.28),
            p.Lf * 0.75, p.width * 0.56,
            boxstyle="round,pad=0.02",
            linewidth=0,
            facecolor='#85B7EB',
            alpha=0.6
        )
        wscreen.set_transform(
            transforms.Affine2D()
            .rotate(yaw).translate(x, y) + ax.transData
        )
        ax.add_patch(wscreen)

        # Four wheels
        wheel_defs = [
            ( p.Lf,  ht,  True),
            ( p.Lf, -ht,  True),
            (-p.Lr,  ht,  False),
            (-p.Lr, -ht,  False),
        ]
        for dx, dy, steered in wheel_defs:
            wx, wy = world(dx, dy)
            angle  = yaw + (state.steer if steered else 0.0)
            self._draw_wheel(ax, wx, wy, angle)

        # Heading arrow (blue)
        arrow_l = p.length * 0.65
        ax.annotate('',
            xy=(x + arrow_l * cos_y, y + arrow_l * sin_y),
            xytext=(x, y),
            arrowprops=dict(arrowstyle='->', color='#185FA5', lw=2.0)
        )

        # Sideslip arrow (orange dashed)
        if abs(state.beta) > 0.003:
            va = yaw + state.beta
            ax.annotate('',
                xy=(x + arrow_l * math.cos(va),
                    y + arrow_l * math.sin(va)),
                xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color='#D85A30', lw=1.8, linestyle='dashed')
            )

        # Steering arc at front axle
        if abs(state.steer) > 0.01:
            fax, fay = world(p.Lf, 0)
            arc = patches.Arc(
                (fax, fay), 1.0, 1.0,
                angle=math.degrees(yaw),
                theta1=0,
                theta2=math.degrees(state.steer),
                color='#378ADD',
                linewidth=1.2
            )
            ax.add_patch(arc)

        # CoG dot
        ax.plot(x, y, 'o', color='#E24B4A', markersize=5, zorder=6)

    def _draw_world(self, state: EgoStateStamped):
        ax = self.ax_world
        ax.cla()
        ax.set_facecolor('#F1EFE8')
        ax.grid(True, color='white', linewidth=0.8, zorder=0)
        ax.set_aspect('equal')

        if len(self.trail_x) > 1:
            ax.plot(self.trail_x, self.trail_y,
                    '--', color='#1D9E75', linewidth=1.4,
                    alpha=0.7, label='path', zorder=1)

        self._draw_car(ax, state)

        r = max(self.vehicle.params.length * 5, 15)
        ax.set_xlim(state.x - r, state.x + r)
        ax.set_ylim(state.y - r, state.y + r)

        ax.set_title(
            f't={state.timestamp:.2f}s   '
            f'v={state.v:.1f} m/s   '
            f'steer={math.degrees(state.steer):.1f}°   '
            f'β={math.degrees(state.beta):.2f}°',
            fontsize=9
        )
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.plot([], [], '-',  color='#185FA5', label='heading ψ')
        ax.plot([], [], '--', color='#D85A30', label='velocity (β)')
        ax.plot([], [], 'o',  color='#E24B4A', label='CoG', markersize=5)
        ax.legend(loc='upper left', fontsize=8)

    def _draw_stats(self):
        ax = self.ax_stats
        ax.cla()
        ax.set_facecolor('#F1EFE8')
        t = self.t_hist

        ax.plot(t, self.steer_hist, color='#185FA5',
                linewidth=1.8, label='steer δ [deg]')
        ax.plot(t, self.yaw_hist,   color='#0F6E56',
                linewidth=1.6, label='yaw ψ [deg]')
        ax.plot(t, self.yr_hist,    color='#534AB7',
                linewidth=1.4, linestyle='-.', label='yaw rate [deg/s]')
        ax.plot(t, self.beta_hist,  color='#D85A30',
                linewidth=1.4, linestyle='--', label='sideslip β [deg]')
        ax.plot(t, self.v_hist,     color='#888780',
                linewidth=1.2, linestyle=':',  label='speed [m/s]')

        ax.axhline(0, color='#B4B2A9', linewidth=0.5)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('value')
        ax.set_title('State history', fontsize=10)
        ax.legend(fontsize=8, loc='upper left')
        ax.grid(True, color='white', linewidth=0.8)

    def update(self, state: EgoStateStamped):
        self.record(state)
        self._draw_world(state)
        self._draw_stats()
        plt.tight_layout()
        plt.pause(0.001)

    def show(self):
        plt.ioff()
        plt.show()


# ── VEHICLE ───────────────────────────────────────────────────────────────────

class Vehicle:
    def __init__(self, x, y, yaw, v):

        self.length        = 2.338
        self.width         = 1.381
        self.rear_to_wheel = 0.339
        self.wheel_length  = 0.531
        self.wheel_width   = 0.125
        self.track         = 1.094
        self.wheel_base    = 1.686

        self.Caf  = 2 * 32857.5
        self.Car  = 2 * 32857.5
        self.mass = 633.0
        self.lf   = 0.9442
        self.lr   = 0.7417
        self.Iz   = 430.166

        self.params = VehicleParameters(
            max_steer        = 0.6,
            max_steer_rate   = 0.5,
            L                = self.wheel_base,
            Lf               = self.lf,
            Lr               = self.lr,
            width            = self.width,
            length           = self.length,
            m                = self.mass,
            Iz               = self.Iz,
            Cf               = self.Caf,
            Cr               = self.Car,
            max_acceleration = 3.0,
            max_deceleration = 5.0,
            mu               = 0.85
        )

        self.state   = EgoStateStamped(x=x, y=y, yaw=yaw, v=v, steer=0.0)
        self.control = EgoInput(acceleration=0.0, steer_rate=0.0)
        self.viz     = Visualizer(self)

    def Motion_control(self, dt: float = 0.05):
        target_steer = math.radians(15)
        current      = self.state.steer
        diff         = target_steer - current

        steer_rate   = np.clip(diff / dt, -self.params.max_steer_rate, self.params.max_steer_rate)

        self.control = EgoInput(acceleration=0.0, steer_rate=steer_rate)
        self.Motion_model(dt)

    def Motion_model(self, dt: float = 0.05):
        self.state = bicycle_model(self.state, self.control, self.params, dt)
        self.viz.update(self.state)

    def Print_ego(self):
        s = self.state
        print(
              f"  x={s.x:.2f}  y={s.y:.2f}  "
              f"yaw={math.degrees(s.yaw):.1f}°  "
              f"steer={math.degrees(s.steer):.1f}°  "
              f"v={s.v:.2f} m/s  "
              f"β={math.degrees(s.beta):.3f}°"
             )


# ── DYNAMIC BICYCLE MODEL ─────────────────────────────────────────────────────

class DynamicBicycleModel:

    def __init__(self, params: VehicleParameters) -> None:
        self.params = params

    def step(self, state: DynamicState, control: EgoInput, dt: float) -> DynamicState:

        steer_new = state.steer + control.steer_rate * dt

        if abs(state.vx) > 1e-6:
            alpha_f = steer_new - np.arctan2(
                state.vy + self.params.Lf * state.yaw_rate, state.vx)
            alpha_r = -np.arctan2(
                state.vy - self.params.Lr * state.yaw_rate, state.vx)
        else:
            alpha_f = 0.0
            alpha_r = 0.0

        F_yf     = self.params.Cf * alpha_f
        F_yr     = self.params.Cr * alpha_r
        ax       = control.acceleration
        ay       = (F_yf + F_yr) / self.params.m
        yaw_ddot = (self.params.Lf * F_yf - self.params.Lr * F_yr) / self.params.Iz

        vx_new       = state.vx + (ax - state.vy * state.yaw_rate) * dt
        vy_new       = state.vy + (ay + state.vx * state.yaw_rate) * dt
        yaw_rate_new = state.yaw_rate + yaw_ddot * dt

        x_new   = state.x + (state.vx * np.cos(state.yaw)
                              - state.vy * np.sin(state.yaw)) * dt
        y_new   = state.y + (state.vx * np.sin(state.yaw)
                              + state.vy * np.cos(state.yaw)) * dt
        yaw_new = state.yaw + state.yaw_rate * dt

        return DynamicState(
            x=x_new, y=y_new, yaw=yaw_new,
            vx=vx_new, vy=vy_new,
            yaw_rate=yaw_rate_new,
            steer=steer_new,
            timestamp=state.timestamp + dt
        )


# ── MAIN ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':

    ego = Vehicle(x=0, y=0, yaw=0, v=10)

    print("Running simulation...")
    for step in range(200):
        ego.Motion_control(dt=0.05)
        if step % 40 == 0:
            ego.Print_ego()

    ego.viz.show()