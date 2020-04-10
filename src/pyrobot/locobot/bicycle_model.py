# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
from scipy.linalg import block_diag


class Foo(object):
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __str__(self):
        str_ = ""
        for v in vars(self).keys():
            a = getattr(self, v)
            if True:  # isinstance(v, object):
                str__ = str(a)
                str__ = str__.replace("\n", "\n  ")
            else:
                str__ = str(a)
            str_ += "{:s}: {:s}".format(v, str__)
            str_ += "\n"
        return str_


def wrap_theta(theta):
    return np.mod(theta + np.pi, np.pi * 2) - np.pi


def subplot(plt, Y_X, sz_y_sz_x=(10, 10), space_y_x=(0.1, 0.1)):
    Y, X = Y_X
    sz_y, sz_x = sz_y_sz_x
    hspace, wspace = space_y_x
    plt.rcParams["figure.figsize"] = (X * sz_x, Y * sz_y)
    fig, axes = plt.subplots(Y, X, squeeze=False)
    plt.subplots_adjust(wspace=wspace, hspace=hspace)
    axes_list = axes.ravel()[::-1].tolist()
    return fig, axes, axes_list


def get_rng(rng):
    """Seed of a new rng from the first sample from this rng."""
    return np.random.RandomState(rng.choice(np.iinfo(np.uint32).max))


class BicycleSystem:
    def __init__(self, dt, min_v=-np.inf, max_v=np.inf, min_w=-np.inf, max_w=np.inf):
        self.dt = dt
        self.min_v = min_v
        self.min_w = min_w
        self.max_v = max_v
        self.max_w = max_w

    def get_system_cost(self, x_traj, x_ref):
        # Returns Q, q, q_ such that cost for the current step is:
        # xt*Q*x + xt*q + q_
        # x_goal is a vector [S x 1], u_ref is a vector [C x 1].
        state_dims = 3
        assert x_traj.shape[0] == state_dims
        assert x_ref.shape[0] == state_dims

        sc = 1
        cx = 3 * sc
        cy = 3 * sc
        ct = 0 * sc
        cdt = 4 * sc
        x, y, t = x_ref.copy()
        xg, yg, tg = x_traj.copy()
        t = np.mod(t, 2 * np.pi)
        delta_t = t - tg

        if np.abs(delta_t) < np.pi:
            theta_dash = ct * np.sin(t - tg) + cdt * delta_t
        else:
            theta_dash = ct * np.sin(t - tg) + cdt * (
                delta_t - 2 * np.pi * np.sign(delta_t)
            )

        Qdash = 2 * np.array([[cx * (x - xg), cy * (y - yg), theta_dash]]).T
        Qdashdash = 2 * np.eye(3)
        Qdashdash[0, 0] = 2 * cx
        Qdashdash[1, 1] = 2 * cy
        Qdashdash[2, 2] = 2 * ct * np.cos(t - tg) + 2 * cdt
        q_ref = cx * (x - xg) ** 2 + cy * (y - yg) ** 2
        q_ref = q_ref + ct * 2 * (1 - np.cos(t - tg))
        q_ref = q_ref + cdt * (
            np.minimum(np.abs(delta_t), 2 * np.pi - np.abs(delta_t)) ** 2
        )

        Qdashdash_v = np.zeros((0, 0))
        Qdash_v = np.zeros((0, 1))
        q_ref_v = 0

        Qdashdash = block_diag(Qdashdash, Qdashdash_v)
        Qdash = np.concatenate([Qdash, Qdash_v], axis=0)
        q_ref = q_ref + q_ref_v

        Q = 0.5 * Qdashdash
        x_ref_ = x_ref[:, np.newaxis].copy()
        q = Qdash - np.dot(Qdashdash, x_ref_)
        q_ = q_ref - np.dot(Qdash.T, x_ref_)
        q_ = q_ + 0.5 * (np.dot(np.dot(x_ref_.T, Qdashdash), x_ref_))

        return Q, q, q_, q_ref

    def dynamics_fn(self, x_ref, u_ref):
        # Returns A, B, C such that:
        # x_t+1 = Ax_t + Bu_t + C
        dt = self.dt
        x, y, theta = x_ref.copy()
        v, w = u_ref.copy()
        v_clip = np.clip(v, a_min=self.min_v, a_max=self.max_v)
        w_clip = np.clip(w, a_min=self.min_w, a_max=self.max_w)
        A = np.array(
            [
                [1, 0, -v_clip * dt * np.sin(theta)],
                [0, 1, v_clip * dt * np.cos(theta)],
                [0, 0, 1],
            ]
        )
        B = np.array(
            [
                [dt * np.cos(theta) * (v_clip == v), 0],
                [dt * np.sin(theta) * (v_clip == v), 0],
                [0, dt * (w_clip == w)],
            ]
        )

        def step(xt, ut, dt):
            x, y, theta = xt.copy()
            v, w = ut.copy()
            v_clip = np.clip(v, a_min=self.min_v, a_max=self.max_v)
            w_clip = np.clip(w, a_min=self.min_w, a_max=self.max_w)
            x = x + v_clip * np.cos(theta) * dt
            y = y + v_clip * np.sin(theta) * dt
            # theta = np.mod(theta + w*dt, 2*np.pi)
            theta = theta + w_clip * dt
            xt1 = np.array([x, y, theta])
            return xt1

        x_ref_ = x_ref[:, np.newaxis].copy()
        u_ref_ = u_ref[:, np.newaxis].copy()
        new_state = step(x_ref, u_ref, dt)
        C = new_state[:, np.newaxis] - np.dot(A, x_ref_) - np.dot(B, u_ref_)
        return A, B, C, new_state

    def get_control_cost(self, u_ref):
        # u_ref is [C x 1].
        u_ref_ = u_ref[:, np.newaxis].copy()
        R = 0.2 * np.eye(u_ref.shape[0])
        r = np.zeros((u_ref.shape[0], 1), dtype=np.float64)
        r_ = np.zeros((1, 1), dtype=np.float64)
        r_ref = np.dot(u_ref_.T, np.dot(R, u_ref_))[0, 0]
        return R, r, r_, r_ref

    def unroll(self, start_state, controls):
        T = controls.shape[0]
        states = []
        state = start_state.copy()
        for j in range(T):
            states.append(state)
            _, _, _, state = self.dynamics_fn(state, controls[j])
        states = np.array(states)
        return states
