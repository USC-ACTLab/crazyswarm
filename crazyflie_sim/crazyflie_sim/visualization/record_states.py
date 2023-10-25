from __future__ import annotations

import datetime
import os

import numpy as np
from rclpy.node import Node

from ..sim_data_types import Action, State


class Visualization:
    """Records states in given format file."""

    def __init__(self, node: Node, params: dict, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.outdir = params['output_dir'] if 'output_dir' in params else 'state_info'
        self.outdir = self.outdir + '/' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
        os.makedirs(self.outdir, exist_ok=True)
        self.names_idx_map = {}
        # how many seconds to leave between logs
        self.logging_time = params['logging_time'] if 'logging_time' in params else 0.3
        # timestamp of last log, initialized to -self.logging_time so log occurs at t=0
        self.last_log = -self.logging_time
        self.supported_formats = {
            'csv': {
                'log': self.__log_csv
            },
            'np': {
                'log': self.__log_np,
                'shutdown': self.__shutdown_np
            }
        }
        self.active_formats = params['file_formats']
        for idx, name in enumerate(names):
            self.names_idx_map[name] = idx
        self.n = len(names)
        if 'csv' in self.active_formats:
            for idx, name in enumerate(names):
                os.makedirs(f'{self.outdir}/csv', exist_ok=True)
                csf = f'{self.outdir}/csv/{name}.csv'
                # initialize <robot_name>.csv
                with open(csf, 'w') as file:
                    file.write('timestamp,x,y,z,qw,qx,qy,qz\n')
        self.ts = []  # list of timestamps
        if 'np' in self.active_formats:
            os.makedirs(f'{self.outdir}/np', exist_ok=True)
            self.Ps = []  # list of positions matrices (self.n x 3)
            self.Qs = []  # list of quaternion matrices (self.n x 4)

    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        if t - self.last_log >= self.logging_time:
            self.last_log = t
            self.ts.append(t)
            P = np.zeros((self.n, 3))
            Q = np.zeros((self.n, 4))
            for name, state in zip(self.names, states):
                idx = self.names_idx_map[name]
                P[idx] = np.array(state.pos)
                Q[idx] = np.array(state.quat)
                for fmt in self.active_formats:
                    self.supported_formats[fmt]['log'](t, idx, P, Q)

    def __log_csv(self, t, idx: int, P: np.ndarray, Q: np.ndarray):
        """Record states in csv file."""
        with open(f'{self.outdir}/csv/{self.names[idx]}.csv', 'a') as file:
            file.write(f'{t},{P[idx,0]},{P[idx,1]},{P[idx,2]},{Q[idx,0]},{Q[idx,1]},{Q[idx,2]},{Q[idx,3]}\n')  # noqa E501

    def __log_np(self, t, idx: int, P: np.ndarray, Q: np.ndarray):
        self.Ps.append(P)
        self.Qs.append(Q)

    def __shutdown_np(self):
        P = np.array(self.Ps)
        Q = np.array(self.Qs)
        for idx, name in enumerate(self.names):
            np.savez_compressed(
                f'{self.outdir}/np/{name}.npz',
                t=self.ts,
                pos=P[idx::self.n, idx, :],
                quat=Q[idx::self.n, idx, :])

    def shutdown(self):
        for fmt in self.active_formats:
            if 'shutdown' in self.supported_formats[fmt]:
                self.supported_formats[fmt]['shutdown']()
