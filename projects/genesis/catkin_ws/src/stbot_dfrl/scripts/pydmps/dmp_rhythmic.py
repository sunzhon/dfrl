"""
Copyright (C) 2013 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

from pydmps.dmp import DMPs

import numpy as np


class DMPs_rhythmic(DMPs):
    """An implementation of discrete DMPs"""

    def __init__(self, **kwargs):
        """
        """

        # call super class constructor
        super(DMPs_rhythmic, self).__init__(pattern="rhythmic", **kwargs)

        self.gen_centers()

        # set variance of Gaussian basis functions
        # trial and error to find this spacing
        self.h = np.ones(self.n_bfs) * self.n_bfs  # 1.75

        self.check_offset()

    def gen_centers(self):
        """Set the centre of the Gaussian basis
        functions be spaced evenly throughout run time"""

        c = np.linspace(0, 2 * np.pi, self.n_bfs + 1)
        c = c[0:-1]
        self.c = c

    def gen_front_term(self, x, dmp_num):
        """Generates the front term on the forcing term.
        For rhythmic DMPs it's non-diminishing, so this
        function is just a placeholder to return 1.

        x float: the current value of the canonical system
        dmp_num int: the index of the current dmp
        """

        if isinstance(x, np.ndarray):
            return np.ones(x.shape)
        return 1

    def gen_goal(self, y_des):
        """Generate the goal for path imitation.
        For rhythmic DMPs the goal is the average of the
        desired trajectory.

        y_des np.array: the desired trajectory to follow
        """

        goal = np.zeros(self.n_dmps)
        for n in range(self.n_dmps):
            num_idx = ~np.isnan(y_des[n])  # ignore nan's when calculating goal
            goal[n] = 0.5 * (y_des[n, num_idx].min() + y_des[n, num_idx].max())

        return goal

    def gen_psi(self, x):
        """Generates the activity of the basis functions for a given
        canonical system state or path.

        x float, array: the canonical system state or path
        """

        if isinstance(x, np.ndarray):
            x = x[:, None]
        return np.exp(self.h * (np.cos(x - self.c) - 1))

    def gen_weights(self, f_target):
        """Generate a set of weights over the basis functions such
        that the target forcing term trajectory is matched.

        f_target np.array: the desired forcing term trajectory
        """

        # calculate x and psi
        x_track = self.cs.rollout()
        psi_track = self.gen_psi(x_track)

        # efficiently calculate BF weights using weighted linear regression
        for d in range(self.n_dmps):
            for b in range(self.n_bfs):
                self.w[d, b] = np.dot(psi_track[:, b], f_target[:, d]) / (
                    np.sum(psi_track[:, b]) + 1e-10
                )

    def save_weights(self,file_name):
        np.savetxt(file_name,self.w)


# ==============================
# Test code
# ==============================
if __name__ == "__main__":
    pass
