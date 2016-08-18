import argparse
import os
import numpy as np

template = r"""/**
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *
 * Copyright (C) 2016 Wolfgang Hoenig and James Preiss,
 * University of Southern California
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "pptraj.h"

struct piecewise_traj {0} = 
{{
  .pieces = {{{1}
  }},
  .n_pieces = {2}
}};
"""

piece = r"""
    {{
      .p = {{
        {{{0}}},
        {{{1}}},
        {{{2}}},
        {{{3}}}
       }},
      .duration = {4}
    }},"""

def array2str(array):
    return ", ".join(str(x) for x in array)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="input csv file")
    args = parser.parse_args()

    name = os.path.splitext(os.path.basename(args.csv_file))[0]
    output_file = name + ".c"

    data = np.loadtxt(open(args.csv_file, "rb"), delimiter=",", skiprows=1)
    num_pieces = len(data)

    pieces = ""
    for row in data:
        pieces += piece.format(array2str(row[1:9]), array2str(row[9:17]), array2str(row[17:25]), array2str(row[25:33]), row[0])

    with open(output_file, 'w') as out:
        out.write(template.format(name, pieces, num_pieces))
