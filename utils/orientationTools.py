# Copyright 2018-2019 CNRS-UM LIRMM
#
# \author Yuquan Wang 
#
# 
#
# pyQpController is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# pyQpController is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
# General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with pyQpController. If not, see
# <http://www.gnu.org/licenses/>.


import numpy as np
import math

def eulerAnglesToRotationMatrix(angle_x, angle_y, angle_z ) :

    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(angle_x), -math.sin(angle_x) ],
                    [0,         math.sin(angle_x), math.cos(angle_x)  ]
                    ])



    R_y = np.array([[math.cos(angle_y),    0,      math.sin(angle_y)  ],
                    [0,                     1,      0                   ],
                    [-math.sin(angle_y),   0,      math.cos(angle_y)  ]
                    ])

    R_z = np.array([[math.cos(angle_z),    -math.sin(angle_z),    0],
                    [math.sin(angle_z),    math.cos(angle_z),     0],
                    [0,                     0,                      1]
                    ])


    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R