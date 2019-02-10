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