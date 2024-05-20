import numpy as np
from PyKDL import Frame, Rotation, Vector

def cartesian_interpolate_step(T_curr, T_goal, max_delta=0.01, deadband=0.01):
    error = np.zeros(6)
    pe = T_goal.p - T_curr.p
    re = (T_curr.M.Inverse() * T_goal.M).GetRPY()
    for i in range(6):
        if i < 3:
            error[i] = pe[i]
        else:
            error[i] = re[i-3]

    done = False
    error_max = max(np.abs(error))
    if error_max <= deadband:
        error_scaled = error * 0.
        done = True
    else:
        error_scaled = error / error_max

    error_scaled = error_scaled * max_delta

    T_step = Frame(Rotation.RPY(error_scaled[3], error_scaled[4], error_scaled[5]),
                   Vector(error_scaled[0], error_scaled[1], error_scaled[2]))
    return T_step, done

if __name__=="__main__":
    test = 1