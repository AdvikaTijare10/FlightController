import math

BETA = 0.1   # Madgwick filter gain

def madgwick_update(ax, ay, az, gx, gy, gz, dt):
    global q0, q1, q2, q3

    # Normalize accelerometer
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return
    ax /= norm
    ay /= norm
    az /= norm

    # Convert gyro to radians
    gx = math.radians(gx)
    gy = math.radians(gy)
    gz = math.radians(gz)

    # Gradient descent step
    f1 = 2*(q1*q3 - q0*q2) - ax
    f2 = 2*(q0*q1 + q2*q3) - ay
    f3 = 2*(0.5 - q1*q1 - q2*q2) - az

    grad0 = -2*q2*f1 + 2*q1*f2
    grad1 = 2*q3*f1 + 2*q0*f2 - 4*q1*f3
    grad2 = -2*q0*f1 + 2*q3*f2 - 4*q2*f3
    grad3 = 2*q1*f1 + 2*q2*f2

    norm = math.sqrt(grad0*grad0 + grad1*grad1 + grad2*grad2 + grad3*grad3)
    if norm == 0:
        return

    grad0 /= norm
    grad1 /= norm
    grad2 /= norm
    grad3 /= norm

    # Quaternion update
    qDot0 = 0.5 * (-q1*gx - q2*gy - q3*gz) - BETA * grad0
    qDot1 = 0.5 * (q0*gx + q2*gz - q3*gy) - BETA * grad1
    qDot2 = 0.5 * (q0*gy - q1*gz + q3*gx) - BETA * grad2
    qDot3 = 0.5 * (q0*gz + q1*gy - q2*gx) - BETA * grad3

    q0 += qDot0 * dt
    q1 += qDot1 * dt
    q2 += qDot2 * dt
    q3 += qDot3 * dt

    # Normalize quaternion
    norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm


# Convert quaternion → Euler
def get_euler():
    roll = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    pitch = math.asin(2*(q0*q2 - q3*q1))
    yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)