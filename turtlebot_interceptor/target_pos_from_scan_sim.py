import numpy as np
import math
import matplotlib.pyplot as plt

true_target_pos = [0, 1.4, 0.0]  # x, y, yaw_deg in world
seeker_pos = [0.0, 0.0, 0.0]     # seeker = world origin

# cones_seeker_frame = [
#     [0.2, 1.2, 0.2], # center x, center y, radius
#     [0.7, 0.4, 0.2],
#     # [1.0, 1.0, 0.2],
#     [-0.5, 1.3, 0.2],
#     # [1.6, 1.9, 0.2],
#     # [0.0, 2.5, 0.2]
# ]

# test symm in sim
cones_seeker_frame = [
    [0.0, 0.7, 0.08],
    [1.0, 0.7, 0.08],
    [-1.0, 0.7, 0.08]
]

cones_target_frame = []        # noisy cones (target)
true_cones_target_frame = []   # ground-truth cones (target)

tx, ty, yaw = true_target_pos
yaw_rad = math.radians(yaw)

R_t2w = np.array([
    [math.cos(yaw_rad), -math.sin(yaw_rad)],
    [math.sin(yaw_rad),  math.cos(yaw_rad)]
])  # target -> world rotation

t_t2w = np.array([tx, ty])      # target -> world translation

R_w2t = R_t2w.T                 # world -> target rotation
t_w2t = -R_w2t @ t_t2w          # world -> target translation

for cone in cones_seeker_frame:
    p_world = np.array(cone[:2])

    p_target_true = R_w2t @ p_world + t_w2t
    true_cones_target_frame.append([p_target_true[0], p_target_true[1], cone[2]])

    p_target_noisy = p_target_true + np.random.normal(0, 0.1, 2)
    cones_target_frame.append([p_target_noisy[0], p_target_noisy[1], cone[2]])

cones_target_frame = np.array(cones_target_frame)
true_cones_target_frame = np.array(true_cones_target_frame)

def find_target(seeker_world, target_frame):
    # A = noisy points in target frame
    # B = corresponding points in world frame
    A = np.array([p[:2] for p in target_frame])
    B = np.array([p[:2] for p in seeker_world])

    A_mean = A.mean(axis=0)
    B_mean = B.mean(axis=0)

    A_center = A - A_mean
    B_center = B - B_mean

    H = A_center.T @ B_center
    U, S, Vt = np.linalg.svd(H)

    R_t2w_est = Vt.T @ U.T
    if np.linalg.det(R_t2w_est) < 0:
        Vt[-1, :] *= -1
        R_t2w_est = Vt.T @ U.T

    t_t2w_est = B_mean - R_t2w_est @ A_mean

    est_yaw = math.degrees(math.atan2(R_t2w_est[1,0], R_t2w_est[0,0]))

    return np.array([t_t2w_est[0], t_t2w_est[1], est_yaw]), A, B, R_t2w_est, t_t2w_est


est_target_pos, A_tgt, B_wld, R_t2w_est, t_t2w_est = find_target(
    cones_seeker_frame,
    cones_target_frame
)

print("Estimated Target Pose (target frame relative to world):", est_target_pos)

B_est_world = (R_t2w_est @ cones_target_frame[:, :2].T).T + t_t2w_est

fig, ax = plt.subplots(1, 2, figsize=(12, 6))

# world frame
ax[0].scatter(0, 0, c='black', label="Seeker")
ax[0].scatter(tx, ty, c='green', label="True Target")
ax[0].scatter(t_t2w_est[0], t_t2w_est[1], c='orange', label="Estimated Target")

# world-frame true cones
A_np = np.array([p[:2] for p in cones_seeker_frame])
ax[0].scatter(A_np[:,0], A_np[:,1], marker='o', label="True Cones (World)")

# noisy cones converted back to world using estimated transform
ax[0].scatter(B_est_world[:,0], B_est_world[:,1], marker='x',
              label="Estimated Cones (World)")

ax[0].set_title("World Frame")
ax[0].axis("equal")
ax[0].grid(True)
ax[0].legend()

# target frame
ax[1].scatter(0, 0, c='green', label="True Target Origin")
ax[1].scatter(true_cones_target_frame[:,0], true_cones_target_frame[:,1],
              marker='o', label="True Cones (Target)")
ax[1].scatter(cones_target_frame[:,0], cones_target_frame[:,1],
              marker='x', label="Noisy Cones (Target)")

ax[1].set_title("Target Frame")
ax[1].axis("equal")
ax[1].grid(True)
ax[1].legend()

plt.tight_layout()
plt.show()
