import pandas as pd
import matplotlib.pyplot as plt
import os

# 1. Figure out where this script lives:
script_dir = os.path.dirname(os.path.abspath(__file__))
print("Script directory:", script_dir)

# 2. Build the absolute path to your CSV
csv_path = os.path.join(script_dir, '..', 'homework', 'hw2', 'q1_joint_pos.csv')
csv_path = os.path.normpath(csv_path)   # clean up any “..” or extra slashes
print("Looking for CSV at:", csv_path)

# 3. (Optional) verify it exists before you try to read it:
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"CSV not found at {csv_path}")

# 4. Load it!
df = pd.read_csv(csv_path)

plt.figure()
plt.plot(df['t'], df['q0'], label='Joint 1')
plt.plot(df['t'], df['q1'], label='Joint 2')
plt.plot(df['t'], df['q2'], label='Joint 3')
plt.plot(df['t'], df['q3'], label='Joint 4')
plt.plot(df['t'], df['q4'], label='Joint 5')
plt.plot(df['t'], df['q5'], label='Joint 6')
plt.plot(df['t'], df['q6'], label='Joint 7')
plt.xlabel('Time (s)')
plt.ylabel('Joint Angles')
plt.title('Joint trajectories with joint and task gravity comp')

plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.plot(df['t'], df['x'], label='x')
plt.plot(df['t'], df['y'], label='y')
plt.plot(df['t'], df['z'], label='z')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('End effector position')

plt.grid(True)
plt.legend()
plt.show()
