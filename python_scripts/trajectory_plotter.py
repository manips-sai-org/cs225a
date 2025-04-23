import pandas as pd
import matplotlib.pyplot as plt
import os

# 1. Figure out where this script lives:
script_dir = os.path.dirname(os.path.abspath(__file__))
print("Script directory:", script_dir)

# 2. Build the absolute path to your CSV
csv_path = os.path.join(script_dir, '..', 'homework', 'hw1', 'q1_joint_pos.csv')
csv_path = os.path.normpath(csv_path)   # clean up any “..” or extra slashes
print("Looking for CSV at:", csv_path)

# 3. (Optional) verify it exists before you try to read it:
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"CSV not found at {csv_path}")

# 4. Load it!
df = pd.read_csv(csv_path)

plt.figure()
plt.plot(df['t'], df['q0'], label='Joint 1')
plt.plot(df['t'], df['q2'], label='Joint 3')
plt.plot(df['t'], df['q3'], label='Joint 4')
plt.xlabel('10 * Time (s)')
plt.ylabel('Joint Angle')
plt.title('bonus: 2.5 EE payload compensation')

plt.grid(True)
plt.legend()
plt.show()

