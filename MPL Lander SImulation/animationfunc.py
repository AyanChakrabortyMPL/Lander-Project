import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

def Animate2D(t_s, StateHistory):

    x = StateHistory[9, :]
    z = StateHistory[11, :]

    fig, ax = plt.subplots()

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")

    ax.set_xlim(np.min(x)-10, np.max(x)+10)
    ax.set_ylim(np.min(z)-10, np.max(z)+10)

    rocket, = ax.plot([], [], 'o')

    # Timestamp text
    time_text = ax.text(
        0.02, 0.95,
        '',
        transform=ax.transAxes,
        fontsize=12
    )

    def init():
        rocket.set_data([], [])
        time_text.set_text('')
        return rocket, time_text

    def update(frame):

        rocket.set_data([x[frame]], [z[frame]])

        time_text.set_text(
            f'Time = {t_s[frame]:.2f} s'
        )

        return rocket, time_text

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(t_s),
        init_func=init,
        interval=20,
        blit=True
    )

    plt.show()

def PlotVerticalVelocity(t_s, StateHistory):

    z = StateHistory[11, :]

    vertical_velocity = np.gradient(z, t_s)

    plt.figure(figsize=(8,5))

    plt.plot(t_s, vertical_velocity)

    plt.xlabel("Time (s)")
    plt.ylabel("Vertical Velocity (m/s)")
    plt.title("Vertical Velocity vs Time")

    plt.grid(True)

    plt.show()

def PlotFlightPath(t_s, StateHistory):

    x = StateHistory[9, :]
    y = StateHistory[10, :]
    z = -StateHistory[11, :]  # flip sign since NED has Z pointing down

    # --- 2D Side View (X vs Altitude) ---
    fig1, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(x, z)
    ax1.set_xlabel("Downrange Distance X (m)")
    ax1.set_ylabel("Altitude (m)")
    ax1.set_title("Flight Path - Side View")
    ax1.grid(True)

    # --- 3D Trajectory ---
    fig2 = plt.figure(figsize=(10, 7))
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.plot(x, y, z)
    ax2.scatter(x[0], y[0], z[0], color='green', s=50, label='Launch')   # start
    ax2.scatter(x[-1], y[-1], z[-1], color='red', s=50, label='Landing') # end
    ax2.set_xlabel("X - North (m)")
    ax2.set_ylabel("Y - East (m)")
    ax2.set_zlabel("Altitude (m)")
    ax2.set_title("3D Flight Path")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

def PlotAllStates(t_s, StateHistory):

    labels = [
        ('u (m/s)', 'Body X Velocity'),
        ('v (m/s)', 'Body Y Velocity'),
        ('w (m/s)', 'Body Z Velocity'),
        ('p (rad/s)', 'Roll Rate'),
        ('q (rad/s)', 'Pitch Rate'),
        ('r (rad/s)', 'Yaw Rate'),
        ('φ (rad)', 'Roll Angle'),
        ('θ (rad)', 'Pitch Angle'),
        ('ψ (rad)', 'Yaw Angle'),
        ('X (m)', 'World X Position'),
        ('Y (m)', 'World Y Position'),
        ('Z (m)', 'World Z Position'),
    ]

    fig, axes = plt.subplots(4, 3, figsize=(15, 12))
    axes = axes.flatten()

    for i, (ylabel, title) in enumerate(labels):
        axes[i].plot(t_s, StateHistory[i, :])
        axes[i].set_title(title)
        axes[i].set_xlabel('Time (s)')
        axes[i].set_ylabel(ylabel)
        axes[i].grid(True)

    plt.suptitle('All States vs Time', fontsize=14)
    plt.tight_layout()
    plt.show()