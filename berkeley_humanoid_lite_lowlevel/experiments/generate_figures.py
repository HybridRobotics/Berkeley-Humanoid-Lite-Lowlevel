import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
from matplotlib.font_manager import FontProperties


os.makedirs("berkeley_humanoid_lite_lowlevel/experiments/figures", exist_ok=True)


line_width = 3

def format_plot(
    ax: plt.Axes,
    font_path: str = "berkeley_humanoid_lite_lowlevel/experiments/assets/Arimo-Regular.ttf",
    font_size: int = 20,
    legend: bool = False,
):
    font = FontProperties(fname=font_path, size=font_size)
    font_larger = FontProperties(fname=font_path, size=font_size + 2)

    # set tick label font
    ax.tick_params(axis="both", labelsize=font_size)
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontproperties(font)

    # set title font
    ax.title.set_fontproperties(font)
    ax.title.set_fontsize(font_size)

    # set legend font
    if legend:
        ax.legend(prop=font)

    # set label font
    ax.xaxis.label.set_fontproperties(font_larger)
    ax.yaxis.label.set_fontproperties(font_larger)



def plot_durability():
    data_path = "berkeley_humanoid_lite_lowlevel/experiments/actuator_durability/6512_actuator_durability_test.csv"
    data = np.genfromtxt(data_path, delimiter=",", skip_header=1)

    time = data[:, 0]
    efficiency_1 = data[:, 1]
    efficiency_2 = data[:, 2]
    backlash_rev = data[:, 3]

    average_efficiency = (efficiency_1 + efficiency_2) / 2
    backlash_rad = backlash_rev * 2 * np.pi

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))
    ax1.plot(time, average_efficiency, label="Efficiency", color="#3949ab")
    ax1.set_ylabel("Efficiency (%)")

    ax1.set_ylim(0.0, 0.4)

    ax2.plot(time, backlash_rad, label="Backlash", color="#d84315")
    ax2.set_xlabel("Time (hours)")
    ax2.set_ylabel("Backlash (rad)")

    ax2.set_ylim(0.00, 0.03)

    format_plot(ax1)
    format_plot(ax2)

    ax1.grid(True, linestyle="--")
    ax2.grid(True, linestyle="--")

    plt.savefig("berkeley_humanoid_lite_lowlevel/experiments/figures/experiment-actuator-durability-6512.pdf", bbox_inches="tight")

    # plt.show()
    plt.close()

# plot_durability()


def plot_efficiency_6512():
    TORQUE_TARGET_IDX = 0
    TORQUE_MOTOR_IDX = 1
    TORQUE_LOADCELL_IDX = 2
    VELOCITY_IDX = 3
    VOLTAGE_IDX = 4
    CURRENT_IDX = 5

    # Read the CSV file
    data_1rad = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/efficiency_test/6512/torque_sweep_1.0_rad_s.csv", delimiter=",", skip_header=1)
    data_2rad = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/efficiency_test/6512/torque_sweep_2.0_rad_s.csv", delimiter=",", skip_header=1)
    data_5rad = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/efficiency_test/6512/torque_sweep_5.0_rad_s.csv", delimiter=",", skip_header=1)
    data_10rad = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/efficiency_test/6512/torque_sweep_10.0_rad_s.csv", delimiter=",", skip_header=1)

    # trim unusable data
    data_1rad = data_1rad[1:]
    data_2rad = data_2rad[1:]
    data_5rad = data_5rad[1:]
    data_10rad = data_10rad[1:-2]

    output_power_1rad = data_1rad[:, TORQUE_LOADCELL_IDX] * data_1rad[:, VELOCITY_IDX]
    output_power_2rad = data_2rad[:, TORQUE_LOADCELL_IDX] * data_2rad[:, VELOCITY_IDX]
    output_power_5rad = data_5rad[:, TORQUE_LOADCELL_IDX] * data_5rad[:, VELOCITY_IDX]
    output_power_10rad = data_10rad[:, TORQUE_LOADCELL_IDX] * data_10rad[:, VELOCITY_IDX]

    input_power_1rad = data_1rad[:, TORQUE_MOTOR_IDX] * data_1rad[:, VELOCITY_IDX]
    input_power_2rad = data_2rad[:, TORQUE_MOTOR_IDX] * data_2rad[:, VELOCITY_IDX]
    input_power_5rad = data_5rad[:, TORQUE_MOTOR_IDX] * data_5rad[:, VELOCITY_IDX]
    input_power_10rad = data_10rad[:, TORQUE_MOTOR_IDX] * data_10rad[:, VELOCITY_IDX]

    electrical_power_1rad = data_1rad[:, VOLTAGE_IDX] * data_1rad[:, CURRENT_IDX]
    electrical_power_2rad = data_2rad[:, VOLTAGE_IDX] * data_2rad[:, CURRENT_IDX]
    electrical_power_5rad = data_5rad[:, VOLTAGE_IDX] * data_5rad[:, CURRENT_IDX]
    electrical_power_10rad = data_10rad[:, VOLTAGE_IDX] * data_10rad[:, CURRENT_IDX]

    efficiency_1rad = output_power_1rad / input_power_1rad
    efficiency_2rad = output_power_2rad / input_power_2rad
    efficiency_5rad = output_power_5rad / input_power_5rad
    efficiency_10rad = output_power_10rad / input_power_10rad

    efficiency_electrical_1rad = output_power_1rad / electrical_power_1rad
    efficiency_electrical_2rad = output_power_2rad / electrical_power_2rad
    efficiency_electrical_5rad = output_power_5rad / electrical_power_5rad
    efficiency_electrical_10rad = output_power_10rad / electrical_power_10rad

    measured_torque_1rad = data_1rad[:, TORQUE_MOTOR_IDX]
    measured_torque_2rad = data_2rad[:, TORQUE_MOTOR_IDX]
    measured_torque_5rad = data_5rad[:, TORQUE_MOTOR_IDX]
    measured_torque_10rad = data_10rad[:, TORQUE_MOTOR_IDX]

    # get sorting indices for each torque array
    sort_idx_1rad = np.argsort(measured_torque_1rad)
    sort_idx_2rad = np.argsort(measured_torque_2rad)
    sort_idx_5rad = np.argsort(measured_torque_5rad)
    sort_idx_10rad = np.argsort(measured_torque_10rad)

    # sort torques and efficiencies using the indices
    measured_torque_1rad = measured_torque_1rad[sort_idx_1rad]
    measured_torque_2rad = measured_torque_2rad[sort_idx_2rad]
    measured_torque_5rad = measured_torque_5rad[sort_idx_5rad]
    measured_torque_10rad = measured_torque_10rad[sort_idx_10rad]

    efficiency_1rad = efficiency_1rad[sort_idx_1rad]
    efficiency_2rad = efficiency_2rad[sort_idx_2rad]
    efficiency_5rad = efficiency_5rad[sort_idx_5rad]
    efficiency_10rad = efficiency_10rad[sort_idx_10rad]

    efficiency_electrical_1rad = efficiency_electrical_1rad[sort_idx_1rad]
    efficiency_electrical_2rad = efficiency_electrical_2rad[sort_idx_2rad]
    efficiency_electrical_5rad = efficiency_electrical_5rad[sort_idx_5rad]
    efficiency_electrical_10rad = efficiency_electrical_10rad[sort_idx_10rad]

    fig, ax1 = plt.subplots(figsize=(12, 8))
    
    # First axis for mechanical efficiency
    ax1.plot(measured_torque_1rad, efficiency_1rad, label="1 rad/s (mech)", color="#3949ab", linewidth=line_width)
    ax1.plot(measured_torque_2rad, efficiency_2rad, label="2 rad/s (mech)", color="#00897b", linewidth=line_width)
    ax1.plot(measured_torque_5rad, efficiency_5rad, label="5 rad/s (mech)", color="#d84315", linewidth=line_width)
    # ax1.plot(data_10rad[:, TORQUE_MOTOR_IDX], efficiency_10rad, label="10 rad/s (mech)", color="#ff6f00", linewidth=line_width)
    ax1.set_ylabel("Efficiency (%)")
    
    # Create twin axis for electrical efficiency
    # ax2 = ax1.twinx()
    ax1.plot(measured_torque_1rad, efficiency_electrical_1rad, label="1 rad/s (elec)", color="#3949ab", linestyle="--", linewidth=line_width)
    ax1.plot(measured_torque_2rad, efficiency_electrical_2rad, label="2 rad/s (elec)", color="#00897b", linestyle="--", linewidth=line_width)
    ax1.plot(measured_torque_5rad, efficiency_electrical_5rad, label="5 rad/s (elec)", color="#d84315", linestyle="--", linewidth=line_width)
    # ax1.plot(data_10rad[:, TORQUE_MOTOR_IDX], efficiency_electrical_10rad, label="10 rad/s (elec)", color="#ff6f00", linestyle="--", linewidth=line_width)
    # ax1.set_ylabel("Electrical Efficiency (%)")
    ax1.set_ylim(0.0, 1.1)
    
    # Combine legends from both axes
    lines1, labels1 = ax1.get_legend_handles_labels()
    # lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1, labels1, loc='upper right')
    
    format_plot(ax1, legend=True)
    # format_plot(ax2)
    
    ax1.grid(True, linestyle='--')
    ax1.set_xlabel("Torque (Nm)")

    plt.savefig("berkeley_humanoid_lite_lowlevel/experiments/figures/experiment-actuator-efficiency-6512.pdf", bbox_inches="tight")

    plt.show()
    plt.close()

# plot_efficiency_6512()


def plot_efficiency_5010():
    TORQUE_TARGET_IDX = 0
    TORQUE_MOTOR_IDX = 1
    TORQUE_LOADCELL_IDX = 2
    VELOCITY_IDX = 3
    VOLTAGE_IDX = 4
    CURRENT_IDX = 5

    # Read the CSV file
    data_1rad = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/efficiency_test/5010/torque_sweep_1.0_rad_s.csv", delimiter=",", skip_header=1)
    data_2rad = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/efficiency_test/5010/torque_sweep_2.0_rad_s.csv", delimiter=",", skip_header=1)
    data_5rad = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/efficiency_test/5010/torque_sweep_5.0_rad_s.csv", delimiter=",", skip_header=1)
    data_10rad = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/efficiency_test/5010/torque_sweep_10.0_rad_s.csv", delimiter=",", skip_header=1)

    # trim unusable data
    data_1rad = data_1rad[1:]
    data_2rad = data_2rad[1:]
    data_5rad = data_5rad[1:-4]
    data_10rad = data_10rad[1:-2]

    output_power_1rad = data_1rad[:, TORQUE_LOADCELL_IDX] * data_1rad[:, VELOCITY_IDX]
    output_power_2rad = data_2rad[:, TORQUE_LOADCELL_IDX] * data_2rad[:, VELOCITY_IDX]
    output_power_5rad = data_5rad[:, TORQUE_LOADCELL_IDX] * data_5rad[:, VELOCITY_IDX]
    output_power_10rad = data_10rad[:, TORQUE_LOADCELL_IDX] * data_10rad[:, VELOCITY_IDX]

    input_power_1rad = data_1rad[:, TORQUE_MOTOR_IDX] * data_1rad[:, VELOCITY_IDX]
    input_power_2rad = data_2rad[:, TORQUE_MOTOR_IDX] * data_2rad[:, VELOCITY_IDX]
    input_power_5rad = data_5rad[:, TORQUE_MOTOR_IDX] * data_5rad[:, VELOCITY_IDX]
    input_power_10rad = data_10rad[:, TORQUE_MOTOR_IDX] * data_10rad[:, VELOCITY_IDX]

    electrical_power_1rad = data_1rad[:, VOLTAGE_IDX] * data_1rad[:, CURRENT_IDX]
    electrical_power_2rad = data_2rad[:, VOLTAGE_IDX] * data_2rad[:, CURRENT_IDX]
    electrical_power_5rad = data_5rad[:, VOLTAGE_IDX] * data_5rad[:, CURRENT_IDX]
    electrical_power_10rad = data_10rad[:, VOLTAGE_IDX] * data_10rad[:, CURRENT_IDX]

    efficiency_1rad = output_power_1rad / input_power_1rad
    efficiency_2rad = output_power_2rad / input_power_2rad
    efficiency_5rad = output_power_5rad / input_power_5rad
    efficiency_10rad = output_power_10rad / input_power_10rad

    efficiency_electrical_1rad = output_power_1rad / electrical_power_1rad
    efficiency_electrical_2rad = output_power_2rad / electrical_power_2rad
    efficiency_electrical_5rad = output_power_5rad / electrical_power_5rad
    efficiency_electrical_10rad = output_power_10rad / electrical_power_10rad

    measured_torque_1rad = data_1rad[:, TORQUE_MOTOR_IDX]
    measured_torque_2rad = data_2rad[:, TORQUE_MOTOR_IDX]
    measured_torque_5rad = data_5rad[:, TORQUE_MOTOR_IDX]
    measured_torque_10rad = data_10rad[:, TORQUE_MOTOR_IDX]

    # get sorting indices for each torque array
    sort_idx_1rad = np.argsort(measured_torque_1rad)
    sort_idx_2rad = np.argsort(measured_torque_2rad)
    sort_idx_5rad = np.argsort(measured_torque_5rad)
    sort_idx_10rad = np.argsort(measured_torque_10rad)

    # sort torques and efficiencies using the indices
    measured_torque_1rad = measured_torque_1rad[sort_idx_1rad]
    measured_torque_2rad = measured_torque_2rad[sort_idx_2rad]
    measured_torque_5rad = measured_torque_5rad[sort_idx_5rad]
    measured_torque_10rad = measured_torque_10rad[sort_idx_10rad]

    efficiency_1rad = efficiency_1rad[sort_idx_1rad]
    efficiency_2rad = efficiency_2rad[sort_idx_2rad]
    efficiency_5rad = efficiency_5rad[sort_idx_5rad]
    efficiency_10rad = efficiency_10rad[sort_idx_10rad]

    efficiency_electrical_1rad = efficiency_electrical_1rad[sort_idx_1rad]
    efficiency_electrical_2rad = efficiency_electrical_2rad[sort_idx_2rad]
    efficiency_electrical_5rad = efficiency_electrical_5rad[sort_idx_5rad]
    efficiency_electrical_10rad = efficiency_electrical_10rad[sort_idx_10rad]

    fig, ax1 = plt.subplots(figsize=(12, 8))
    
    # First axis for mechanical efficiency
    ax1.plot(measured_torque_1rad, efficiency_1rad, label="1 rad/s (mech)", color="#3949ab", linewidth=line_width)
    ax1.plot(measured_torque_2rad, efficiency_2rad, label="2 rad/s (mech)", color="#00897b", linewidth=line_width)
    ax1.plot(measured_torque_5rad, efficiency_5rad, label="5 rad/s (mech)", color="#d84315", linewidth=line_width)
    ax1.set_ylabel("Efficiency (%)")
    
    # Create twin axis for electrical efficiency
    # ax2 = ax1.twinx()
    ax1.plot(measured_torque_1rad, efficiency_electrical_1rad, label="1 rad/s (elec)", color="#3949ab", linestyle="--", linewidth=line_width)
    ax1.plot(measured_torque_2rad, efficiency_electrical_2rad, label="2 rad/s (elec)", color="#00897b", linestyle="--", linewidth=line_width)
    ax1.plot(measured_torque_5rad, efficiency_electrical_5rad, label="5 rad/s (elec)", color="#d84315", linestyle="--", linewidth=line_width)
    # ax1.set_ylabel("Electrical Efficiency (%)")
    # ax2.set_ylim(0.0, 0.6)
    
    # Combine legends from both axes
    lines1, labels1 = ax1.get_legend_handles_labels()
    # lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1, labels1, loc='upper right')
    
    format_plot(ax1, legend=True)
    # format_plot(ax2)
    
    ax1.grid(True, linestyle='--')
    ax1.set_xlabel("Torque (Nm)")

    plt.savefig("berkeley_humanoid_lite_lowlevel/experiments/figures/experiment-actuator-efficiency-5010.pdf", bbox_inches="tight")

    plt.show()
    plt.close()

# plot_efficiency_5010()



def plot_stiffness():
    data = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/actuator_stiffness/actuator_stiffness.txt", delimiter=",", skip_header=1)
    torques = data[:, 0]
    positions = data[:, 1]

    position_max = np.max(positions)
    position_min = np.min(positions)

    position_offset = (position_max + position_min) / 2
    positions -= position_offset

    fig, ax1 = plt.subplots(figsize=(12, 4))
    ax1.plot(torques, positions, label="Stiffness", color="#3949ab")
    ax1.set_ylabel("Position (rad)")
    ax1.set_xlabel("Torque (Nm)")

    # draw a line of y=mx+b with dotted style
    b = 0.0369
    m = (0.0995 - b) / 20
    ax1.plot(torques[torques > 0], m * torques[torques > 0] + b, 
             label="Linear fit", color="#d84315", linestyle=":", linewidth=line_width)

    ax1.plot(torques[torques < 0], m * torques[torques < 0] - b, 
             color="#d84315", linestyle=":", linewidth=line_width)

    print(1 / m)

    format_plot(ax1)

    ax1.grid(True, linestyle="--")

    plt.savefig("berkeley_humanoid_lite_lowlevel/experiments/figures/experiment-actuator-stiffness-6512.pdf", bbox_inches="tight")

    # plt.show()
    plt.close()

# plot_stiffness()




def plot_rebuttal_efficiency():
    TORQUE_TARGET_IDX = 0
    TORQUE_MOTOR_IDX = 1
    TORQUE_LOADCELL_IDX = 2
    VELOCITY_IDX = 3
    VOLTAGE_IDX = 4
    CURRENT_IDX = 5

    # Read the CSV file
    data_rad_original = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/rebuttal_efficiency/torque_sweep_1.0_rad_s_original.csv", delimiter=",", skip_header=1)
    data_rad_x1c_1 = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/rebuttal_efficiency/torque_sweep_1.0_rad_s_x1c_1.csv", delimiter=",", skip_header=1)
    data_rad_x1c_2 = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/rebuttal_efficiency/torque_sweep_1.0_rad_s_x1c_2.csv", delimiter=",", skip_header=1)
    data_rad_x1c_3 = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/rebuttal_efficiency/torque_sweep_1.0_rad_s_x1c_3.csv", delimiter=",", skip_header=1)
    data_rad_a1_1 = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/rebuttal_efficiency/torque_sweep_1.0_rad_s_a1_1.csv", delimiter=",", skip_header=1)
    data_rad_a1_2 = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/rebuttal_efficiency/torque_sweep_1.0_rad_s_a1_2.csv", delimiter=",", skip_header=1)
    data_rad_a1_3 = np.genfromtxt("berkeley_humanoid_lite_lowlevel/experiments/rebuttal_efficiency/torque_sweep_1.0_rad_s_a1_3.csv", delimiter=",", skip_header=1)
    
    # trim unusable data
    data_rad_original = data_rad_original[1:]
    data_rad_x1c_1 = data_rad_x1c_1[1:]
    data_rad_x1c_2 = data_rad_x1c_2[1:]
    data_rad_x1c_3 = data_rad_x1c_3[1:]
    data_rad_a1_1 = data_rad_a1_1[1:]
    data_rad_a1_2 = data_rad_a1_2[1:]
    data_rad_a1_3 = data_rad_a1_3[1:]

    output_power_1rad_original = data_rad_original[:, TORQUE_LOADCELL_IDX] * data_rad_original[:, VELOCITY_IDX]
    output_power_1rad_x1c_1 = data_rad_x1c_1[:, TORQUE_LOADCELL_IDX] * data_rad_x1c_1[:, VELOCITY_IDX]
    output_power_1rad_x1c_2 = data_rad_x1c_2[:, TORQUE_LOADCELL_IDX] * data_rad_x1c_2[:, VELOCITY_IDX]
    output_power_1rad_x1c_3 = data_rad_x1c_3[:, TORQUE_LOADCELL_IDX] * data_rad_x1c_3[:, VELOCITY_IDX]
    output_power_1rad_a1_1 = data_rad_a1_1[:, TORQUE_LOADCELL_IDX] * data_rad_a1_1[:, VELOCITY_IDX]
    output_power_1rad_a1_2 = data_rad_a1_2[:, TORQUE_LOADCELL_IDX] * data_rad_a1_2[:, VELOCITY_IDX]
    output_power_1rad_a1_3 = data_rad_a1_3[:, TORQUE_LOADCELL_IDX] * data_rad_a1_3[:, VELOCITY_IDX]

    input_power_1rad_original = data_rad_original[:, TORQUE_MOTOR_IDX] * data_rad_original[:, VELOCITY_IDX]
    input_power_1rad_x1c_1 = data_rad_x1c_1[:, TORQUE_MOTOR_IDX] * data_rad_x1c_1[:, VELOCITY_IDX]
    input_power_1rad_x1c_2 = data_rad_x1c_2[:, TORQUE_MOTOR_IDX] * data_rad_x1c_2[:, VELOCITY_IDX]
    input_power_1rad_x1c_3 = data_rad_x1c_3[:, TORQUE_MOTOR_IDX] * data_rad_x1c_3[:, VELOCITY_IDX]
    input_power_1rad_a1_1 = data_rad_a1_1[:, TORQUE_MOTOR_IDX] * data_rad_a1_1[:, VELOCITY_IDX]
    input_power_1rad_a1_2 = data_rad_a1_2[:, TORQUE_MOTOR_IDX] * data_rad_a1_2[:, VELOCITY_IDX]
    input_power_1rad_a1_3 = data_rad_a1_3[:, TORQUE_MOTOR_IDX] * data_rad_a1_3[:, VELOCITY_IDX]

    electrical_power_1rad_original = data_rad_original[:, VOLTAGE_IDX] * data_rad_original[:, CURRENT_IDX]
    electrical_power_1rad_x1c_1 = data_rad_x1c_1[:, VOLTAGE_IDX] * data_rad_x1c_1[:, CURRENT_IDX]
    electrical_power_1rad_x1c_2 = data_rad_x1c_2[:, VOLTAGE_IDX] * data_rad_x1c_2[:, CURRENT_IDX]
    electrical_power_1rad_x1c_3 = data_rad_x1c_3[:, VOLTAGE_IDX] * data_rad_x1c_3[:, CURRENT_IDX]
    electrical_power_1rad_a1_1 = data_rad_a1_1[:, VOLTAGE_IDX] * data_rad_a1_1[:, CURRENT_IDX]
    electrical_power_1rad_a1_2 = data_rad_a1_2[:, VOLTAGE_IDX] * data_rad_a1_2[:, CURRENT_IDX]
    electrical_power_1rad_a1_3 = data_rad_a1_3[:, VOLTAGE_IDX] * data_rad_a1_3[:, CURRENT_IDX]

    efficiency_1rad_original = output_power_1rad_original / input_power_1rad_original
    efficiency_1rad_x1c_1 = output_power_1rad_x1c_1 / input_power_1rad_x1c_1
    efficiency_1rad_x1c_2 = output_power_1rad_x1c_2 / input_power_1rad_x1c_2
    efficiency_1rad_x1c_3 = output_power_1rad_x1c_3 / input_power_1rad_x1c_3
    efficiency_1rad_a1_1 = output_power_1rad_a1_1 / input_power_1rad_a1_1
    efficiency_1rad_a1_2 = output_power_1rad_a1_2 / input_power_1rad_a1_2
    efficiency_1rad_a1_3 = output_power_1rad_a1_3 / input_power_1rad_a1_3

    efficiency_electrical_1rad_original = output_power_1rad_original / electrical_power_1rad_original
    efficiency_electrical_1rad_x1c_1 = output_power_1rad_x1c_1 / electrical_power_1rad_x1c_1
    efficiency_electrical_1rad_x1c_2 = output_power_1rad_x1c_2 / electrical_power_1rad_x1c_2
    efficiency_electrical_1rad_x1c_3 = output_power_1rad_x1c_3 / electrical_power_1rad_x1c_3
    efficiency_electrical_1rad_a1_1 = output_power_1rad_a1_1 / electrical_power_1rad_a1_1
    efficiency_electrical_1rad_a1_2 = output_power_1rad_a1_2 / electrical_power_1rad_a1_2
    efficiency_electrical_1rad_a1_3 = output_power_1rad_a1_3 / electrical_power_1rad_a1_3

    target_torque_1rad_original = data_rad_original[:, TORQUE_TARGET_IDX]
    target_torque_1rad_x1c_1 = data_rad_x1c_1[:, TORQUE_TARGET_IDX]
    target_torque_1rad_x1c_2 = data_rad_x1c_2[:, TORQUE_TARGET_IDX]
    target_torque_1rad_x1c_3 = data_rad_x1c_3[:, TORQUE_TARGET_IDX]
    target_torque_1rad_a1_1 = data_rad_a1_1[:, TORQUE_TARGET_IDX]
    target_torque_1rad_a1_2 = data_rad_a1_2[:, TORQUE_TARGET_IDX]
    target_torque_1rad_a1_3 = data_rad_a1_3[:, TORQUE_TARGET_IDX]

    measured_torque_1rad_original = data_rad_original[:, TORQUE_MOTOR_IDX]
    measured_torque_1rad_x1c_1 = data_rad_x1c_1[:, TORQUE_MOTOR_IDX]
    measured_torque_1rad_x1c_2 = data_rad_x1c_2[:, TORQUE_MOTOR_IDX]
    measured_torque_1rad_x1c_3 = data_rad_x1c_3[:, TORQUE_MOTOR_IDX]
    measured_torque_1rad_a1_1 = data_rad_a1_1[:, TORQUE_MOTOR_IDX]
    measured_torque_1rad_a1_2 = data_rad_a1_2[:, TORQUE_MOTOR_IDX]
    measured_torque_1rad_a1_3 = data_rad_a1_3[:, TORQUE_MOTOR_IDX]

    # get sorting indices for each torque array
    sort_idx_1rad_original = np.argsort(measured_torque_1rad_original)
    sort_idx_1rad_x1c_1 = np.argsort(measured_torque_1rad_x1c_1)
    sort_idx_1rad_x1c_2 = np.argsort(measured_torque_1rad_x1c_2)
    sort_idx_1rad_x1c_3 = np.argsort(measured_torque_1rad_x1c_3)
    sort_idx_1rad_a1_1 = np.argsort(measured_torque_1rad_a1_1)
    sort_idx_1rad_a1_2 = np.argsort(measured_torque_1rad_a1_2)
    sort_idx_1rad_a1_3 = np.argsort(measured_torque_1rad_a1_3)

    # sort torques and efficiencies using the indices
    measured_torque_1rad_original = measured_torque_1rad_original[sort_idx_1rad_original]
    measured_torque_1rad_x1c_1 = measured_torque_1rad_x1c_1[sort_idx_1rad_x1c_1]
    measured_torque_1rad_x1c_2 = measured_torque_1rad_x1c_2[sort_idx_1rad_x1c_2]
    measured_torque_1rad_x1c_3 = measured_torque_1rad_x1c_3[sort_idx_1rad_x1c_3]
    measured_torque_1rad_a1_1 = measured_torque_1rad_a1_1[sort_idx_1rad_a1_1]
    measured_torque_1rad_a1_2 = measured_torque_1rad_a1_2[sort_idx_1rad_a1_2]
    measured_torque_1rad_a1_3 = measured_torque_1rad_a1_3[sort_idx_1rad_a1_3]

    efficiency_1rad_original = efficiency_1rad_original[sort_idx_1rad_original]
    efficiency_1rad_x1c_1 = efficiency_1rad_x1c_1[sort_idx_1rad_x1c_1]
    efficiency_1rad_x1c_2 = efficiency_1rad_x1c_2[sort_idx_1rad_x1c_2]
    efficiency_1rad_x1c_3 = efficiency_1rad_x1c_3[sort_idx_1rad_x1c_3]
    efficiency_1rad_a1_1 = efficiency_1rad_a1_1[sort_idx_1rad_a1_1]
    efficiency_1rad_a1_2 = efficiency_1rad_a1_2[sort_idx_1rad_a1_2]
    efficiency_1rad_a1_3 = efficiency_1rad_a1_3[sort_idx_1rad_a1_3]

    efficiency_electrical_1rad_original = efficiency_electrical_1rad_original[sort_idx_1rad_original]
    efficiency_electrical_1rad_x1c_1 = efficiency_electrical_1rad_x1c_1[sort_idx_1rad_x1c_1]
    efficiency_electrical_1rad_x1c_2 = efficiency_electrical_1rad_x1c_2[sort_idx_1rad_x1c_2]
    efficiency_electrical_1rad_x1c_3 = efficiency_electrical_1rad_x1c_3[sort_idx_1rad_x1c_3]
    efficiency_electrical_1rad_a1_1 = efficiency_electrical_1rad_a1_1[sort_idx_1rad_a1_1]
    efficiency_electrical_1rad_a1_2 = efficiency_electrical_1rad_a1_2[sort_idx_1rad_a1_2]
    efficiency_electrical_1rad_a1_3 = efficiency_electrical_1rad_a1_3[sort_idx_1rad_a1_3]

    efficiency_1rad_original = np.interp(measured_torque_1rad_x1c_1, measured_torque_1rad_original, efficiency_1rad_original)
    efficiency_electrical_1rad_original = np.interp(measured_torque_1rad_x1c_1, measured_torque_1rad_original, efficiency_electrical_1rad_original)

    efficiency_mean_1 = np.mean([efficiency_1rad_x1c_1, efficiency_1rad_x1c_2, efficiency_1rad_x1c_3], axis=0)
    efficiency_mean_2 = np.mean([efficiency_1rad_a1_1, efficiency_1rad_a1_2, efficiency_1rad_a1_3], axis=0)
    efficiency_std_1 = np.std([efficiency_1rad_x1c_1, efficiency_1rad_x1c_2, efficiency_1rad_x1c_3], axis=0)
    efficiency_std_2 = np.std([efficiency_1rad_a1_1, efficiency_1rad_a1_2, efficiency_1rad_a1_3], axis=0)

    efficiency_electrical_mean_1 = np.mean([efficiency_electrical_1rad_x1c_1, efficiency_electrical_1rad_x1c_2, efficiency_electrical_1rad_x1c_3], axis=0)
    efficiency_electrical_mean_2 = np.mean([efficiency_electrical_1rad_a1_1, efficiency_electrical_1rad_a1_2, efficiency_electrical_1rad_a1_3], axis=0)
    efficiency_electrical_std_1 = np.std([efficiency_electrical_1rad_x1c_1, efficiency_electrical_1rad_x1c_2, efficiency_electrical_1rad_x1c_3], axis=0)
    efficiency_electrical_std_2 = np.std([efficiency_electrical_1rad_a1_1, efficiency_electrical_1rad_a1_2, efficiency_electrical_1rad_a1_3], axis=0)

    torque_error_mean_1 = np.mean([target_torque_1rad_x1c_1 - measured_torque_1rad_x1c_1, target_torque_1rad_x1c_2 - measured_torque_1rad_x1c_1, target_torque_1rad_x1c_3 - measured_torque_1rad_x1c_3], axis=0)
    torque_error_mean_2 = np.mean([target_torque_1rad_a1_1 - measured_torque_1rad_a1_1, target_torque_1rad_a1_2 - measured_torque_1rad_a1_1, target_torque_1rad_a1_3 - measured_torque_1rad_a1_3], axis=0)
    torque_error_std_1 = np.std([target_torque_1rad_x1c_1 - measured_torque_1rad_x1c_1, target_torque_1rad_x1c_2 - measured_torque_1rad_x1c_1, target_torque_1rad_x1c_3 - measured_torque_1rad_x1c_3], axis=0)
    torque_error_std_2 = np.std([target_torque_1rad_a1_1 - measured_torque_1rad_a1_1, target_torque_1rad_a1_2 - measured_torque_1rad_a1_1, target_torque_1rad_a1_3 - measured_torque_1rad_a1_3], axis=0)


    fig, ax1 = plt.subplots(figsize=(12, 6))
    # Create twin axis for torque tracking
    ax2 = ax1.twinx()
    
    # First axis for mechanical efficiency
    ax1.plot(measured_torque_1rad_x1c_1, efficiency_mean_1, label="effi. (mech), P1", color="#db4437", linewidth=line_width)
    ax1.plot(measured_torque_1rad_x1c_1, efficiency_mean_2, label="effi. (mech), P2", color="#f4b400", linewidth=line_width)
    ax1.fill_between(measured_torque_1rad_x1c_1, efficiency_mean_1 - efficiency_std_1, efficiency_mean_1 + efficiency_std_1, color="#db4437", alpha=0.2)
    ax1.fill_between(measured_torque_1rad_x1c_1, efficiency_mean_2 - efficiency_std_2, efficiency_mean_2 + efficiency_std_2, color="#f4b400", alpha=0.2)

    ax1.set_ylabel("Efficiency (%)")
    
    ax1.plot(measured_torque_1rad_x1c_1, efficiency_electrical_mean_1, label="effi. (elec), P1", color="#db4437", linestyle="--", linewidth=line_width)
    ax1.plot(measured_torque_1rad_x1c_1, efficiency_electrical_mean_2, label="effi. (elec), P2", color="#f4b400", linestyle="--", linewidth=line_width)
    ax1.fill_between(measured_torque_1rad_x1c_1, efficiency_electrical_mean_1 - efficiency_electrical_std_1, efficiency_electrical_mean_1 + efficiency_electrical_std_1, color="#db4437", alpha=0.2)
    ax1.fill_between(measured_torque_1rad_x1c_1, efficiency_electrical_mean_2 - efficiency_electrical_std_2, efficiency_electrical_mean_2 + efficiency_electrical_std_2, color="#f4b400", alpha=0.2)


    # Second axis for torque tracking
    ax2.plot(measured_torque_1rad_x1c_1, torque_error_mean_1, label="torque error, P1", color="#4285f4", linewidth=line_width)
    ax2.plot(measured_torque_1rad_x1c_1, torque_error_mean_2, label="torque error, P2", color="#ab47bc", linewidth=line_width)
    ax2.fill_between(measured_torque_1rad_x1c_1, torque_error_mean_1 - torque_error_std_1, torque_error_mean_1 + torque_error_std_1, color="#4285f4", alpha=0.2)
    ax2.fill_between(measured_torque_1rad_x1c_1, torque_error_mean_2 - torque_error_std_2, torque_error_mean_2 + torque_error_std_2, color="#ab47bc", alpha=0.2)

    ax1.set_ylim(0.0, 1.1)

    ax2.set_ylabel("Torque Tracking Error (Nm)")
    ax2.set_ylim(-1.1, 1.1)

    ax1.set_xlim(0, 25)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    
    # Remove the duplicate legend setting and font setting
    # ax1.legend(lines1 + lines2, labels1 + labels2, loc="lower right", ncol=2)
    # ax2.legend(lines1 + lines2, labels1 + labels2, loc="lower right", ncol=2)
    

    # Create a single, properly formatted legend inside the plot area
    font_path = "berkeley_humanoid_lite_lowlevel/experiments/assets/Arimo-Regular.ttf"
    font_size = 20
    font = FontProperties(fname=font_path, size=font_size)
    ax2.legend(lines1 + lines2, labels1 + labels2, 
              loc="lower right", ncol=1, prop=font)

    ax1.grid(True, linestyle="--", zorder=-2)
    ax1.set_xlabel("Torque (Nm)")

    format_plot(ax1)
    format_plot(ax2)

    plt.savefig("berkeley_humanoid_lite_lowlevel/experiments/figures/experiment-actuator-efficiency-rebuttal.pdf", bbox_inches="tight")

    plt.show()
    plt.close()


# plot_rebuttal_efficiency()






def plot_rebuttal_benchmark():
    datapoints = {
        "Duke": {
            "power factor": 2.28,
            "cost": 16000,
            "ndof": 10,
            "open_sourced": 2,
        },
        "Berkeley Humanoid": {
            "power factor": 4.34,
            "cost": 10000,
            "ndof": 12,
            "open_sourced": 0,
        },
        "BRUCE": {
            "power factor": 1.50,
            "cost": 6500,
            "ndof": 16,
            "open_sourced": 0,
        },
        "Unitree H1": {
            "power factor": 2.85,
            "cost": 70000,
            "ndof": 19,
            "open_sourced": 0,
        },
        "Robotis OP3": {
            "power factor": 4.68,
            "cost": 11000,
            "ndof": 20,
            "open_sourced": 2,
        },
        "Booster T1": {
            "power factor": 3.21,
            "cost": 34000,
            "ndof": 23,
            "open_sourced": 0,
        },
        "NAO H25": {
            "power factor": 5.02,
            "cost": 14000,
            "ndof": 23,
            "open_sourced": 0,
        },
        "Unitree G1": {
            "power factor": 3.46,
            "cost": 57000,
            "ndof": 29,
            "open_sourced": 0,
        },
        "ToddlerBot": {
            "power factor": 2.74,
            "cost": 6000,
            "ndof": 30,
            "open_sourced": 2,
        },
        "Fourier GR1": {
            "power factor": 1.18,
            "cost": 110000,
            "ndof": 32,
            "open_sourced": 0,
        },
        "Ours": {
            "power factor": 3.00,
            "cost": 5000,
            "ndof": 22,
            "open_sourced": 2,
        },

        "iCub": {
            "power factor": 2.88625,
            "cost": 300000,
            "ndof": 29,
            "open_sourced": 2,
        },
        "Poppy": {
            "power factor": 2.460,
            "cost": 10000,
            "ndof": 25,
            "open_sourced": 2,
        },
        "PM01": {
            "power factor": 4.289,
            "cost": 13700,
            "ndof": 23,
            "open_sourced": 0,
        },
        "Digit": {
            "power factor": 0.0967,
            "cost": 250000,
            "ndof": 20,
            "open_sourced": 0,
        },
    }



    fig, ax1 = plt.subplots(figsize=(12, 6))

    # Explicitly set tick positions
    tick_positions = [4000, 6000, 8000, 10000, 20000, 40000, 60000, 80000, 100000, 200000, 400000]

    # draw a line for fixed performance factor per dollar cost
    # performance per kdollar
    costs = np.logspace(np.log10(tick_positions[0]), np.log10(tick_positions[-1]), 100)

    perf_per_kdollar = 0.001
    ax1.plot(costs, (costs/1000) * perf_per_kdollar, color="#666666", linestyle="--", linewidth=1)
    c = 250000
    ax1.text(c, (c/1000) * perf_per_kdollar, "$\\varphi$ = 0.001", fontsize=20, color="#444444")


    perf_per_kdollar = 0.003
    ax1.plot(costs, (costs/1000) * perf_per_kdollar, color="#666666", linestyle="--", linewidth=1)
    c = 83333
    ax1.text(c, (c/1000) * perf_per_kdollar, "$\\varphi$ = 0.003", fontsize=20, color="#444444")

    perf_per_kdollar = 0.01
    ax1.plot(costs, (costs/1000) * perf_per_kdollar, color="#666666", linestyle="--", linewidth=1)
    c = 28000
    ax1.text(c, (c/1000) * perf_per_kdollar, "$\\varphi$ = 0.01", fontsize=20, color="#444444")

    perf_per_kdollar = 0.03
    ax1.plot(costs, (costs/1000) * perf_per_kdollar, color="#666666", linestyle="--", linewidth=1)
    c = 9333
    ax1.text(c, (c/1000) * perf_per_kdollar, "$\\varphi$ = 0.03", fontsize=20, color="#444444")


    # plot cost vs power factor
    for robot in datapoints:
        performance_factor = datapoints[robot]["power factor"] / datapoints[robot]["ndof"]

        match datapoints[robot]["open_sourced"]:
            case 0:
                ax1.scatter(datapoints[robot]["cost"], performance_factor, label="Proprietary", color="#546e7a", marker="o", facecolors='none', edgecolors="#546e7a", s=100, zorder=10, linewidths=3.0)
            case 1:
                raise NotImplementedError("Partially open-source robots are not supported yet")
                ax1.scatter(datapoints[robot]["cost"], performance_factor, label=robot, color="#db4437", marker="o", facecolors='none', edgecolors="#db4437", s=100, zorder=10, linewidths=3.0)
            case 2:
                
                if robot == "Ours":
                    ax1.scatter(datapoints[robot]["cost"], performance_factor, label="Fully open-source", color="#ff7043", marker="o", s=100, zorder=10, linewidths=3.0)
                else:
                    ax1.scatter(datapoints[robot]["cost"], performance_factor, label="Fully open-source", color="#546e7a", marker="o", s=100, zorder=10, linewidths=3.0)

        offset_x = 0
        offset_y = 0

        if robot == "ToddlerBot":
            offset_y = -0.04
        if robot == "BRUCE":
            offset_x = 500
        if robot == "Poppy":
            offset_x = 3500
            offset_y = -0.02
        
        if robot == "Robotis OP3":
            offset_x = -4500
            offset_y = -0.02
        if robot == "NAO H25":
            offset_x = -4600
            offset_y = -0.03
        if robot == "Duke":
            offset_x = 1000
        if robot == "PM01":
            offset_y = -0.045
        
        if robot == "Booster T1":
            offset_x = -1500
        if robot == "Unitree G1":
            offset_y = -0.04
        
        ax1.text(datapoints[robot]["cost"] + offset_x, performance_factor + 0.018 + offset_y, robot, fontsize=20, ha="center", va="center")

    ax1.set_xlabel("Cost (USD)")
    ax1.set_ylabel("Performance factor $\hat{p}$")
    
    ax1.set_xscale("log")
    ax1.set_xticks(tick_positions)
    ax1.set_xlim(tick_positions[-1], tick_positions[0])
    ax1.set_ylim(0, 0.4)

    # Format x labels as k USD
    ax1.xaxis.set_major_formatter(mtick.FuncFormatter(lambda x, pos: f"{x/1000:.0f} k"))
    
    # Rotate x-tick labels by 45 degrees
    plt.setp(ax1.get_xticklabels(), rotation=45, ha='right')
    
    # grid
    ax1.grid(True, linestyle="--", zorder=0)

    font_path = "berkeley_humanoid_lite_lowlevel/experiments/assets/Arimo-Regular.ttf"
    font_size = 20
    font = FontProperties(size=font_size)

    # add legend to explain the point characters
    ax1.legend(
        handles=[
            plt.scatter([], [], s=100, color="#546e7a", marker="o", facecolors='none', edgecolors="#546e7a", linewidths=3.0),
            plt.scatter([], [], s=100, color="#546e7a", marker="o", linewidths=3.0),
        ],
        labels=[
            "Proprietary",
            "Fully open-source",
        ],
        loc="upper left",
        prop=font,
    )

    format_plot(ax1)

    plt.savefig("berkeley_humanoid_lite_lowlevel/experiments/figures/comparision.pdf", bbox_inches="tight")

    plt.show()
    plt.close()


plot_rebuttal_benchmark()