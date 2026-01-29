#!/usr/bin/env python3
"""
Generate keyboard teleop architecture diagram as PNG.

This script creates a visual representation of the communication architecture
for the keyboard teleoperation system.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.lines as mlines

def create_architecture_diagram():
    """Create the keyboard teleop architecture diagram."""
    
    # Create figure with white background
    fig, ax = plt.subplots(1, 1, figsize=(16, 20))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 25)
    ax.axis('off')
    
    # Title
    ax.text(5, 24.5, 'Keyboard Teleop Communication Architecture', 
            ha='center', va='top', fontsize=18, fontweight='bold')
    
    # Color scheme
    terminal1_color = '#E8F4F8'  # Light blue
    terminal2_color = '#FFF4E6'  # Light orange
    ros2_color = '#E8F5E9'       # Light green
    robot_color = '#F3E5F5'      # Light purple
    
    # ============================================================
    # TERMINAL 1: Keyboard Teleop
    # ============================================================
    y_base = 22
    
    # Terminal 1 box
    terminal1_box = FancyBboxPatch(
        (0.5, y_base-3), 4, 3,
        boxstyle="round,pad=0.1", 
        edgecolor='#0277BD', linewidth=2,
        facecolor=terminal1_color
    )
    ax.add_patch(terminal1_box)
    
    ax.text(2.5, y_base-0.3, 'TERMINAL 1', 
            ha='center', va='center', fontsize=12, fontweight='bold')
    ax.text(2.5, y_base-0.7, 'keyboard_ee_teleop.py', 
            ha='center', va='center', fontsize=10, style='italic')
    
    # Keyboard Listener box
    kb_box = FancyBboxPatch(
        (0.7, y_base-2.5), 1.2, 0.8,
        boxstyle="round,pad=0.05", 
        edgecolor='black', linewidth=1,
        facecolor='white'
    )
    ax.add_patch(kb_box)
    ax.text(1.3, y_base-2.1, 'Keyboard\nListener\n(pynput)', 
            ha='center', va='center', fontsize=8)
    
    # Process box
    process_box = FancyBboxPatch(
        (2.1, y_base-2.5), 1.2, 0.8,
        boxstyle="round,pad=0.05", 
        edgecolor='black', linewidth=1,
        facecolor='white'
    )
    ax.add_patch(process_box)
    ax.text(2.7, y_base-2.1, 'Process\nKeys &\nUpdate Pose', 
            ha='center', va='center', fontsize=8)
    
    # ROS2 Publisher box
    pub_box = FancyBboxPatch(
        (3.5, y_base-2.5), 1.2, 0.8,
        boxstyle="round,pad=0.05", 
        edgecolor='black', linewidth=1,
        facecolor='white'
    )
    ax.add_patch(pub_box)
    ax.text(4.1, y_base-2.1, 'ROS2\nPublisher\n10 Hz', 
            ha='center', va='center', fontsize=8)
    
    # Arrows between boxes
    arrow1 = FancyArrowPatch(
        (1.9, y_base-2.1), (2.1, y_base-2.1),
        arrowstyle='->', mutation_scale=20, linewidth=2, color='black'
    )
    ax.add_patch(arrow1)
    
    arrow2 = FancyArrowPatch(
        (3.3, y_base-2.1), (3.5, y_base-2.1),
        arrowstyle='->', mutation_scale=20, linewidth=2, color='black'
    )
    ax.add_patch(arrow2)
    
    # User input annotation
    ax.text(0.6, y_base-2.8, '↑↓←→ QWASZX +/-', 
            ha='left', va='top', fontsize=7, style='italic', color='#1565C0')
    
    # ============================================================
    # ROS2 TOPIC
    # ============================================================
    y_ros2 = 17
    
    ros2_box = FancyBboxPatch(
        (1.5, y_ros2-1), 7, 2,
        boxstyle="round,pad=0.1", 
        edgecolor='#2E7D32', linewidth=3,
        facecolor=ros2_color
    )
    ax.add_patch(ros2_box)
    
    ax.text(5, y_ros2+0.7, 'ROS2 Topic: /ee_command', 
            ha='center', va='center', fontsize=12, fontweight='bold')
    ax.text(5, y_ros2+0.3, 'geometry_msgs/PoseStamped', 
            ha='center', va='center', fontsize=10, style='italic')
    ax.text(5, y_ros2-0.1, 'Position (x, y, z) + Orientation (x, y, z, w)', 
            ha='center', va='center', fontsize=9)
    ax.text(5, y_ros2-0.5, 'Publishing Rate: 10 Hz  |  QoS Depth: 10', 
            ha='center', va='center', fontsize=9)
    
    # Arrows to/from ROS2 topic
    arrow_pub = FancyArrowPatch(
        (4.1, y_base-2.5), (5, y_ros2+1),
        arrowstyle='->', mutation_scale=25, linewidth=3, 
        color='#2E7D32', linestyle='--'
    )
    ax.add_patch(arrow_pub)
    ax.text(4.3, y_base-3.5, 'Publishes', 
            ha='left', va='center', fontsize=9, color='#2E7D32', fontweight='bold')
    
    # ============================================================
    # TERMINAL 2: Control Loop
    # ============================================================
    y_term2 = 15
    
    terminal2_box = FancyBboxPatch(
        (0.5, 1), 9, y_term2-1.5,
        boxstyle="round,pad=0.1", 
        edgecolor='#E65100', linewidth=2,
        facecolor=terminal2_color
    )
    ax.add_patch(terminal2_box)
    
    ax.text(5, y_term2-0.3, 'TERMINAL 2', 
            ha='center', va='center', fontsize=12, fontweight='bold')
    ax.text(5, y_term2-0.7, 'run_franka_sim_ros2_teleop.py', 
            ha='center', va='center', fontsize=10, style='italic')
    
    # Multi-rate control loop header
    loop_box = FancyBboxPatch(
        (0.8, 2), 8.4, y_term2-2.5,
        boxstyle="round,pad=0.1", 
        edgecolor='#BF360C', linewidth=2,
        facecolor='#FFE0B2'
    )
    ax.add_patch(loop_box)
    
    ax.text(5, y_term2-1.2, 'Multi-Rate Control Loop (Base: 100 Hz)', 
            ha='center', va='center', fontsize=11, fontweight='bold')
    ax.text(5, y_term2-1.6, 'Uses decimation for different task rates', 
            ha='center', va='center', fontsize=9, style='italic')
    
    # Arrow from ROS2 to control loop
    arrow_sub = FancyArrowPatch(
        (5, y_ros2-1), (5, y_term2-1.7),
        arrowstyle='->', mutation_scale=25, linewidth=3, 
        color='#2E7D32', linestyle='--'
    )
    ax.add_patch(arrow_sub)
    ax.text(5.3, y_ros2-1.5, 'Subscribes', 
            ha='left', va='center', fontsize=9, color='#2E7D32', fontweight='bold')
    
    # ============================================================
    # TASK 3: ROS2 Spin + Visualization
    # ============================================================
    y_task3 = 12
    
    task3_box = FancyBboxPatch(
        (1, y_task3-2), 8, 1.8,
        boxstyle="round,pad=0.08", 
        edgecolor='#6A1B9A', linewidth=2,
        facecolor='#E1BEE7'
    )
    ax.add_patch(task3_box)
    
    ax.text(5, y_task3-0.3, 'TASK 3: ROS2 Spin + Visualization', 
            ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(5, y_task3-0.6, '50 Hz (decimation=2)', 
            ha='center', va='center', fontsize=9, style='italic')
    
    task3_content = [
        '• spin_ros2_once() - Non-blocking',
        '• EEPoseSubscriber callback triggered',
        '• Store in EEPoseCommandHandler (thread-safe)',
        '• Update MuJoCo visualization',
        '• Publish /joint_states'
    ]
    
    y_offset = y_task3-0.9
    for item in task3_content:
        ax.text(1.3, y_offset, item, ha='left', va='top', fontsize=7)
        y_offset -= 0.25
    
    # ============================================================
    # TASK 1: Planner Task
    # ============================================================
    y_task1 = 9
    
    task1_box = FancyBboxPatch(
        (1, y_task1-2.5), 8, 2.3,
        boxstyle="round,pad=0.08", 
        edgecolor='#1565C0', linewidth=2,
        facecolor='#BBDEFB'
    )
    ax.add_patch(task1_box)
    
    ax.text(5, y_task1-0.3, 'TASK 1: Planner Task', 
            ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(5, y_task1-0.6, '50 Hz (decimation=2)', 
            ha='center', va='center', fontsize=9, style='italic')
    
    # Decision boxes
    decision_y = y_task1-1.2
    
    # IF box
    if_box = FancyBboxPatch(
        (1.5, decision_y-0.8), 3.2, 0.7,
        boxstyle="round,pad=0.05", 
        edgecolor='#388E3C', linewidth=1.5,
        facecolor='#C8E6C9'
    )
    ax.add_patch(if_box)
    ax.text(3.1, decision_y-0.45, 'IF ROS2 command valid:', 
            ha='center', va='center', fontsize=8, fontweight='bold')
    ax.text(3.1, decision_y-0.65, 'Create TaskSpaceReference\nMode = TRACK', 
            ha='center', va='center', fontsize=7)
    
    # ELSE box
    else_box = FancyBboxPatch(
        (5.3, decision_y-0.8), 3.2, 0.7,
        boxstyle="round,pad=0.05", 
        edgecolor='#D32F2F', linewidth=1.5,
        facecolor='#FFCDD2'
    )
    ax.add_patch(else_box)
    ax.text(6.9, decision_y-0.45, 'ELSE (no/stale command):', 
            ha='center', va='center', fontsize=8, fontweight='bold')
    ax.text(6.9, decision_y-0.65, 'HoldPlanner returns None\nMode = HOLD', 
            ha='center', va='center', fontsize=7)
    
    ax.text(5, decision_y-1.3, '↓ Bridge.planner_tick(reference)', 
            ha='center', va='center', fontsize=8)
    
    # ============================================================
    # TASK 2: Controller Task
    # ============================================================
    y_task2 = 5.5
    
    task2_box = FancyBboxPatch(
        (1, y_task2-2.5), 8, 2.3,
        boxstyle="round,pad=0.08", 
        edgecolor='#F57C00', linewidth=2,
        facecolor='#FFE0B2'
    )
    ax.add_patch(task2_box)
    
    ax.text(5, y_task2-0.3, 'TASK 2: Controller Task', 
            ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(5, y_task2-0.6, '100 Hz (decimation=1) - Base Rate', 
            ha='center', va='center', fontsize=9, style='italic')
    
    # Controller flow
    controller_flow = [
        '1. CrispRobotInterface.get_state() → RobotState',
        '2. Bridge.control_tick(state)',
        '3. MinkIKController.step(state)',
    ]
    
    y_offset = y_task2-1.0
    for item in controller_flow:
        ax.text(1.3, y_offset, item, ha='left', va='top', fontsize=8)
        y_offset -= 0.3
    
    # IK boxes
    ik_y = y_task2-1.9
    
    # TRACK box
    track_box = FancyBboxPatch(
        (1.5, ik_y-0.5), 3.2, 0.45,
        boxstyle="round,pad=0.05", 
        edgecolor='#388E3C', linewidth=1.5,
        facecolor='#C8E6C9'
    )
    ax.add_patch(track_box)
    ax.text(3.1, ik_y-0.275, 'TRACK: Solve IK\n20 iter, 1e-4 threshold', 
            ha='center', va='center', fontsize=7)
    
    # HOLD box
    hold_box = FancyBboxPatch(
        (5.3, ik_y-0.5), 3.2, 0.45,
        boxstyle="round,pad=0.05", 
        edgecolor='#D32F2F', linewidth=1.5,
        facecolor='#FFCDD2'
    )
    ax.add_patch(hold_box)
    ax.text(6.9, ik_y-0.275, 'HOLD: Command\ncurrent position', 
            ha='center', va='center', fontsize=7)
    
    # ============================================================
    # Robot Interface
    # ============================================================
    y_robot = 2
    
    robot_interface_box = FancyBboxPatch(
        (2, y_robot-0.8), 6, 0.7,
        boxstyle="round,pad=0.08", 
        edgecolor='#4A148C', linewidth=2,
        facecolor='#E1BEE7'
    )
    ax.add_patch(robot_interface_box)
    
    ax.text(5, y_robot-0.45, 'CrispRobotInterface (crisp_py wrapper)', 
            ha='center', va='center', fontsize=9, fontweight='bold')
    ax.text(5, y_robot-0.65, 'ROS2 Action Client → joint_impedance_controller', 
            ha='center', va='center', fontsize=7, style='italic')
    
    # Arrow to robot
    arrow_robot = FancyArrowPatch(
        (5, y_task2-2.5), (5, y_robot-0.1),
        arrowstyle='->', mutation_scale=20, linewidth=2, color='#4A148C'
    )
    ax.add_patch(arrow_robot)
    
    # ============================================================
    # FR3 Robot Hardware/Sim
    # ============================================================
    y_hw = 0.5
    
    hardware_box = FancyBboxPatch(
        (2.5, y_hw-0.4), 5, 0.7,
        boxstyle="round,pad=0.08", 
        edgecolor='#6A1B9A', linewidth=3,
        facecolor=robot_color
    )
    ax.add_patch(hardware_box)
    
    ax.text(5, y_hw-0.05, 'FR3 Robot Hardware/Simulation', 
            ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(5, y_hw-0.25, 'ROS2 Control | Joint Impedance Controller', 
            ha='center', va='center', fontsize=8, style='italic')
    
    # Arrow to hardware
    arrow_hw = FancyArrowPatch(
        (5, y_robot-0.8), (5, y_hw+0.3),
        arrowstyle='->', mutation_scale=25, linewidth=3, color='#6A1B9A'
    )
    ax.add_patch(arrow_hw)
    ax.text(5.3, 1, 'Joint\nCommands', 
            ha='left', va='center', fontsize=8, color='#6A1B9A', fontweight='bold')
    
    # ============================================================
    # Legend
    # ============================================================
    legend_elements = [
        mpatches.Patch(facecolor=terminal1_color, edgecolor='#0277BD', label='Keyboard Teleop Process'),
        mpatches.Patch(facecolor=ros2_color, edgecolor='#2E7D32', label='ROS2 Communication'),
        mpatches.Patch(facecolor=terminal2_color, edgecolor='#E65100', label='Control Loop Process'),
        mpatches.Patch(facecolor=robot_color, edgecolor='#6A1B9A', label='Robot Hardware/Sim'),
    ]
    
    ax.legend(handles=legend_elements, loc='upper right', fontsize=9, 
              framealpha=0.9, bbox_to_anchor=(0.98, 0.98))
    
    # Add frequency annotations
    freq_text = (
        'Loop Rates:\n'
        '  Task 1 (Planner): 50 Hz\n'
        '  Task 2 (Controller): 100 Hz\n'
        '  Task 3 (ROS2/Viz): 50 Hz\n'
        '  Keyboard Pub: 10 Hz'
    )
    ax.text(0.7, 15, freq_text, ha='left', va='top', fontsize=8,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    return fig

def main():
    """Generate and save the architecture diagram."""
    print("Generating keyboard teleop architecture diagram...")
    
    fig = create_architecture_diagram()
    
    # Save as PNG
    output_path = '/home/ben/avant_robot_interface/docs/keyboard_teleop_architecture.png'
    fig.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"✓ Diagram saved to: {output_path}")
    
    # Also save as high-res version
    output_path_hires = '/home/ben/avant_robot_interface/docs/keyboard_teleop_architecture_hires.png'
    fig.savefig(output_path_hires, dpi=600, bbox_inches='tight', facecolor='white')
    print(f"✓ High-res diagram saved to: {output_path_hires}")
    
    plt.close()
    print("\nDone! The architecture diagram has been generated.")

if __name__ == '__main__':
    main()
