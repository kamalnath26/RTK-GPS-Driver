import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.interpolate import splprep, splev

# File paths to your bag files
current_path = os.path.join(os.getcwd(), "../data/ros_bags_rtk")
moving_data_with_buildings = os.path.join(current_path, 'occuluded_walking.bag')
moving_data_without_buildings = os.path.join(current_path, 'open_walking.bag')

# Function to read bag file and return processed data
def read_bag_data(bag_file):
    utm_eastings = []
    utm_northings = []
    
    # Read the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/gps']):
            utm_eastings.append(msg.utm_easting)
            utm_northings.append(msg.utm_northing)
    
    # Convert to numpy arrays for easier manipulation
    utm_eastings = np.array(utm_eastings)
    utm_northings = np.array(utm_northings)
    
    # Subtract the first point to remove the offset
    utm_eastings -= utm_eastings[0]
    utm_northings -= utm_northings[0]

    return utm_eastings, utm_northings

# Function to read bag file and return processed data
def read_bag_data_altitude(bag_file):
    altitudes = []
    timestamps = []
    
    # Read the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/gps']):
            altitudes.append(msg.altitude)
            timestamps.append(t.to_sec())  # Convert ROS time to seconds
    
    # Convert to numpy arrays for easier manipulation
    altitudes = np.array(altitudes)
    timestamps = np.array(timestamps)
    
    # Subtract the first point to remove the offset
    altitudes -= altitudes[0]
    timestamps -= timestamps[0]

    return altitudes, timestamps

utm_eastings_with, utm_northings_with = read_bag_data(moving_data_with_buildings)
altitudes_with, timestamps_with = read_bag_data_altitude(moving_data_with_buildings)
utm_eastings_without, utm_northings_without = read_bag_data(moving_data_without_buildings)
altitudes_without, timestamps_without = read_bag_data_altitude(moving_data_without_buildings)

# Fit a spline to the GPS data
tck_with, u_with = splprep([utm_eastings_with, utm_northings_with], s=0)  # s=0 for no smoothing
spline_points_with = splev(u_with, tck_with)
tck_without, u_without = splprep([utm_eastings_without, utm_northings_without], s=0)  # s=0 for no smoothing
spline_points_without = splev(u_without, tck_without)


# Print some altitude statistics
print(f'Min Altitude With: {np.min(altitudes_with):.2f} m')
print(f'Max Altitude With: {np.max(altitudes_with):.2f} m')
print(f'Mean Altitude With: {np.mean(altitudes_with):.2f} m')

print(f'Min Altitude Without: {np.min(altitudes_without):.2f} m')
print(f'Max Altitude Without: {np.max(altitudes_without):.2f} m')
print(f'Mean Altitude Without: {np.mean(altitudes_without):.2f} m')


# error calculations
error_with = np.sqrt((spline_points_with[0] - utm_eastings_with )**2 + (spline_points_with[1] - utm_northings_with)**2)
error_without = np.sqrt((spline_points_without[0] - utm_eastings_without  )**2 + (spline_points_without[1] - utm_northings_without)**2)
print(f"Error with buildings: {np.mean(error_with)}")
print(f"Error without buildings: {np.mean(error_without)}")


# Plotting

################################ separate plots########################################
# plt.figure(figsize=(10, 6))
plt.figure()
plt.scatter(utm_eastings_with, utm_northings_with, alpha=0.7, label='GPS Data Points', color='blue')
plt.plot(spline_points_with[0], spline_points_with[1], color='red', label='Spline Fit', linewidth=2)

# Add labels and title
plt.suptitle('Moving Data With Buildings')
plt.title('Northing vs. Easting Scatterplot with Line of Best Fit')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.axhline(0, color='grey', lw=0.5, ls='--')  # Optional: horizontal line at y=0
plt.axvline(0, color='grey', lw=0.5, ls='--')  # Optional: vertical line at x=0
plt.grid()
plt.legend()
plt.tight_layout()

plt.figure()
plt.plot(timestamps_with, altitudes_with, color='blue', marker='o', linestyle='-', alpha=0.7)

# Add labels and title
plt.suptitle('Moving Altitude Data With Buildings')
plt.title('Altitude vs. Time Plot')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.grid()
plt.tight_layout()

plt.figure()
plt.scatter(utm_eastings_without, utm_northings_without, alpha=0.7, label='GPS Data Points', color='blue')
plt.plot(spline_points_without[0], spline_points_without[1], color='red', label='Spline Fit', linewidth=2)

# Add labels and title
plt.suptitle('Moving Data Without Buildings')
plt.title('Northing vs. Easting Scatterplot with Line of Best Fit')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.axhline(0, color='grey', lw=0.5, ls='--')  # Optional: horizontal line at y=0
plt.axvline(0, color='grey', lw=0.5, ls='--')  # Optional: vertical line at x=0
plt.grid()
plt.legend()
plt.tight_layout()

plt.figure()
plt.plot(timestamps_without, altitudes_without, color='blue', marker='o', linestyle='-', alpha=0.7)

# Add labels and title
plt.suptitle('Moving Altitude Data Without Buildings')
plt.title('Altitude vs. Time Plot')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.grid()
plt.tight_layout()
plt.show()

