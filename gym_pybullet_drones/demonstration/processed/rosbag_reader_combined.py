'''
Description: This script reads a rosbag file and extracts the positions of the drone, payload
and round bar from the Vicon system.
The extracted data is then interpolated to fill in missing values and saved to a CSV file.

Install by running: 'pip3 install rosbags'
Usage: python3 rosbag_reader_combined.py --path <path-to-file>

All ros topics related to the positions of the drone, payload, and round bar are as follows:

Drone ROS2 Topics: /vicon/beemav/beemav
Payload ROS2 Topics: /vicon/perching_payload/perching_payload
Round Bar ROS2 Topics: /vicon/perching_round_bar/perching_round_bar

Script adapted from one created by Atar Babgei
'''

import os
import argparse
from collections import defaultdict
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
import numpy as np
import pandas as pd

# Topics to read from
TOPICS = {
    'drone': '/vicon/beemav/beemav',  # Drone ROS2 Topic
    'payload': '/vicon/perching_payload/perching_payload',  # Payload ROS2 Topic
    'round_bar': '/vicon/perching_round_bar/perching_round_bar'  # Round Bar ROS2 Topic
}

# vicon_msgs/msg/Position message definition
STRIDX_MSG = """
float32 x_trans
float32 y_trans
float32 z_trans
float32 x_rot
float32 y_rot
float32 z_rot
float32 w
string segment_name
string subject_name
int32 frame_number
string translation_type
"""

# Initialize the typestore and register the custom message type
typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(get_types_from_msg(STRIDX_MSG, 'vicon_msgs/msg/Position'))

StrIdx = typestore.types['vicon_msgs/msg/Position']


def print_connections(reader):
    """Prints all topic and msgtype information available in the rosbag."""
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)


def collect_messages(reader, topics):
    """Collects messages from specified topics in the rosbag and returns a dictionary of timestamped data."""
    data = defaultdict(lambda: {
        'drone_x': np.nan, 'drone_y': np.nan, 'drone_z': np.nan,
        'payload_x': np.nan, 'payload_y': np.nan, 'payload_z': np.nan,
        'round_bar_x': np.nan, 'round_bar_y': np.nan, 'round_bar_z': np.nan
    })

    for name, topic in topics.items():
        connections = [x for x in reader.connections if x.topic == topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            data[timestamp][f'{name}_x'] = msg.x_trans
            data[timestamp][f'{name}_y'] = msg.y_trans
            data[timestamp][f'{name}_z'] = msg.z_trans

    return data


def interpolate_data(data):
    """Interpolates missing values in the collected data."""
    df = pd.DataFrame.from_dict(data, orient='index')
    df.sort_index(inplace=True)
    df.interpolate(method='linear', inplace=True)
    df.ffill(inplace=True)
    df.bfill(inplace=True)
    return df


def save_messages_to_csv(df, csv_filename):
    """Saves the interpolated data to a CSV file."""
    df.to_csv(csv_filename, index_label='Timestamp')
    print(f"Data saved to {csv_filename}")


def main():
    """Main function to read the rosbag, process messages, interpolate data, and save them to a CSV file."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Process a rosbag file and save extracted data to a CSV file.')
    parser.add_argument('--path', required=True, help='Path to the rosbag file')
    args = parser.parse_args()
    bag_path = args.path

    with Reader(bag_path) as reader:
        print("Connections:")
        print_connections(reader)

        print("\nCollecting Positions:")
        data = collect_messages(reader, TOPICS)

        print("\nInterpolating Data:")
        df = interpolate_data(data)

        # Extract folder name from the bag path and create the CSV filename
        folder_name = os.path.basename(bag_path)
        csv_filename = f'{folder_name}.csv'
        save_messages_to_csv(df, csv_filename)


if __name__ == "__main__":
    main()
