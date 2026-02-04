from pydoc import locate
import cv2
import numpy as np
import argparse
import time
import os
import subprocess
import shlex

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter

def create_mp4_writer(shape, filename, fps):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    return cv2.VideoWriter(filename, fourcc, fps, (shape[1], shape[0]))

def read_ros2_bag(bag_path, topic_name, output_name, use_x265):

    error_message = "You are attempting to use X265 encoding but "
    if use_x265:
        try:
            result = subprocess.run(["ffmpeg"], capture_output=True, text=True)
            if result.returncode == 127:
                print(error_message + "do not have ffmpeg installed")
                return
        except FileNotFoundError:
            print(error_message + "do not have ffmpeg installed")
            return
        except Exception as e:
            print(error_message + f" an unknown error occured: {e}")

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    reader.set_filter(StorageFilter(topics=[topic_name]))

    # Time Metadata
    metadata = reader.get_metadata()
    bag_start = metadata.starting_time.nanoseconds
    bag_length = metadata.duration.nanoseconds / 10e9
    start_time = time.time()
    print_time = time.time()

    # Get topic information
    topic_types = reader.get_all_topics_and_types()
    type_map = {meta.name: locate(meta.type.replace("/", ".")) for meta in topic_types}
    topic_type = type_map[topic_name]

    fps = 30
    output_process = None
    mp4_output = None
    output_configured = False

    encoding_string = "H265 Encoding" if use_x265 else "Standard Encoding"
    print(f"{encoding_string}: {topic_name}")
    # Iterate through messages
    while reader.has_next():

        _, data, timestamp = reader.read_next()
        msg = deserialize_message(data, topic_type)

        now = time.time()
        if now - print_time > 0.25:
            bag_time = timestamp - bag_start
            print(f"{(bag_time / 10e9):.2f}s / {bag_length:.2f}s", end = "\r")
            print_time = now

        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if not output_configured:
            if use_x265:
                output_process = subprocess.Popen(
                    shlex.split(f'ffmpeg -y -s {image.shape[1]}x{image.shape[0]} -pixel_format bgr24 -f rawvideo -r {fps} -i pipe: -vcodec libx265 -pix_fmt yuv420p -crf 24 {output_name}'), stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
            else: 
                # Default to standard opencv output
                mp4_output = create_mp4_writer(image.shape, output_name, fps)

            output_configured = True
        else:
            if use_x265:
                output_process.stdin.write(image.tobytes())
            else:
                mp4_output.write(image)

    # Clean up our output system
    if use_x265:
        output_process.stdin.close()
        output_process.wait()
        output_process.terminate() # just in case
    elif mp4_output is not None:
        mp4_output.release()  # Properly release the VideoWriter

    print(f"Completed {bag_length:.2f}s bag in {(time.time() - start_time):.2f}s")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", help="Path to bag folder")
    parser.add_argument("-t", "--topic", help="topic of an Image or CompressedImage to output", default="/pontus/camera_2/image_raw/compressed")
    parser.add_argument("-o", "--output", help="name of the output mp4 file", default="output.mp4")
    parser.add_argument("-x", "--x265", help="Use ffmpeg to output using H.265 encoding", action="store_true")

    args = parser.parse_args()

    if not os.path.exists(args.bag_path):
        print("The provided bag folder path does not exist: ", args.bag_path)
        return

    output_name = args.output
    if not output_name.endswith(".mp4"):
        output_name += ".mp4"

    read_ros2_bag(args.bag_path, args.topic, output_name, args.x265)

if __name__ == '__main__':
    main()