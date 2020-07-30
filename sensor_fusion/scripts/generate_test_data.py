import csv
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.legend import Legend

SRR_SENSOR_ID = 0
LRR_SENSOR_ID = 1
CAM_SENSOR_ID = 2

NO_OBJECT_ID = 0
PEDESTRIAN_OBJECT_ID = 1
CAR_OBJECT_ID = 2
TREE_OBJECT_ID = 3

width = 300
height = 300
num_samples = 150
num_detections = 20

segment_x = width/3;
segment_y = height/3

car_ped_x_offset = -150

srr_range = [car_ped_x_offset,0,width, segment_y]
lrr_range = [car_ped_x_offset + segment_x, 0, segment_x, height]
cam_range = [car_ped_x_offset + 25, 0, width-50, segment_y*2]

static_tree_x = 125
static_tree_y = 175

def range_contains(rect, x,y):
    if (x < rect[0] or y < rect[1]):
        return False
    x -= rect[0]
    y -= rect[1]    
    return x < rect[2] and y < rect[3]

def add_measurement_with_noise(measurement, sensor_id, object_id, timestamp, x, y, spread, num_samples):
    for i in range(0, num_samples):
        measurement.append(sensor_id)
        measurement.append(object_id)
        measurement.append(timestamp)

        # add some gaussian noise to coordinate
        measurement.append(np.random.normal(x, spread))
        measurement.append(np.random.normal(y, spread))

def generate_measurements(srr, lrr, cam, obj_id, x, y, timestamp):
    srr_spread = 0.5
    lrr_spread = 0.5
    cam_spread = 0.5

    if range_contains(srr_range, x, y):
        add_measurement_with_noise(srr, SRR_SENSOR_ID, obj_id,
                                        timestamp, x, y, srr_spread, num_detections)
    if range_contains(lrr_range, x, y):
        add_measurement_with_noise(lrr, LRR_SENSOR_ID, obj_id,
                                        timestamp, x, y, lrr_spread, num_detections)
    if range_contains(cam_range, x, y):
        add_measurement_with_noise(cam, CAM_SENSOR_ID, obj_id,
                                        timestamp, x, y, cam_spread, 1)

def generate_empty_measurement_if_needed(srr, lrr, cam, obj_id, x, y, timestamp):
    if not range_contains(srr_range, x, y):
        add_measurement_with_noise(srr, SRR_SENSOR_ID, NO_OBJECT_ID,
                                        timestamp, 0, 0, 0, 1)
    if not range_contains(lrr_range, x, y):
        add_measurement_with_noise(lrr, LRR_SENSOR_ID, NO_OBJECT_ID,
                                        timestamp, 0, 0, 0, 1)
    if not range_contains(cam_range, x, y):
        add_measurement_with_noise(cam, CAM_SENSOR_ID, NO_OBJECT_ID,
                                        timestamp, 0, 0, 0, 1)
        
def chop_measurement(measurement, n):
    for i in range(0, len(measurement), n):
        yield measurement[i:i+n]

def write_measurement_to_csv_file(measurement, filename):
    entries = list(chop_measurement(measurement, 5))
    with open(filename, mode='w') as csv_file:
        fieldnames = ['objId', 'x', 'y', 'timestamp']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for entry in entries:
            writer.writerow({'objId': entry[1], \
                                'x': entry[3], 'y': entry[4], 'timestamp': entry[2]})

def write_ground_truth_to_csv_file(obj_id, x, y, filename, m, write_header=False):
    with open(filename, mode=m) as csv_file:
        fieldnames = ['objId', 'x', 'y', 'timestamp']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for i in range(len(x)):
            writer.writerow({'objId': obj_id, \
                            'x': x[i], 'y': y[i], 'timestamp': i})


def generate_ground_truth(width, scale, anchor_x, anchor_y, spacing, num_samples):
    global car_ped_x_offset
    ground_x = np.linspace(car_ped_x_offset, 150, num_samples)
    ground_x = list(np.asarray(ground_x))
    ground_y = np.interp(ground_x, anchor_x, anchor_y)
    ground_y = list(np.asarray(ground_y) + scale + spacing)
    return ground_x, ground_y

if __name__ == '__main__':
    # use seed to get determined randomness for signal noise
    np.random.seed(21)

    # Input for pedestrian samples
    spacing = 10
    scale = srr_range[3]/2 - spacing
    anchor_x = np.linspace(car_ped_x_offset, width, 10)
    anchor_y = scale*np.sin(anchor_x*0.2)
    pedestrian_x, pedestrian_y = \
        generate_ground_truth(width, scale, anchor_x, anchor_y, spacing, num_samples)

    # Input for car samples
    offset = 10
    scale = cam_range[3]/4 - offset
    anchor_x = np.linspace(car_ped_x_offset, width, 10)
    anchor_y = scale*np.sin(anchor_x*0.175)
    car_x, car_y = \
        generate_ground_truth(width, scale, anchor_x, anchor_y, srr_range[3] + offset, num_samples)


    tree_x = []
    tree_y = []
    for i in range(num_samples):
        tree_x.append(static_tree_x + car_ped_x_offset)
        tree_y.append(static_tree_y)

    # Generate measurements
    timestamp = 0
    current_pedestrian_sample = 0
    current_car_sample = 0
    current_tree_sample = 0
    start_pedestrian_on_timestap = 0
    start_car_on_timestap = 10

    srr = []
    lrr = []
    cam = []

    while current_pedestrian_sample < num_samples and current_car_sample < num_samples:
        if current_pedestrian_sample < num_samples and timestamp >= start_pedestrian_on_timestap:
            x = pedestrian_x[current_pedestrian_sample]
            y = pedestrian_y[current_pedestrian_sample]
            generate_measurements(srr, lrr, cam, PEDESTRIAN_OBJECT_ID, x, y, timestamp)

        if current_car_sample < num_samples and timestamp >= start_car_on_timestap:
            x = car_x[current_car_sample]
            y = car_y[current_car_sample]
            generate_measurements(srr, lrr, cam, CAR_OBJECT_ID, x, y, timestamp)

        if current_tree_sample < num_samples:
            x = tree_x[current_tree_sample]
            y = tree_y[current_tree_sample]
            generate_measurements(srr, lrr, cam, TREE_OBJECT_ID, x, y, timestamp)

        generate_empty_measurement_if_needed(srr, lrr, cam, TREE_OBJECT_ID, x, y, timestamp)

        if timestamp >= start_pedestrian_on_timestap:
            current_pedestrian_sample += 1
        if timestamp >= start_car_on_timestap:
            current_car_sample += 1
        current_tree_sample += 1
        timestamp += 1

    write_measurement_to_csv_file(srr, 'short_range_radar.csv')
    write_measurement_to_csv_file(lrr, 'long_range_radar.csv')
    write_measurement_to_csv_file(cam, 'camera.csv')
    write_ground_truth_to_csv_file(PEDESTRIAN_OBJECT_ID, \
                                    pedestrian_x, pedestrian_y, \
                                    'ground_truth.csv', 'w+', True)
    write_ground_truth_to_csv_file(CAR_OBJECT_ID, \
                                    car_x, car_y, \
                                    'ground_truth.csv', 'a')
    write_ground_truth_to_csv_file(TREE_OBJECT_ID, \
                                    tree_x, tree_y, \
                                    'ground_truth.csv', 'a')

    # Visualize samples
    plt.figure(figsize=(7, 5))
    lrr_rect = plt.Rectangle((lrr_range[0], lrr_range[1]),
                              lrr_range[2], lrr_range[3],
                              color='y', edgecolor='None', alpha=0.5,
                              label='LRR range')
    plt.gca().add_patch(lrr_rect)
    cam_rect = plt.Rectangle((cam_range[0], cam_range[1]),
                              cam_range[2], cam_range[3],
                              color='r', edgecolor='None', alpha=0.5,
                              label='CAM range')
    plt.gca().add_patch(cam_rect)
    srr_rect = plt.Rectangle((srr_range[0], srr_range[1]),
                              srr_range[2], srr_range[3],
                              color='b', edgecolor='None', alpha=0.5,
                              label='SRR range')
    plt.gca().add_patch(srr_rect)

    # measurements
    plt.plot(srr[3::5], srr[4::5], 'bx')
    plt.plot(cam[3::5], cam[4::5], 'rx')
    plt.plot(lrr[3::5], lrr[4::5], 'yx')

    # ground truth
    plt.plot(pedestrian_x, pedestrian_y, '-bo', label='pedestrian')
    plt.plot(car_x, car_y, '-ro', label='car')
    plt.plot(tree_x, tree_y, 'go', label='tree')

    legend1 = plt.legend(['pedestrian', 'car', 'tree'], loc=2)
    plt.gca().add_artist(legend1)
    plt.legend(handles=[srr_rect, cam_rect, lrr_rect], loc=1)

    plt.show()
