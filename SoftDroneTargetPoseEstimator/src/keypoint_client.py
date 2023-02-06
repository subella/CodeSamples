import cv2
import zmq
import base64
import numpy as np
from torch2trt import TRTModule
from PIL import Image, ImageDraw
import torch
import torch.utils.data
import torch.nn
import os
import PIL.Image
import json
import tqdm
import trt_pose
import trt_pose.plugins
import glob
import torchvision.transforms.functional as FT
import numpy as np
from trt_pose.parse_objects import ParseObjects
import pycocotools
import pycocotools.coco
import pycocotools.cocoeval
import torchvision
from trt_pose.coco import *
import time

NUM_KEYPOINTS = 7
IMAGE_AREA = 234000

def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)

def detect_keypoints(cv2_image, model):
    orig_image = Image.fromarray(cv2_image).convert("RGB")
    orig_image = orig_image.crop((0, 
                                  0.5 * orig_image.height,
                                  orig_image.width, 
                                  orig_image.height))
    resize_factor = np.sqrt((orig_image.width * orig_image.height) / IMAGE_AREA)
    image_shape = (int(orig_image.width / resize_factor), int(orig_image.height / resize_factor))
    #ar = float(image.width) / float(image.height)
    ar = 1.0
    quad = get_quad(0.0, (0, 0), 1.0, aspect_ratio=ar)
    image = transform_image(orig_image, image_shape, quad)
    #image.save("img.png")
    data = transform(image).cuda()[None, ...]
    cmap = model(data)
    cmap = cmap.cpu()
    object_counts, objects, peaks = parse_objects(cmap, cmap)
    object_counts, objects, peaks = int(object_counts[0]), objects[0], peaks[0]

    kps = [0]*(NUM_KEYPOINTS*3)
    for i in range(object_counts):
        object = objects[i]
        cnt = 0
        for j in range(NUM_KEYPOINTS):
            k = object[j]
            if k >= 0:
                peak = peaks[j][k]
                if ar > 1.0: # w > h w/h
                    x = peak[1]
                    y = (peak[0] - 0.5) * ar + 0.5
                else:
                    x = (peak[1] - 0.5) / ar + 0.5
                    y = peak[0]

                x = round(float(orig_image.width * x))
                y = round(float(orig_image.height * y + orig_image.height))
                kps[j * 3 + 0] = x
                kps[j * 3 + 1] = y
                kps[j * 3 + 2] = 2
    return np.array(kps).reshape((NUM_KEYPOINTS,3))

if __name__=="__main__":
    OPTIMIZED_MODEL = '/home/sparklab/KeypointDetectorModels/pepsi_bottle.pth'
    model = TRTModule()
    model.load_state_dict(torch.load(OPTIMIZED_MODEL))
    print("Model loaded!")

    with open('target_pose.json', 'r') as f:
        target_pose = json.load(f)

    topology = trt_pose.coco.coco_category_to_topology(target_pose)
    transform = torchvision.transforms.Compose([
                torchvision.transforms.ToTensor(),
                torchvision.transforms.Normalize([0.42174017, 0.39773076, 0.40150955], [0.25029679, 0.23943031, 0.25025475])
        ])
    parse_objects = ParseObjects(topology, cmap_threshold=0.1, link_threshold=0.1, cmap_window=5, line_integral_samples=7, max_num_parts=100, max_num_objects=100)
    
    context = zmq.Context()
    footage_socket = context.socket(zmq.REP)
    footage_socket.setsockopt(zmq.CONFLATE, 1)
    footage_socket.bind('tcp://*:5555')
    #footage_socket.setsockopt_string(zmq.SUBSCRIBE, str(''))
   
    while True:
        try:
            start_time = time.time()
            msg = footage_socket.recv()
            buf = memoryview(msg)
            img_flat = np.frombuffer(buf, np.uint8)
            img = img_flat.reshape((720, 1280, 3))
            kps = detect_keypoints(img, model)
            #kps = np.zeros((33,3), dtype=int)
            send_array(footage_socket, kps)
            print("Total time: ")
            print(time.time() - start_time)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            break

