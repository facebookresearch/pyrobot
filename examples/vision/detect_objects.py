# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import sys

import numpy
import models
import torch

from absl import flags, app
from torch.autograd import Variable
from utils.datasets import pad_to_square, resize
from utils.utils import non_max_suppression, load_classes
from pyrobot import Robot

# FIXME: Should not hard-code this locatoin
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
    import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

FLAGS = flags.FLAGS
flags.DEFINE_string('botname', 'locobot',
                    'Robot name, locobot, locobot_lite, ...')


class Detector(object):
    def __init__(self):
        yolov3_path = os.path.dirname(models.__file__)

        model = models.Darknet(yolov3_path + "/config/yolov3.cfg").to("cpu")
        model.load_darknet_weights(yolov3_path + "/weights/yolov3.weights")
        model.eval()

        self.object_classes = load_classes(yolov3_path + "/data/coco.names")

        self.model = model

    def detect(self, image):
        with torch.no_grad():
            image = Variable(torch.from_numpy(image).type(torch.FloatTensor))
            image = image.permute(2, 0, 1)
            image = image / 255.0

            image, _ = pad_to_square(image, 0)
            image = resize(image, 416)
            image = image.unsqueeze(0)
            detections = self.model(image)
            detections = non_max_suppression(detections, 0.8, 0.0)
            if detections[0] is not None:
                for d in detections[0]:
                    print("Class %s" % self.object_classes[int(d[6])])
            else:
                print("No objects detected")
        return detections,\
            (image.squeeze().permute(1, 2, 0).numpy()*255).astype(numpy.uint8)


def main(_):
    bot = Robot(FLAGS.botname)
    camera = bot.camera

    detector = Detector()

    for i in range(0, 10):
        image = camera.get_rgb()

        objects, image = detector.detect(image)
        image = image[:, :, ::-1]  # RGB -> BGR
        if objects[0] is not None:
            for detected in objects[0]:
                detect = detected.numpy().astype(int)
                print((detect[0], detect[1]))
                image = cv2.rectangle(image, (detect[0], detect[1]),
                                      (detect[2], detect[3]), (0, 255, 0), 2)
        cv2.imshow('Color', image)
        cv2.waitKey(1000)

if __name__ == "__main__":
    app.run(main)
