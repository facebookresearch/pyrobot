# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import torch

from test import Tester

model_path = "models/1_2_CWneg_0_8_EPS_100/checkpoint.pth.bst"
video_path = "/home/senthil/projects/caffe2pytorch/video_2.mp4"
save_path = "video_net_2_bst_epochs.avi"
frame_skip = 2

M = torch.load(model_path)
E = Tester(M)
cap = cv2.VideoCapture(video_path)
out = cv2.VideoWriter(
    save_path, cv2.VideoWriter_fourcc("M", "J", "P", "G"), 10, (1280, 720)
)

while cap.isOpened():
    for _ in range(frame_skip):
        ret, frame = cap.read()
    ret, frame = cap.read()
    if ret == True:
        outs = E.test(frame)
        print(outs)
        frame_outs = outs * 720
        print(frame_outs)
        cv2.line(frame, (320, 0), (320, int(frame_outs[1])), (0, 255, 0), 10)
        cv2.line(frame, (640, 0), (640, int(frame_outs[0])), (0, 255, 0), 10)
        cv2.line(frame, (960, 0), (960, int(frame_outs[2])), (0, 255, 0), 10)
        out.write(frame)
    else:
        break

cap.release()
out.release()
