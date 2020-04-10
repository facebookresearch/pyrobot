# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import torch
from IPython import embed
from PIL import Image
from torchvision import transforms

PYTORCH_TRANSFORM = transforms.Compose(
    [
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ]
)

ASPECT_RATIO = 290.0 / 640.0


class Tester(object):
    """
    """

    def __init__(self, model):
        self.model = model

    def test(self, I):
        PIL_Is = self.convert_to_PIL(I)
        outputs = self.model.forward(PIL_Is)
        np_outs = outputs[:, 1].cpu().data.numpy()
        return np_outs

    def convert_to_PIL(self, image):
        h, w, _ = image.shape
        des_h = int(w * ASPECT_RATIO)
        diff = h - des_h
        half_diff = int(diff / 2.0)
        image = image[half_diff : -half_diff - 1]
        print("New Shape:{}".format(image.shape))
        image = cv2.resize(image, (640, 290))
        self.image = image
        I = Image.fromarray(image.astype("uint8"), "RGB")
        I_center = I.crop((160, 72, 480, 217))
        I_left = I.crop((0, 72, 320, 217))
        I_right = I.crop((320, 72, 640, 217))
        I_list = torch.cat(
            map(
                torch.unsqueeze,
                map(PYTORCH_TRANSFORM, [I_center, I_left, I_right, I]),
                [0] * 4,
            )
        )
        return I_list


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_path")
    parser.add_argument("--image_path")
    args = parser.parse_args()
    M = torch.load(args.model_path)
    I = Image.open(args.image_path)
    I_left = I.crop((0, 72, 320, 217))
    I_right = I.crop((320, 72, 640, 217))
    I_list = torch.cat(
        map(torch.unsqueeze, map(PYTORCH_TRANSFORM, [I, I_left, I_right]), [0] * 3)
    )
    outputs = M.forward(I_list)
    np_outs = outputs[:, 1].cpu().data.numpy()
    embed()


if __name__ == "__main__":
    main()
