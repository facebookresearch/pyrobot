## Vision Examples

This folder contains examples that show how to incorporate vision algorithms into pyrobot.

## detect_object.py

This example shows how to detect objects from the camera using YoloV3.  Running the example will require you to install
PyTorch-YoloV3

* Running the example
  * Clone the PyTorch-YOLOV3 repository: https://github.com/eriklindernoren/PyTorch-YOLOv3
    * The example was tested with commit `47b7c912877ca69db35b8af3a38d6522681b3bb3`
  * Follow the PyTorch-YOLOV3 install instructions to install requirements and download pre-trained weights.
    * It is not necessary to download the COCO dataset.
  * Add the PyTorch-YOLOV3 directory to your `PYTHONPATH` environment variable
    * `export PYTHONPATH=$PYTHONPATH;/path/to/PyTorch-YOLOV3/`
  * `load_pyrobot_env`
  * `roslaunch locobot_control main.launch use_base:=false use_camera:=true use_arm:=false`
  * `python detect_objects.py`

Once the example runs, you will see a live image from the camera with bounding boxes displaying object detections.
Object classes are printed to the console.