# Overview of Perception Tasks
This markdown file (`perception.md`) is intended to supplement and act as a foundation for my section of the report. Here I will describe how I went about developing my tasks

## Perception 1
A new node `CameraProcessor` was created as a seperate ROS package to both learn more about ROS based development as well as to achieve the objective of collecting images to serve as training data.

An initial pass was done allowing the defualt random walk model to autonomously explore the cave. Through this, the `CameraProcessor` node listened for images every 3 seconds and saved them to a local folder.

This path taken from this initial pass is seen below:

![capture_path_1.png](images/capture_path_1.png)

In total `268` images were collected in this first pass. Currently we have only gathered images from the RGB camera and *not* the depth camera.

## Perception 2
- Images were manually labelled using LabelStudio
- Labeled images were exported in yolo campatible format 
- Executed the yolo training process on a Google Colab runtime environment
- Saved the trained model, weights and some results

### Labelling
Annotation/labelling of data was done manually on the `268` images collected for the first pass. Example of annotations done can be seen below:

![labelling_process_2.png](images/labelling_process_2.png)

![labelling_process_3.png](images/labelling_process_3.png)

Annotated images were then imported into a Google Colab environment and split into training and validation folders.

### Model Training
The `ultralytics` package was installed to the environment's python runtime and training was executed using the command:
```sh
!yolo detect train data=/content/data.yaml model=yolo11s.pt epochs=60 imgsz=640
```

The model was then executed on the validatio nset using the commmand:
```sh
!yolo detect predict model=runs/detect/train/weights/best.pt source=data/validation/images save=True
```

Results from the model trained on the first pass images can be seen below:

![pass_1_model.png](ros_workspace/src/model_runner/models/model_1/train/train_batch802.jpg)

The relevant weights and pytorch model was then downloaded from the Collab environment and stored in this repository as `model_1`.

### Initial Deployment
Initial deployment of YOLO model to environmemt:

[![progress_1-perception_task_2](https://img.youtube.com/vi/Pje4ALhp0z0/0.jpg)](https://youtu.be/Pje4ALhp0z0)

### Text
I added some text next to the bounding boxes:

[![progress_1.2-perception_task_2](https://img.youtube.com/vi/TXohq1eB3Mk/0.jpg)](https://www.youtube.com/watch?v=TXohq1eB3Mk)

### Refinements
Then the confidence thresholds that categorised a detection was tweaked from the default 0.25 confidence to 0.5 confidence.

[![progress_1.3-perception_task_2](https://img.youtube.com/vi/pDmgowRG3t0/0.jpg)](https://www.youtube.com/watch?v=pDmgowRG3t0)


## Perception 3 
My initial approach to this task was to use camera intrinsic and extrinsic values in conjunction with the linear algebra relationship between 2D points on an image, camera matrix and 3D points in the "real world" to do a transform that would allow me to estimate an object's real world coordinates based on its coordinates in an image.

However I have switched my approach to instead focus firstly on allowing close-inspection of an artefact with hte intent of doing a proper localisation once the rover is close to the artefact.

I have struggle ddefining what close means so for the purposes of development I am going to do the following:
- Center the detected artefact in the camera frame within an arbitrary -50 and +50 pixel range
- Create a vector that points in the direction that the robot is facing once the above condition is met
- Have the close-inspection path planner kick in to move the robot within an arbitrary distance towards the artefact
- Create a marker a few meters in the direction of the artefact

This will then serve as the initial estimate. I then hope to use the depth camera or lidar scanner to refine this estimate, but I am yet to look at the sensor data being received from the depth camera and lidar scanner so this remains an extension goal for Perception.