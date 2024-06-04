from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.pt")  # load a tiger-pose trained model

# Run inference
results = model.predict(source="https://youtu.be/MIBAT6BGE6U", show=True)


# https://github.com/Deci-AI/super-gradients/blob/master/src/super_gradients/recipes/dataset_params/coco_pose_estimation_common_dataset_params.yaml
# https://sites.google.com/view/animal-pose/
# https://github.com/noahcao/animal-pose-dataset?tab=readme-ov-file
# https://drive.google.com/drive/folders/1-yOSGWts2ZDYFx29u7vPcX4CdGJkPx1w
# https://drive.google.com/drive/folders/1xxm6ZjfsDSmv6C9JvbgiGrmHktrUjV5x
# https://learnopencv.com/animal-pose-estimation/
# https://www.youtube.com/watch?v=J83ZvWfxjoA
# https://docs.ultralytics.com/tasks/pose/
# https://universe.roboflow.com/search?q=cow+pose
# https://github.com/DeepLabCut/DeepLabCut/blob/main/docs/UseOverviewGuide.md
# https://github.com/DeepLabCut/DeepLabCut/blob/main/examples/README.md
# https://github.com/DeepLabCut/DeepLabCut-Workshop-Materials/blob/main/DLCcourse.md
# https://github.com/DeepLabCut/DeepLabCut
# https://viso.ai/deep-learning/pose-estimation-ultimate-overview/