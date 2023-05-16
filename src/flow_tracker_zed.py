#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import cv2
import os

# limita o numero de processadores usados por pacotes pesados
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import sys
import platform
import numpy as np
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov8 strongsort root directory
WEIGHTS = ROOT / 'weights'

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
if str(ROOT / 'yolov8') not in sys.path:
    sys.path.append(str(ROOT / 'yolov8'))  # add yolov8 ROOT to PATH
if str(ROOT / 'trackers' / 'strongsort') not in sys.path:
    sys.path.append(str(ROOT / 'trackers' / 'strongsort'))  # add strong_sort ROOT to PATH

ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  

import logging
from yolov8.ultralytics.nn.autobackend import AutoBackend
from yolov8.ultralytics.yolo.data.dataloaders.stream_loaders import LoadImages, LoadStreams
from yolov8.ultralytics.yolo.data.utils import IMG_FORMATS, VID_FORMATS
from yolov8.ultralytics.yolo.utils import DEFAULT_CFG, LOGGER, SETTINGS, callbacks, colorstr, ops
from yolov8.ultralytics.yolo.utils.checks import check_file, check_imgsz, check_imshow, print_args, check_requirements
from yolov8.ultralytics.yolo.utils.files import increment_path
from yolov8.ultralytics.yolo.utils.torch_utils import select_device
from yolov8.ultralytics.yolo.utils.ops import Profile, non_max_suppression, scale_boxes, process_mask, process_mask_native
from yolov8.ultralytics.yolo.utils.plotting import Annotator, colors, save_one_box

from trackers.multi_tracker_zoo import create_tracker

import rospy
from follow_me.msg import dc_to_follow
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


import rospy
import numpy as np
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
import torch
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
import time
import traceback


@torch.no_grad()

#def calculate_centroid(bbox):
#    x_min, y_min, x_max, y_max = bbox
#    centroid_x = (x_min + x_max) / 2
#    centroid_y = (y_min + y_max) / 2
#    return centroid_x, centroid_y

class Flow:
    def __init__(self):
        self._global_frame = 'zed2i_left_camera_frame'
        self.pub = rospy.Publisher('p_to_follow', dc_to_follow, queue_size=10)
        self.msg = dc_to_follow()
        self.time=rospy.Time.now()
        point_cloud_topic = '/zed_node/point_cloud/cloud_registered'

        self._tf_listener = tf.TransformListener()
        self._current_image = None
        self._current_pc = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Create detector
        self._bridge = CvBridge()

        # Publisher for frames with detected objects
        self._imagepub = rospy.Publisher('~objects_label', Image, queue_size=10)
        if point_cloud_topic is not None:
            rospy.Subscriber(point_cloud_topic, PointCloud2, self.pc_callback)
        else:
            rospy.loginfo('No point cloud information available. Objects will not be placed in the scene.')

        self._tfpub = tf.TransformBroadcaster()
        rospy.loginfo('Ready to detect!')

    def pc_callback(self, pc):
        """Point cloud callback"""
        # Store value on a private attribute
        self._current_pc = pc

    def calculate_centroid_y(self, bbox):
        x_min, y_min, x_max, y_max = bbox
        centroid_y = (y_min + y_max) / 2

        return centroid_y

    def calculate_centroid_x(self, bbox):
        x_min, y_min, x_max, y_max = bbox
        centroid_x = (x_min + x_max) / 2

        return centroid_x

    def run(
            self, 
            source='0',
            yolo_weights=WEIGHTS / 'yolov8m.pt',  # model.pt path(s),
            reid_weights=WEIGHTS / 'osnet_x0_25_msmt17.pt',  # model.pt path,
            tracking_method='strongsort',
            tracking_config=None,
            imgsz=(640, 640),  # inference size (height, width)
            conf_thres=0.25,  # confidence threshold
            iou_thres=0.45,  # NMS IOU threshold
            max_det=1000,  # maximum detections per image
            device='0',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
            show_vid=True,  # show results
            save_txt=False,  # save results to *.txt
            save_conf=False,  # save confidences in --save-txt labels
            save_crop=False,  # save cropped prediction boxes
            save_trajectories=False,  # save trajectories for each track
            save_vid=False,  # save confidences in --save-txt labels
            nosave=False,  # do not save images/videos
            classes=None,  # filter by class: --class 0, or --class 0 2 3
            agnostic_nms=False,  # class-agnostic NMS
            augment=False,  # augmented inference
            visualize=False,  # visualize features
            update=False,  # update all models
            project=ROOT / 'runs' / 'track',  # save results to project/name
            name='exp',  # save results to project/name
            exist_ok=False,  # existing project/name ok, do not increment
            line_thickness=3,  # bounding box thickness (pixels)
            hide_labels=False,  # hide labels
            hide_conf=False,  # hide confidences
            hide_class=False,  # hide IDs
            half=False,  # use FP16 half-precision inference
            dnn=False,  # use OpenCV DNN for ONNX inference
            vid_stride=1,  # video frame-rate stride
            retina_masks=False,
    ):
        direction='not_defined'
        val_correc=1.5 # valor de correcao, usar entre 0.5 e 2.5
        previous_centroid_y = 387.54083195327223 # valor inicial do centroide (apenas para o primeiro frame a passar do flow, depois e corrigido)
        previous_centroid_x = 387.54083195327223 # valor inicial do centroide (apenas para o primeiro frame a passar do flow, depois e corrigido)
        source = str(source)
        save_img = not nosave and not source.endswith('.txt')  # save inference images
        is_file = Path(source).suffix[1:] in (VID_FORMATS)
        webcam = source.isnumeric() or source.endswith('.txt')

        # Directories
        if not isinstance(yolo_weights, list):  # single yolo model
            exp_name = yolo_weights.stem
        elif type(yolo_weights) is list and len(yolo_weights) == 1:  # single models after --yolo_weights
            exp_name = Path(yolo_weights[0]).stem
        else:  # multiple models after --yolo_weights
            exp_name = 'ensemble'
        exp_name = name if name else exp_name + "_" + reid_weights.stem
        save_dir = increment_path(Path(project) / exp_name, exist_ok=exist_ok)  # increment run
        (save_dir / 'tracks' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # Load model
        device = select_device(device)
        is_seg = '-seg' in str(yolo_weights)
        model = AutoBackend(yolo_weights, device=device, dnn=dnn, fp16=half)
        stride, names, pt = model.stride, model.names, model.pt
        imgsz = check_imgsz(imgsz, stride=stride)  # check image size

        # Dataloader
        bs = 1
        if webcam:
            show_vid = check_imshow(warn=True)
            dataset = LoadStreams(
                source,
                imgsz=imgsz,
                stride=stride,
                auto=pt,
                transforms=getattr(model.model, 'transforms', None),
                vid_stride=vid_stride
            )
            bs = len(dataset)
        else:
            dataset = LoadImages(
                source,
                imgsz=imgsz,
                stride=stride,
                auto=pt,
                transforms=getattr(model.model, 'transforms', None),
                vid_stride=vid_stride
            )
        vid_path, vid_writer, txt_path = [None] * bs, [None] * bs, [None] * bs
        model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup

        # Create as many strong sort instances as there are video sources
        tracker_list = []
        for i in range(bs):
            tracker = create_tracker(tracking_method, tracking_config, reid_weights, device, half)
            tracker_list.append(tracker, )
            if hasattr(tracker_list[i], 'model'):
                if hasattr(tracker_list[i].model, 'warmup'):
                    tracker_list[i].model.warmup()
        outputs = [None] * bs

        # Run tracking
        model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
        seen, windows, dt = 0, [], (Profile(), Profile(), Profile(), Profile())
        curr_frames, prev_frames = [None] * bs, [None] * bs
        for frame_idx, batch in enumerate(dataset):
            path, im, im0s, vid_cap, s = batch
            visualize = increment_path(save_dir / Path(path[0]).stem, mkdir=True) if visualize else False
            with dt[0]:
                im = torch.from_numpy(im).to(device)
                im = im.half() if half else im.float()  # uint8 to fp16/32
                im /= 255.0  # 0 - 255 to 0.0 - 1.0
                if len(im.shape) == 3:
                    im = im[None]  # expand for batch dim

            # Inference
            with dt[1]:
                preds = model(im, augment=augment, visualize=visualize)

            # Apply NMS
            with dt[2]:
                if is_seg:
                    masks = []
                    p = non_max_suppression(preds[0], conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det, nm=32)
                    proto = preds[1][-1]
                else:
                    p = non_max_suppression(preds, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
                
            # Process detections
            for i, det in enumerate(p):  # detections per image
                seen += 1
                if webcam:  # bs >= 1
                    p, im0, _ = path[i], im0s[i].copy(), dataset.count
                    p = Path(p)  # to Path
                    s += f'{i}: '
                    txt_file_name = p.name
                    save_path = str(save_dir / p.name)  # im.jpg, vid.mp4, ...
                else:
                    p, im0, _ = path, im0s.copy(), getattr(dataset, 'frame', 0)
                    p = Path(p)  # to Path
                    # video file
                    if source.endswith(VID_FORMATS):
                        txt_file_name = p.stem
                        save_path = str(save_dir / p.name)  # im.jpg, vid.mp4, ...
                    # folder with imgs
                    else:
                        txt_file_name = p.parent.name  # get folder name containing current img
                        save_path = str(save_dir / p.parent.name)  # im.jpg, vid.mp4, ...
                curr_frames[i] = im0

                txt_path = str(save_dir / 'tracks' / txt_file_name)  # im.txt
                s += '%gx%g ' % im.shape[2:]  # print string
                imc = im0.copy() if save_crop else im0  # for save_crop

                annotator = Annotator(im0, line_width=line_thickness, example=str(names))
                
                if hasattr(tracker_list[i], 'tracker') and hasattr(tracker_list[i].tracker, 'camera_update'):
                    if prev_frames[i] is not None and curr_frames[i] is not None:  # camera motion compensation
                        tracker_list[i].tracker.camera_update(prev_frames[i], curr_frames[i])

                if det is not None and len(det):
                    if is_seg:
                        shape = im0.shape
                        # scale bbox first the crop masks
                        if retina_masks:
                            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], shape).round()  # rescale boxes to im0 size
                            masks.append(process_mask_native(proto[i], det[:, 6:], det[:, :4], im0.shape[:2]))  # HWC
                        else:
                            masks.append(process_mask(proto[i], det[:, 6:], det[:, :4], im.shape[2:], upsample=True))  # HWC
                            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], shape).round()  # rescale boxes to im0 size
                    else:
                        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # rescale boxes to im0 size

                    # Print results
                    for c in det[:, 5].unique():
                        n = (det[:, 5] == c).sum()  # detections per class
                        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    # pass detections to strongsort
                    with dt[3]:
                        outputs[i] = tracker_list[i].update(det.cpu(), im0)
                    
                    # draw boxes for visualization
                    if len(outputs[i]) > 0:
                        
                        if is_seg:
                            # Mask plotting
                            annotator.masks(
                                masks[i],
                                colors=[colors(x, True) for x in det[:, 5]],
                                im_gpu=torch.as_tensor(im0, dtype=torch.float16).to(device).permute(2, 0, 1).flip(0).contiguous() /
                                255 if retina_masks else im[i]
                            )
                        for j, (output) in enumerate(outputs[i]):
                            bbox_dict = {}
                            bbox = output[0:4]
                            id = output[4]

                            if id not in bbox_dict:
                                bbox_dict[id] = []

                            bbox_dict[id].append(bbox)

                            cls = output[5]
                            conf = output[6]

                            id_to_find = 1

                            if id_to_find in bbox_dict:
                                bbox_values = bbox_dict[id_to_find][-1]  # Get the last entry in the list for the ID
                                #pub.publish(bbox_values)
                                current_centroid_x = int(self.calculate_centroid_x(bbox_values))
                                current_centroid_y = int(self.calculate_centroid_y(bbox_values))
                                # centroid_x_to_tf=
                                # centroid_y_to_tf=
                                print(f"Centroid_x for ID {id_to_find}: {current_centroid_x}")

                            #if id_to_find in previous_centroid_x:
                                # Compare the current and previous centroid_x values
                                if current_centroid_x-previous_centroid_x <val_correc and  current_centroid_x-previous_centroid_x > -val_correc :
                                    print(f'ID {id_to_find} is not moving horizontally')
                                    direction=('is not moving horizontally')
                                elif current_centroid_x > previous_centroid_x:
                                    print(f'ID {id_to_find} is moving right')
                                    direction=('right')
                                elif current_centroid_x < previous_centroid_x:
                                    print(f'ID {id_to_find} is moving left')
                                    direction=('left')
                                else:
                                    print(f'ID {id_to_find} is not moving horizontally')
                                    direction=('unknow')

                                # Update the previous centroid_x value for the ID
                                previous_centroid_x = current_centroid_x
                                previous_centroid_y = current_centroid_y
                                
                                #calculate_tf(current_centroid_x, current_centroid_y)
                            else:
                                print(f'No bounding box found for ID {id_to_find}')

                            (trans, _) = self._tf_listener.lookupTransform('/' + self._global_frame, '/zed_camera_center', rospy.Time(0))

                            if self._current_pc is None:
                                rospy.loginfo('No point cloud')

                            pc_list = list(
                                pc2.read_points(self._current_pc,
                                                skip_nans=True,
                                                field_names=('x', 'y', 'z'),
                                                uvs=[(current_centroid_x, current_centroid_y)]))

                            if len(pc_list) > 0:
                                publish_tf = True
                                tf_id = 'person_follow'
                                point_z, point_x, point_y = pc_list[0]

                            if publish_tf:
                                # Object tf (x, y, z) must be passed as (z, -x, -y)
                                object_tf = [point_z, point_x, point_y]
                                frame = 'zed_camera_center'

                                # Translate the tf in regard to the fixed frame
                                if self._global_frame is not None:
                                    object_tf = np.array(trans) + object_tf
                                    frame = self._global_frame

                                if object_tf is not None:
                                    self._tfpub.sendTransform((object_tf),
                                                                tf.transformations.quaternion_from_euler(0, 0, 0),
                                                                self.time,
                                                                tf_id,
                                                                frame)
                                    self.time = self.time + rospy.Duration(1e-3)
                                    
                            
                            #msg.direction=
                            self.msg.direction=direction
                            self.pub.publish(self.msg)

                            if save_txt:
                                # to MOT format
                                bbox_left = output[0]
                                bbox_top = output[1]
                                bbox_w = output[2] - output[0]
                                bbox_h = output[3] - output[1]
                                # Write MOT compliant results to file
                                with open(txt_path + '.txt', 'a') as f:
                                    f.write(('%g ' * 10 + '\n') % (frame_idx + 1, id, bbox_left,  # MOT format
                                                                bbox_top, bbox_w, bbox_h, -1, -1, -1, i))

                            if save_vid or save_crop or show_vid:  # Add bbox/seg to image
                                c = int(cls)  # integer class
                                id = int(id)  # integer id
                                label = None if hide_labels else (f'{id} {names[c]}' if hide_conf else \
                                    (f'{id} {conf:.2f}' if hide_class else f'{id} {names[c]} {conf:.2f}'))
                                color = colors(c, True)
                                annotator.box_label(bbox, label, color=color)
                                
                                if save_trajectories and tracking_method == 'strongsort':
                                    q = output[7]
                                    tracker_list[i].trajectory(im0, q, color=color)
                                if save_crop:
                                    txt_file_name = txt_file_name if (isinstance(path, list) and len(path) > 1) else ''
                                    save_one_box(np.array(bbox, dtype=np.int16), imc, file=save_dir / 'crops' / txt_file_name / names[c] / f'{id}' / f'{p.stem}.jpg', BGR=True)
                                
                else:
                    pass
                    #tracker_list[i].tracker.pred_n_update_all_tracks()
                    
                # Stream results
                im0 = annotator.result()
                # Flow results
                direction_text = f'Direction: {direction}'
                self._imagepub.publish(self._bridge.cv2_to_imgmsg(im0, 'bgr8'))
                cv2.putText(im0, direction_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                if show_vid:
                    if platform.system() == 'Linux' and p not in windows:
                        windows.append(p)
                        cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                        cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                        
                    cv2.imshow(str(p), im0)
                    if cv2.waitKey(1) == ord('q'):  # 1 millisecond
                        exit()

                # Save results (image with detections)
                if save_vid:
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer[i].write(im0)

                prev_frames[i] = curr_frames[i]
                
            # Print total time (preprocessing + inference + NMS + tracking)
            LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{sum([dt.dt for dt in dt if hasattr(dt, 'dt')]) * 1E3:.1f}ms")

            # loop over the detections
            #for *xyxy, conf, cls in bbox.xyxy[0]:
            #    x1, y1, x2, y2 = map(int, xyxy)
            #    bbox = (x1, y1, x2 - x1, y2 - y1)  # bounding box coordinates (x, y, w, h)
            #    label = f"{names[int(cls)]} {conf:.2f}"  # class label and confidence score
            #    id = int(cls)  # ID of the detected object
            #    annotator.bbox(bbox, label=label, id=id)  # add the annotation to the annotator


        # Print results
        t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
        LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS, %.1fms {tracking_method} update per image at shape {(1, 3, *imgsz)}' % t)
        if save_txt or save_vid:
            s = f"\n{len(list((save_dir / 'tracks').glob('*.txt')))} tracks saved to {save_dir / 'tracks'}" if save_txt else ''
            LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
        if update:
            strip_optimizer(yolo_weights)  # update model (to fix SourceChangeWarning)

    def main(self):
        check_requirements(requirements=ROOT / 'requirements.txt', exclude=('tensorboard', 'thop'))
        self.run(
            source='4',
            yolo_weights=WEIGHTS / 'yolov8s-seg.pt',  # model.pt path(s),
            reid_weights=WEIGHTS / 'resnet50_msmt17.pt',  # model.pt path,
            tracking_method='strongsort',
            tracking_config='/home/robofei/Workspace/catkin_ws/src/3rd_party/Vision_System/track-flow/src/trackers/strongsort/configs/strongsort.yaml',
            imgsz=[1280, 720],  # inference size (height, width)
            conf_thres=0.25,  # confidence threshold
            iou_thres=0.45,  # NMS IOU threshold
            max_det=1000,  # maximum detections per image
            device='0',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
            show_vid=False,  # show results
            save_txt=False,  # save results to *.txt
            save_conf=False,  # save confidences in --save-txt labels
            save_crop=False,  # save cropped prediction boxes
            save_trajectories=False,  # save trajectories for each track
            save_vid=False,  # save confidences in --save-txt labels
            nosave=False,  # do not save images/videos
            classes=0,  # filter by class: --class 0, or --class 0 2 3
            agnostic_nms=False,  # class-agnostic NMS
            augment=False,  # augmented inference
            visualize=False,  # visualize features
            update=False,  # update all models
            project=ROOT / 'runs' / 'track',  # save results to project/name
            name='exp',  # save results to project/name
            exist_ok=False,  # existing project/name ok, do not increment
            line_thickness=3,  # bounding box thickness (pixels)
            hide_labels=False,  # hide labels
            hide_conf=False,  # hide confidences
            hide_class=False,  # hide IDs
            half=False,  # use FP16 half-precision inference
            dnn=False,  # use OpenCV DNN for ONNX inference
            vid_stride=1,  # video frame-rate stride
            retina_masks=False,
            
        )

if __name__ == "__main__":
    rospy.init_node('follow')
    
    try:
        Flow().main()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')
