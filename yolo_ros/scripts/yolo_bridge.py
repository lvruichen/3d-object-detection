#!/home/eric/miniconda3/envs/torch/bin/python
from matplotlib import image
import rospy
import torch
import torch.backends.cudnn as cudnn
import numpy as np
import time
import cv2
from models.experimental import attempt_load
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, plot_one_box, strip_optimizer, set_logging)
from utils.torch_utils import select_device, load_classifier, time_synchronized
from sensor_msgs.msg import Image
from std_msgs.msg import Header

ros_image = 0

class ROS2YOLO:
    def __init__(self, node_name='yolo_detection', name='yolo'):
        self.yolo5_path = rospy.get_param('yolov5/path', None)
        self.model = rospy.get_param('yolov5/model', 'yolov5s')
        self.weight = rospy.get_param('yolov5/weight', None)
        self.device = rospy.get_param('yolov5/device', 'gpu')
        self.img_size = rospy.get_param('yolov5/img_size', 640)
        self.image_topic = rospy.get_param('yolov5/image_topic', None)
        self.valid = False if (self.yolo5_path is None) and (self.weight is None) else True
        self.stride = 32
        self.half = True if self.device == 'gpu' else False
        #for ROS
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher('yolo_result', Image, queue_size=1)
        
        self.frame_id = 'camera_link'
        self.image_height = 480
        self.image_width = 640

        if not self.load_model():
            print('error occurred while loading !')

    def load_model(self):
        if not self.valid:
            return  False
        self.device = torch.device('cuda:0' if (self.device != 'cpu' and torch.cuda.is_available()) else 'cpu')
        if self.yolo5_path is None:
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path_or_model=self.weight)
            self.model = self.model.cuda() if self.device != 'cpu' else self.model
        else:
            self.model = attempt_load(self.weight, self.device)
        self.stride = int(self.model.stride.max())
        self.img_size = check_img_size(self.img_size, s=self.stride)
        self.half = self.device.type != 'cpu'
        if self.half:
            self.model.half()
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        print('labels: '), self.print_list_multiline(self.names)
        return True
    
    @staticmethod
    def print_list_multiline(names, num_line=5):
        length = len(names)
        print('[', end="")
        for i in range(0, length):
            print(str(i) + ':' + '\'', end=''), print(names[i], end=''), print('\'', end='')
            if i == length - 1:
                print(']')
            elif i % num_line == num_line - 1:
                print(',\n ', end='')
            else:
                print(',', end=' ')

    def image_callback(self, image):
        global ros_image
        self.image_height = image.height
        self.image_width = image.width 
        ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        
        self.detect(ros_image)
    
    def detect(self, img):
        time1 = time.time()
        global ros_image
        cudnn.benchmark = True
        dataset = self.loadimg(img)
        conf_thres = 0.3
        iou_thres = 0.45
        classes = (0,1,2,3,5,7, 67)
        agnostic_nms = 'store_true'
        img = torch.zeros((1, 3, self.img_size, self.img_size), device=self.device)  # init img
        path = dataset[0]
        img = dataset[1]
        im0s = dataset[2]
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        time2 = time.time()
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
        with torch.no_grad():
        # Inference
            pred = self.model(img)[0]
        # Apply NMS
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)

        view_img = 1
        time3 = time.time()

        for i, det in enumerate(pred):  # detections per image
            p, s, im0 = path, '', im0s
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if det is not None:
                #print(det)
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, self.names[int(c)])  # add to string
                    # Write results
                for *xyxy, conf, cls in reversed(det):
                    if view_img:  # Add bbox to image
                        label = '%s %.2f' % (self.names[int(cls)], conf)
                        plot_one_box(xyxy, im0, label=label, color=[0,255,0], line_thickness=3)
        time4 = time.time()
        print('detect time %.2f'%((time3-time1) * 1000), 'ms')
        out_img = im0[:, :, [2, 1, 0]]
        ros_image=out_img
        cv2.imshow('YOLOV5', out_img)
        a = cv2.waitKey(1)
        #### Create CompressedIamge ####
        self.publish_image(im0)
        
    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
        # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
        shape = img.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)  

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better test mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, 32), np.mod(dh, 32)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return img, ratio, (dw, dh)

    def loadimg(self, img):  # 接受opencv图片
        cap=None
        path=None
        img0 = img
        img = self.letterbox(img0, new_shape=self.img_size)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        return path, img, img0, cap
    def publish_image(self, img):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.frame_id
        image_temp.height = self.image_height
        image_temp.width = self.image_width
        image_temp.encoding = 'rgb8'
        image_temp.data = np.array(img).tostring()
        image_temp.header=header
        self.pub.publish(image_temp)

