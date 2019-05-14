import os
import cv2
import numpy as np
import rospy
import tensorflow as tf
from styx_msgs.msg import TrafficLight
import time
import json
from datetime import datetime

import os
dir = os.path.dirname(__file__)

class TLClassifier(object):

    def now(self):
        return str(datetime.now().strftime('%I:%M:%S.%f'))

    def log(self, msg):
        filename = os.path.join(dir, '../../../../master.log')
        f = open(filename, 'a+')
        f.write('{} [tl_classifier]: {}\n'.format(self.now(), msg))
        f.close()

    #def __init__(self):
    def __init__(self, model_file): 
        # TODO load classifier
        self.current_light = TrafficLight.UNKNOWN

        cwd = os.path.dirname(os.path.realpath(__file__))

        model_path = os.path.join(cwd, "train_model/{}".format(model_file))
        rospy.logwarn("model_path={}".format(model_path))

        # load frozen tensorflow model
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.category_index = {1: {'id': 1, 'name': 'Green'}, 2: {'id': 2, 'name': 'Red'},
                               3: {'id': 3, 'name': 'Yellow'}, 4: {'id': 4, 'name': 'off'}}

        # create tensorflow session for detection
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # end
        self.sess = tf.Session(graph=self.detection_graph, config=config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    """
    Sample response (some fields are added later):
    {
        "lights": {
            "green": {"count": 2, "sum": 1.98, "average": 0.99202722311019897},
            "red": {"count": 0, "sum": 0.0, "average": 0.0},
            "final": {"color": "GREEN", "average": 0.99, "state": 2}
        },
        "boxes": [
            {"xmin": 312, "score": 0.99, "ymin": 122, "ymax": 287, "xmax": 393},
            {"xmin": 652, "score": 0.99, "ymin": 140, "ymax": 295, "xmax": 731}
        ],
        "filename": "/home/james/github/udacity/jmsktm/T2-CarND-Capstone/images/img-01-49-57-974795.jpg",
        "waypoints": {"current": 747, "traffic_light": 753},
        "time": {"dashed": "01-49-57-974795", "colon": "01:49:57.974795"}
    }
    """
    def get_classification(self, image):
        current_time = datetime.now()
        time_colon = str(current_time.strftime('%I:%M:%S.%f'))
        time_dashed = str(current_time.strftime('%I-%M-%S-%f'))
        result = { "time": { "colon": time_colon, "dashed": time_dashed } }

        filename = os.path.join(dir, '../../../../images/img-{}.jpg'.format(result["time"]["dashed"]))
        result["filename"] = filename
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # return TrafficLight.RED
        # TODO implement light color prediction
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        (im_width, im_height, _) = image_rgb.shape
        image_np = np.expand_dims(image_rgb, axis=0)

        # Actual detection.
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores,
                 self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        min_score_thresh = .70
        
        red_count = 0
        red_sum = 0.0
        red_average = 0.0

        green_count = 0
        green_sum = 0.0
        green_average = 0.0

        total_count = 0
        average = 1.0
        count = 0

        height, width, channels = image.shape

        arr = []
        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                total_count += 1
                class_name = self.category_index[classes[i]]['name']

                # Traffic light thing
                if class_name == 'Red' or class_name == 'Yellow':
                    red_count += 1
                    red_sum += scores[i]
                elif class_name == 'Green':
                    green_count += 1
                    green_sum += scores[i]

                box = boxes[i]
                ymin, xmin, ymax, xmax = box
                
                xmin1 = int(xmin * width)
                ymin1 = int(ymin * height)
                xmax1 = int(xmax * width)
                ymax1 = int(ymax * height)
                score = round(scores[i], 2)
                arr.append({ "xmin": xmin1, "ymin": ymin1, "xmax": xmax1, "ymax": ymax1, "score": score })

        result["boxes"] = arr

        if red_count > 0:
            red_average = red_sum / red_count

        if green_count > 0:
            green_average = green_sum / green_count

        light_color = 'UNKNOWN'
        self.current_light = TrafficLight.UNKNOWN
        if red_count > 0 and red_average > min_score_thresh and red_average > green_average:
            light_color = 'RED'
            red_sum = round(red_sum, 2)
            average = round(red_average, 2)
            self.current_light = TrafficLight.RED
        elif green_count > 0 and green_average > min_score_thresh and green_average > red_average:
            light_color = 'GREEN'
            green_sum = round(green_sum, 2)
            average = round(green_average, 2)
            self.current_light = TrafficLight.GREEN

        result["lights"] = {
            "red": { "count": red_count, "sum": red_sum, "average": red_average },
            "green": { "count": green_count, "sum": green_sum, "average": green_average },
            "final": { "color": light_color, "average": average, "state": self.current_light }
        }

        return result
