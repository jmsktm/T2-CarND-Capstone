import os
import cv2
import numpy as np
import rospy
import tensorflow as tf
from styx_msgs.msg import TrafficLight
import time
from datetime import datetime

class TLClassifier(object):

    def now(self):
        return str(datetime.now().strftime('%I:%M:%S.%f'))

    def now_dashed(self):
        return str(datetime.now().strftime('%I-%M-%S-%f'))

    def log(self, msg):
        f = open("/home/james/github/udacity/jmsktm/T2-CarND-Capstone/master.log","w+")
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

    def get_classification(self, image):
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

        min_score_thresh = .85
        count = 0
        count1 = 0

        height, width, channels = image.shape

        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                count1 += 1
                class_name = self.category_index[classes[i]]['name']

                # Traffic light thing
                if class_name == 'Red':
                    count += 1

                box = boxes[i]
                ymin, xmin, ymax, xmax = box
                
                xmin1 = int(xmin * width)
                ymin1 = int(ymin * height)
                xmax1 = int(xmax * width)
                ymax1 = int(ymax * height)
                cv2.rectangle(image,(xmin1, ymin1),(xmax1, ymax1), (0,0,255), 2)

                confidence = '{}%'.format(round(scores[i],2))
                cv2.putText(image, confidence, (xmin1+10, ymin1+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

        # print(count)
        light_color = 'UNKNOWN'
        if count < count1 - count:
            light_color = 'GREEN'
            self.current_light = TrafficLight.GREEN
        else:
            light_color = 'RED'
            self.current_light = TrafficLight.RED

        text = '{} / {}'.format(self.now(), light_color)
        filename = '/home/james/github/udacity/jmsktm/T2-CarND-Capstone/images/img-{}.jpg'.format(self.now_dashed())
        cv2.putText(image, text, (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
        
        cv2.imwrite(filename, image)

        return self.current_light
