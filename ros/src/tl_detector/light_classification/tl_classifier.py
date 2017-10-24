from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import time
import os


class TLClassifier(object):
    def __init__(self):
        print("INIT TL CLASSIFIER: {}".format(os.getcwd()))

        self.detection_graph = None
        self.session = None
        self.image_tensor = None
        self.score_tensor = None
        self.class_tensor = None
        self.graph_created = False

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.graph_created:
            # TODO: Check how to safely include this model file when buliding catkin package.
            self.create_detection_graph("./frozen_inference_graph.pb")
            return

        traffic_light_index = self.infer(image)
        return TLClassifier.convert_tl_id(traffic_light_index)

    def create_detection_graph(self, model_path):
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.session = tf.Session(graph=self.detection_graph)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.score_tensor = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.class_tensor = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.graph_created = True

    def infer(self, image):
        image_np_expanded = np.expand_dims(image, axis=0)
        st = time.time()

        (scores, classes) = self.session.run(
            [self.score_tensor, self.class_tensor],
            feed_dict={self.image_tensor: image_np_expanded})

        class_scores = [0, 0, 0, 0, 0]
        for i in range(len(scores[0])):
            score = scores[0][i]
            if score > 0.7:
                index = int(classes[0][i])
                class_scores[index] += score
        strongest_class = class_scores.index(max(class_scores))

        dt = "{:.2f}".format(time.time() - st)

        if class_scores[strongest_class] > 1:
            print("{} ({})".format(TLClassifier.get_light_name(strongest_class), dt))
            return strongest_class
        else:
            print("")
            return 0

    @staticmethod
    def get_light_name(id):
        light_names = ["NONE", "RED", "YELLOW", "GREEN", "UNKNOWN"]
        return light_names[id]

    @staticmethod
    def convert_tl_id(index):
        if 1 <= index <= 3:
            return index - 1
        else:
            return TrafficLight.UNKNOWN
