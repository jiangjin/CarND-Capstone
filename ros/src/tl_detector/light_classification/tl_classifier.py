from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import time
import os
from functools import partial

MODEL_FILE = './frozen_inference_graph.pb'
MODEL_CHUNKS_PATH = './model_chunks'
MODEL_CHUNK_SIZE = 1024


class TLClassifier(object):
    def __init__(self):
        print("INIT TL CLASSIFIER: {}".format(os.getcwd()))

        self.detection_graph = None
        self.session = None
        self.image_tensor = None
        self.score_tensor = None
        self.class_tensor = None

        # was model existing?
        if not os.path.exists(MODEL_FILE):
            # if not - build it from the chunks
            if os.path.exists(MODEL_CHUNKS_PATH):
                output = open(MODEL_FILE, 'wb')
                chunks = os.listdir(MODEL_CHUNKS_PATH)
                chunks.sort()
                for filename in chunks:
                    filepath = os.path.join(MODEL_CHUNKS_PATH, filename)
                    with open(filepath, 'rb') as fileobj:
                        for chunk in iter(partial(fileobj.read, MODEL_CHUNK_SIZE), ''):
                            output.write(chunk)
                output.close()

        self.create_detection_graph(MODEL_FILE)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
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

    def infer(self, image):
        image_np_expanded = np.expand_dims(image, axis=0)
        st = time.time()

        (scores, classes) = self.session.run(
            [self.score_tensor, self.class_tensor],
            feed_dict={self.image_tensor: image_np_expanded})

        class_scores = [0, 0, 0, 0]
        for j in range(len(scores)):
            for i in range(len(scores[j])):
                score = scores[j][i]
                if score > 0.5:
                    index = int(classes[j][i])
                    class_scores[index] += score

        strongest_class = class_scores.index(max(class_scores))

        dt = "{:.2f}".format(time.time() - st)
        if class_scores[strongest_class] > 0.75:
            print("{} (score: {:.2f}, t: {})".format(TLClassifier.get_light_name(strongest_class), class_scores[strongest_class], dt))
            return strongest_class
        else:
            print("")
            return 0

    @staticmethod
    def get_light_name(id):
        light_names = ["NONE", "RED", "YELLOW", "GREEN"]
        return light_names[id]

    @staticmethod
    def convert_tl_id(index):
        if 1 <= index <= 3:
            return index - 1
        else:
            return TrafficLight.UNKNOWN
