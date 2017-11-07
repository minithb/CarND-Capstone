from styx_msgs.msg import TrafficLight
import pickle 
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix
import numpy as np
import csv
import math
import tensorflow as tf
from tensorflow.contrib.layers import flatten

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        #self.model_path = model_path
        #self.labels = [TrafficLight.NaN, TrafficLight.R, TrafficLight.Y, TrafficLight.G, TrafficLight.NaN]
        #PATH_TO_CKPT = self.model_path + '/checkpoints/frozen_inference_graph.pb'
        self.graph = tf.Graph()
        with self.graph.as_default():
            gpu_options = tf.GPUOptions(allow_growth=True)
            config = tf.ConfigProto(log_device_placement=False, gpu_options=gpu_options)
            self.sess = tf.Session(config=config)
                #sess.run(tf.initialize_all_variables())
            server = tf.train.import_meta_graph('lenet.meta')
            server.restore(sess, tf.train.latest_checkpoint('./'))
            self.input_image = tf.get_default_graph().get_tensor_by_name("input_image:0")
            self.model_output = tf.get_default_graph().get_tensor_by_name("model_output:0")
        #pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #return TrafficLight.UNKNOWN
        with self.graph.as_default():
            classes = self.sess.run(self.model_output, {
                self.input_image: [image]
            })[0]
            return np.argmax(classes)

