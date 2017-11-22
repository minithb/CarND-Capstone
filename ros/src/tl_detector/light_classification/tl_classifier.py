<<<<<<< HEAD
 
from styx_msgs.msg import TrafficLight
import pickle
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix
=======
>>>>>>> a3d08ff39c3745923f23212b622390128549f0ff
import numpy as np
import tensorflow as tf
import cv2
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        #self.model_path = model_path
<<<<<<< HEAD
        self.labels = [TrafficLight.NaN, TrafficLight.R, TrafficLight.Y, TrafficLight.G, TrafficLight.NaN]
        #PATH_TO_CKPT = self.model_path + 'cnn/checkpoints/frozen_inference_graph.pb'
        self.graph = tf.Graph()
        with self.graph.as_default():
            gpu_options = tf.GPUOptions(allow_growth=True)
            config = tf.ConfigProto(log_device_placement=False, gpu_options=gpu_options)
            self.sess = tf.Session(config=config)
=======
        #self.labels = [TrafficLight.NaN, TrafficLight.R, TrafficLight.Y, TrafficLight.G, TrafficLight.NaN]
        #PATH_TO_CKPT = self.model_path + '/checkpoints/frozen_inference_graph.pb'
        self.tf_session = None

        #self.graph = tf.Graph()
        #with self.graph.as_default():
        #    gpu_options = tf.GPUOptions(allow_growth=True)
        #    config = tf.ConfigProto(log_device_placement=False, gpu_options=gpu_options)
        #    self.sess = tf.Session(config=config)
>>>>>>> a3d08ff39c3745923f23212b622390128549f0ff
                #sess.run(tf.initialize_all_variables())
            server = tf.train.import_meta_graph('cnn/checkpoint/cnn.ckpt.meta')
            server.restore(sess, tf.train.latest_checkpoint('cnn/checkpoint/cnn.ckpt'))
            self.input_image = tf.get_default_graph().get_tensor_by_name("input_image:0")
            self.model_output = tf.get_default_graph().get_tensor_by_name("model_output:0")
        #pass
<<<<<<< HEAD
=======

    def get_features(self, img):
        #img = (img*255).astype(np.uint8) # For png images
        img = cv2.resize(img,(224,224))
        channel1_hist = np.histogram(img[:,:,0], bins=256, range=(0,256))
        channel2_hist = np.histogram(img[:,:,1], bins=256, range=(0,256))
        channel3_hist = np.histogram(img[:,:,2], bins=256, range=(0,256))
        hist_features = [np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0])).reshape(-1,1)]
        return hist_features
        
>>>>>>> a3d08ff39c3745923f23212b622390128549f0ff
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        #return TrafficLight.UNKNOWN
<<<<<<< HEAD
        image = image[50:150,190:490]

 
        with self.graph.as_default():
            classes = self.sess.run(self.model_output, {
                self.input_image: [image]
            })[0]
            return np.argmax(classes)
 


=======
        if self.tf_session is None:
            # get the traffic light classifier
            config = tf.ConfigProto(log_device_placement=True)
            config.gpu_options.per_process_gpu_memory_fraction = 0.2 # don't hog all the VRAM
            config.operation_timeout_in_ms = 50000 # terminate in 50s if something goes wrong
            self.tf_session = tf.Session(config=config)
            # Classifier for simulator
            saver = tf.train.import_meta_graph('light_classification/m/m.meta')
            saver.restore(self.tf_session, tf.train.latest_checkpoint('./light_classification/m/'))
            # Classifier for Carla
            #saver = tf.train.import_meta_graph('light_classification/carla/carla.meta')
            #saver.restore(self.tf_session, tf.train.latest_checkpoint('./light_classification/carla/'))            
            # get the tensors we need for doing the predictions by name
            tf_graph = tf.get_default_graph()
            self.x = tf_graph.get_tensor_by_name("Placeholder:0")
            self.logits = tf_graph.get_tensor_by_name("logits:0")
            
        pred = self.tf_session.run(self.logits, feed_dict ={self.x: self.get_features(image)})
        softmax = self.tf_session.run(tf.nn.softmax(pred))
        prediction = np.argmax(softmax)
        if (prediction == 0):
            return TrafficLight.RED
        elif (prediction == 1):
            return TrafficLight.GREEN
        elif (prediction == 2):
            return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN
        #with self.graph.as_default():
        #    classes = self.sess.run(self.model_output, {
        #        self.input_image: [image]
        #    })[0]
        #    return np.argmax(classes)
>>>>>>> a3d08ff39c3745923f23212b622390128549f0ff
