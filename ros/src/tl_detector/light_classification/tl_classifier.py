import numpy as np
import tensorflow as tf
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        #self.model_path = model_path
        #self.labels = [TrafficLight.NaN, TrafficLight.R, TrafficLight.Y, TrafficLight.G, TrafficLight.NaN]
        #PATH_TO_CKPT = self.model_path + '/checkpoints/frozen_inference_graph.pb'
        self.tf_session = None

        #self.graph = tf.Graph()
        #with self.graph.as_default():
        #    gpu_options = tf.GPUOptions(allow_growth=True)
        #    config = tf.ConfigProto(log_device_placement=False, gpu_options=gpu_options)
        #    self.sess = tf.Session(config=config)
                #sess.run(tf.initialize_all_variables())
        #    server = tf.train.import_meta_graph('lenet.meta')
        #    server.restore(sess, tf.train.latest_checkpoint('./'))
        #    self.input_image = tf.get_default_graph().get_tensor_by_name("input_image:0")
        #    self.model_output = tf.get_default_graph().get_tensor_by_name("model_output:0")
        #pass

    def get_features(self, img):
        #img = (img*255).astype(np.uint8)
        img = cv2.resize(img,(224,224))
        channel1_hist = np.histogram(img[:,:,0], bins=256, range=(0,256))
        channel2_hist = np.histogram(img[:,:,1], bins=256, range=(0,256))
        channel3_hist = np.histogram(img[:,:,2], bins=256, range=(0,256))
        hist_features = [np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0])).reshape(-1,1)]
        return hist_features
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #return TrafficLight.UNKNOWN
        if self.tf_session is None:
            # get the traffic light classifier
            config = tf.ConfigProto(log_device_placement=True)
            config.gpu_options.per_process_gpu_memory_fraction = 0.2 # don't hog all the VRAM
            config.operation_timeout_in_ms = 50000 # terminate in 50s if something goes wrong
            self.tf_session = tf.Session(config=config)
            saver = tf.train.import_meta_graph('m.meta')
            saver.restore(self.tf_session, tf.train.latest_checkpoint('./'))
            
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