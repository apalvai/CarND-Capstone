from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
	
        self.labels = [line.rstrip() for line in tf.gfile.GFile('light_classification/model/labels.txt')]
        
        with tf.Session() as session:
            with tf.gfile.FastGFile('light_classification/model/graph.pb', 'rb') as f:
                graph_def = tf.GraphDef()
                graph_def.ParseFromString(f.read())
                session.graph.as_default()
                tf.import_graph_def(graph_def, name='')
                self.session = session

    def process_image(self, image):
	# re-size
        processed_image = cv2.resize(image, (224,224))
	# normalize        
	processed_image = (processed_image - 128.)/128.
	# re-shape        
	processed_image = np.reshape(processed_image, (1, 224,224,3))
        return processed_image

    def run(self, image, labels, input_layer, output_layer, top_n=1):
        softmax = self.session.graph.get_tensor_by_name(output_layer)
        preds = self.session.run(softmax, {input_layer: image})
 	preds = np.squeeze(preds)

	# Reverse sort and take the top_preds
        top_preds = preds.argsort()[-top_n:][::-1]
        for index in top_preds:
            label = labels[index]
            score = preds[index]
	    # rospy.logdebug('light: %s with score: %f', label, score)
        
        return label
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Implement light color prediction
	processed_image = self.process_image(image)
        predicted_label = self.run(processed_image, self.labels, 'input:0', 'final_result:0', 1)
	
	if predicted_label == 'yellow':
	    return TrafficLight.YELLOW
	elif predicted_label == 'green':
	    return TrafficLight.GREEN
	elif predicted_label == 'red':
	    return TrafficLight.RED
	else:
            return TrafficLight.UNKNOWN
