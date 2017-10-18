from styx_msgs.msg import TrafficLight
from simple_detector import image_preprocessing, simple_detector

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, raw_image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image = image_preprocessing(raw_image)
        result = simple_detector(image)
        if result == -1:
            TrafficLight.UNKNOWN
        return result
