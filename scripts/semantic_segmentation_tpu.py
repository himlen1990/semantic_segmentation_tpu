#!/usr/bin/env python3
"""A ROS wrapper of the example of semantic segmentation from pycoral
more detail can be found in https://github.com/google-coral/pycoral/blob/master/examples/semantic_segmentation.py
"""

import roslib
roslib.load_manifest("semantic_segmentation_tpu")
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageMsg
from pycoral.adapters import common
from pycoral.adapters import segment
from pycoral.utils.edgetpu import make_interpreter
from PIL import Image
import sys
import numpy as np

class semantic_segmentation_tpu:
    def __init__(self, model, device=':0', keep_aspect_ratio=True):
        
        self.img_sub = rospy.Subscriber("~input", ImageMsg, self.callback)
        self.interpreter = make_interpreter(model, device=device)
        self.interpreter.allocate_tensors()
        self.model_input_width, self.model_input_height = common.input_size(self.interpreter)
        self.keep_aspect_ratio = keep_aspect_ratio
        self.bridge = CvBridge()
        
        
    def create_pascal_label_colormap(self):
        """Creates a label colormap used in PASCAL VOC segmentation benchmark.

        Returns:
        A Colormap for visualizing segmentation results.
        """
        colormap = np.zeros((256, 3), dtype=int)
        indices = np.arange(256, dtype=int)

        for shift in reversed(range(8)):
            for channel in range(3):
                colormap[:, channel] |= ((indices >> channel) & 1) << shift
            indices >>= 3

        return colormap


    def label_to_color_image(self,label):
        """Adds color defined by the dataset colormap to the label.

        Args:
        label: A 2D array with integer type, storing the segmentation label.

        Returns:
        result: A 2D array with floating type. The element of the array
        is the color indexed by the corresponding element in the input label
        to the PASCAL color map.

        Raises:
        ValueError: If label is not of rank 2 or its value is larger than color
        map maximum entry.
        """
        if label.ndim != 2:
            raise ValueError('Expect 2-D input label')

        colormap = self.create_pascal_label_colormap()

        if np.max(label) >= len(colormap):
            raise ValueError('label value too large.')

        return colormap[label]

    def callback(self, data):
      
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        img = Image.fromarray(cv_image)
        if self.keep_aspect_ratio:
            resized_img, _ = common.set_resized_input(
                self.interpreter, img.size, lambda size: img.resize(size, Image.ANTIALIAS))
        else:
            resized_img = img.resize((self.model_input_width, self.model_input_height), Image.ANTIALIAS)
            common.set_input(interpreter, resized_img)

        self.interpreter.invoke()
        
        result = segment.get_output(self.interpreter)
        if len(result.shape) == 3:
            result = np.argmax(result, axis=-1)

        # If keep_aspect_ratio, we need to remove the padding area.
        new_width, new_height = resized_img.size
        result = result[:new_height, :new_width]
        mask_img = Image.fromarray(self.label_to_color_image(result).astype(np.uint8))

        # Concat resized input image and processed segmentation results.
        output_img = Image.new('RGB', (2 * new_width, new_height))
        output_img.paste(resized_img, (0, 0))
        output_img.paste(mask_img, (self.model_input_width, 0))
        original_width, original_height = img.size
        recovered_cvimg = np.array(output_img.resize((2*original_width, original_height), Image.ANTIALIAS))
        cv2.imshow("resizedimg",recovered_cvimg)
        cv2.waitKey(3)        


def main(args):
    rospy.init_node('semantic_segmentation_tpu', anonymous=True)
    
    model_path = rospy.get_param('~model_path')
    device = rospy.get_param('~device', default=':0')
    keep_aspect_ratio = rospy.get_param('~keep_aspect_ratio', default=True)
    sst = semantic_segmentation_tpu(model_path, device, keep_aspect_ratio)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)
