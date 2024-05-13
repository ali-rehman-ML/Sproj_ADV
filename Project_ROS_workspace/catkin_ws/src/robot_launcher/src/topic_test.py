#!/usr/bin/env python3
import rospy
import cv2
import sensor_msgs
from sensor_msgs.msg import Image
import time
from cv_bridge import CvBridge
import numpy as np
import os
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import matplotlib.pyplot as plt
import PIL
import time
import os
TRT_LOGGER = trt.Logger()


engine_file = "unet_trt.engine"
input_file  = "color_139_png.rf.9bb2cecaee2631226490870f8dd44327.jpg"
output_file = "output.ppm"


def imgmsg_to_cv2(img_msg, dtype=np.uint8):
    # it should be possible to determine dtype from img_msg.encoding but there is many different cases to take into account
    # original function args: imgmsg_to_cv2(img_msg, desired_encoding = "passthrough")
    return np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, -1)

def cv2_to_imgmsg(cvim, encoding="passthrough", header=None):
    if not isinstance(cvim, (np.ndarray, np.generic)):
        raise TypeError('Your input type is not a numpy array')
    # prepare msg
    img_msg = sensor_msgs.msg.Image()
    img_msg.height = cvim.shape[0]
    img_msg.width = cvim.shape[1]
    if header is not None:
        img_msg.header = header
    # encoding handling
    numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                            'int16': '16S', 'int32': '32S', 'float32': '32F',
                            'float64': '64F'}
    numpy_type_to_cvtype.update(dict((v, k) for (k, v) in numpy_type_to_cvtype.items()))
    if len(cvim.shape) < 3:
        cv_type = '{}C{}'.format(numpy_type_to_cvtype[cvim.dtype.name], 1)
    else:
        cv_type = '{}C{}'.format(numpy_type_to_cvtype[cvim.dtype.name], cvim.shape[2])
    if encoding == "passthrough":
        img_msg.encoding = cv_type
    else:
        img_msg.encoding = encoding
    if cvim.dtype.byteorder == '>':
        img_msg.is_bigendian = True
    # img data to msg data
    img_msg.data = cvim.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height

    return img_msg

class ImageConverter:

    def __init__(self):
        rospy.init_node('image_listener', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=10)
        self.br = CvBridge()
        self.image1=None
        self.engine=None
        self.context=None
        self.binding=None
        self.input_memory=None
        self.output_memory=None
        self.stream=None
        self.output_buffer=None
        self.mask_pub=rospy.Publisher("road_mask",Image,queue_size=10 )
    def callback(self,msg):
        self.image1=imgmsg_to_cv2(msg)

    def load_engine(self,engine_file_path):
        assert os.path.exists(engine_file_path)
        print("Reading engine from file {}".format(engine_file_path))
        with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            return runtime.deserialize_cuda_engine(f.read())


    def preprocess(self,image):
        # Mean normalization
        img=image
        img=img.resize((512,512))
        img=np.asarray(img)/float(255)
        data=img.astype('float32')
        # print(data.shape)

        # mean = np.array([0.485, 0.456, 0.406]).astype('float32')
        # stddev = np.array([0.229, 0.224, 0.225]).astype('float32')
        # data = (np.asarray(image).astype('float32') / float(255.0) - mean) / stddev
        # Switch from HWC to to CHW order
        return np.moveaxis(data, 2, 0)

    def load_input(self,img):
        # img=cv2.imread(path)#Image.open(path)
        # img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        img=PIL.Image.fromarray(img)

        x=self.preprocess(img)
        # x=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        # x=cv2.resize(img,(512,512)).astype('float32')
        
        # # x=np.asarray(x)/(255)
        # x=(x/255).astype('float32')
        # x=np.moveaxis(x, 2, 0)
        # # print("here ", x.max(),x.min(),x.shape, type(x))

        inp=np.ascontiguousarray(x)
        return inp
    def inferrence(self,x):
        cuda.memcpy_htod_async(self.input_memory, x, self.stream)
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
        cuda.memcpy_dtoh_async(self.output_buffer, self.output_memory, self.stream)
        self.stream.synchronize()
        return np.reshape(self.output_buffer, (1,5,512, 512))

            

    def warm_engine(self):
        print("Loading Engine")
        t1=time.time()
        self.engine=self.load_engine(engine_file)
        print("Engine Loaded in ", time.time()-t1)

        img=PIL.Image.open(input_file)
        input_image = self.preprocess(img)
        image_width = img.width
        image_height = img.height

        self.context=self.engine.create_execution_context()

        self.context.set_binding_shape(0, (1, 3, 512, 512))
        self.bindings = []
        for binding in self.engine:
            binding_idx = self.engine.get_binding_index(binding)
            size = trt.volume(self.context.get_binding_shape(binding_idx))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            if self.engine.binding_is_input(binding):
                input_buffer = np.ascontiguousarray(input_image)
                self.input_memory = cuda.mem_alloc(input_image.nbytes)
                self.bindings.append(int(self.input_memory))
            else:
                self.output_buffer = cuda.pagelocked_empty(size, dtype)
                self.output_memory = cuda.mem_alloc(self.output_buffer.nbytes)
                self.bindings.append(int(self.output_memory))
        self.stream = cuda.Stream()
        t1=time.time()
        print("warming engine")
        self.inferrence(input_buffer)
        print("done warming up ",time.time()-t1)


    def run(self):
        self.warm_engine()
        
        while not rospy.is_shutdown(): 
            if self.image1 is not None:
                x=self.load_input(self.image1)
                out=self.inferrence(x)
                o=np.argmax(out,axis=1)*(50)
                o=o.squeeze()
                print(o.shape)
                o=np.clip(o,0,255).astype(np.uint8)
                mask_msg=cv2_to_imgmsg(o)
                self.mask_pub.publish(mask_msg)





        #         cv2.imshow("image",self.image1)
        #         cv2.imshow("mask ",o)
            
        #         cv2.waitKey(1)
            
            
        # cv2.destroyAllWindows()
            
if __name__ == '__main__':
    imc=ImageConverter()
    time.sleep(0.5)
    imc.run()
    
        



