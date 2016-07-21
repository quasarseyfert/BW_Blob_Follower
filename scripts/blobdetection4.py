#!/usr/bin/env python
import cv2
import numpy as np
import math
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
from std_msgs.msg import Float64, ColorRGBA
from geometry_msgs.msg import Point
from racecar.msg import BlobDetections


global colors, color_dims, color_map
colors = ["red", "yellow", "green"]
color_dims = {"red":("r", 150, 1.7, 2.5),\
    "yellow": ("g",150, .6, 2.2),\
    "green": ("g", 60, 1.5, 1.5)}
redB = ColorRGBA()
redB.r = 255
yellowB = ColorRGBA()
yellowB.r = 255
yellowB.g = 255
greenB = ColorRGBA()
greenB.g = 255
color_map = {"red": redB,"yellow":yellowB,"green":greenB}

#find all the blobs in the thresholded image
def blobdetection(img, cond1, cond2, col):
    contours, hierarchy = cv2.findContours(img.copy(), mode = cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)#change later for cv
    Boxes = []
    ImgBox = []
    for co in contours:
        rect = cv2.minAreaRect(co)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        center = rect[0]
        (w,h) = rect[1]
        angle = rect[2]
        area = w*h
        if not cond1(area): continue#could make more efficient
	p = max(w/h,h/w)
	if p> 1.5: continue
        if angle < -30: continue#too angled
        Boxes.append((area,center,col))
        ImgBox.append(box)#does not matter the order

    return Boxes, ImgBox

#threshold the image for a certain range of color, to be modified
def thresholdColor(img, colattr):
    (domchan, dommin, first, second) = colattr
    channels = cv2.split(img)#red, green, blue
    width, height, cha = img.shape
    mult = np.empty((width,height)).astype(np.uint8)
    mult.fill(255)
    red = channels[2].astype(np.uint8)
    green = channels[1].astype(np.uint8)
    blue = channels[0].astype(np.uint8)
    firsttype = np.zeros(img.shape)
    secondtype = np.zeros(img.shape)

    if domchan == "r":
        zerotype = (red > dommin)
        firsttype = np.true_divide(red,green)#r/g
        secondtype = np.true_divide(red,blue)#r/b
    elif domchan == "g":
        zerotype = (green > dommin)
        firsttype = np.true_divide(green,red)#g/r
        secondtype = np.true_divide(green,blue)#g/b

    zerotype = zerotype.astype(np.int)
    firsttype = (firsttype > first).astype(np.int)# & (firsttype < first[1])
    secondtype = (secondtype > second).astype(np.int)# & (secondtype < second[1])
    combined = cv2.bitwise_and(cv2.bitwise_and(zerotype, secondtype), firsttype)
    combined = cv2.multiply(combined.astype(np.uint8), mult)

    return combined

#fix blob parameters here to choose which blobs to consider
def blobParams(thres):
    #params
    lowthresarea = 5000
    #highthresarea = 10000000000
    roundness = 0.85

    #create conditional for acceptable blobs
    cond1 = lambda x: lowthresarea < x

    #determine what "isSquare" means
    cond2 = lambda x: x < roundness
    
    return cond1,cond2

#show information about each interesting blob
def getBlobStats(dom, img, arr, boxarr):
    Bimg = np.zeros(img.shape).astype(np.uint8)
    width, height, channels = dom.shape

    outputArea = []
    outputCentroid = []
    outputColor = []
    for b in arr:
        area = int(b[0])
        (y,x) = b[1]#center
        colo = b[2]
        cent = Point()
        cent.x = x/width
        cent.y = y/height
        
        outputArea.append(area)
        outputCentroid.append(cent)
        outputColor.append(colo)
    for b in boxarr:
        Bimg = cv2.drawContours(Bimg,[b],0,(255,255,255),4)
        
    return outputArea, outputCentroid, outputColor, Bimg.astype(np.uint8)

#main process
def process(img):
    #for every color
    objdetect = np.zeros(img.shape).astype(np.uint8)
    combAll = np.zeros(img.shape).astype(np.uint8)

    Arr = []
    BoxArr = []
    for i in range(len(colors)):
        col = color_dims[colors[i]]
        comb = thresholdColor(img, col)
        
        #find and display the array that describes the blobs
        cond1,cond2 = blobParams(comb)
        arr, blobp = blobdetection(comb, cond1, cond2, color_map[colors[i]])
        Arr += arr
        BoxArr += blobp
        combC = cv2.cvtColor(comb, cv2.COLOR_GRAY2BGR).astype(np.uint8)
        combAll = np.add(combAll, combC)
        
    Arr.sort()#BoxArr does not need to be sorted
    Arr = Arr[::-1]
    
    areas, centroids, cols, modimag = getBlobStats(img, comb, Arr, BoxArr)
    modImag = cv2.cvtColor(modimag, cv2.COLOR_GRAY2BGR).astype(np.uint8)
    objdetect = np.add(objdetect, modImag)

    #create message
    mesg = BlobDetections()
    mesg.sizes = areas
    mesg.locations = centroids
    mesg.colors = cols

    objimage = img.copy()
    #obj3 = cv2.merge((objdetect,objdetect,objdetect))
    objimage[objdetect == 255] = 255

    rospy.loginfo("mesg: " + str(mesg))
    return mesg, objimage

class Echo:
    def __init__(self):
        self.node_name = "BlobDetector"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("/blob_image",\
                Image, queue_size=1)
        self.pub_data = rospy.Publisher("/blob_detections", BlobDetections, queue_size = 1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        msg, img = process(image_cv) 
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            self.pub_data.publish(msg)
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

    #start running
if __name__ == "__main__":#may want to set a hertz value
    rospy.init_node("BlobDetector")
    bd = Echo()
    rospy.spin()
