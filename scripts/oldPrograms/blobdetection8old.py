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
colors = ["green"]#"red","yellow","green"]#"yellow", "

color_dims = {"red":(np.array([0,190,200]).astype(np.uint8), np.array([15,255,255]).astype(np.uint8)),\
              "green":(np.array([50, 80, 150]).astype(np.uint8), np.array([90,255,255]).astype(np.uint8)),\
              "yellow": (np.array([25,150,150]).astype(np.uint8), np.array([45,255,255]).astype(np.uint8))}


#color_dims = {"red":(np.array([0,150,150]).astype(np.uint8), np.array([15,255,255]).astype(np.uint8), np.array([175,150,150]).astype(np.uint8), np.array([0, 255, 255]).astype(np.uint8)),\
#              "green":(np.array([55, 150, 150]).astype(np.uint8), np.array([75, 255, 255]).astype(np.uint8)),\
#              "yellow": (np.array([25,150,150]).astype(np.uint8), np.array([45,255,255]).astype(np.uint8))}

#color_dims = {"red":("r", 150, 1.7, 2.5),\
#    "yellow": ("g",150, .6, 2.2),\
#    "green": ("g", 60, 1.5, 1)}#"g", 60, 1.5, 1.5
redB = ColorRGBA()
redB.r = 255
yellowB = ColorRGBA()
yellowB.r = 255
yellowB.g = 255
greenB = ColorRGBA()
greenB.g = 255
color_map = {"red": redB,"yellow":yellowB,"green":greenB}

#find all the blobs in the thresholded image
def blobdetection(img, cond, col):
    contours, hierarchy = cv2.findContours(img, mode = cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)#change later for cv
    Boxes = []
    ImgBox = []
    for co in contours:
        rect = cv2.minAreaRect(co)
        center = rect[0]
        (w,h) = rect[1]
        angle = rect[2]
        area = w*h
        if not cond(area): continue#could make more efficient
        p = max(w/h,h/w) 
        if p > 1.7: continue#the rectangle probably does not enclose the paper 
        if angle < -30 and angle > -60: continue#too angled
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        Boxes.append((area,center,col))
        ImgBox.append(box)#does not matter the order

    #emp = np.zeros((img.shape))
    #imgcont = cv2.drawContours(emp, contours, -1, (255,255,255), 4)
    #cv2.imshow("cont",imgcont)
    return Boxes, ImgBox

#threshold the image for a certain range of color, to be modified
def thresholdColor(img):
    #CANCEL MESSAGE TEMP
    
    hsv = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.uint8)
    combs = []
    for c in colors:
        colattr = color_dims[c]
#        if c == "red":
 #           (lower1,upper1,lower2,upper2) = colattr
  #          right = cv2.inRange(hsv,lower1,upper1)
   #         left = cv2.inRange(hsv,lower2,upper2)
    #        #rospy.loginfo(right)
     #       combs.append(cv2.bitwise_and(left,right))
      #  else:
        #(lower, upper) = colattr
        lower = np.array([50, 80, 150]).astype(np.uint8)
        upper = np.array([90,255,255]).astype(np.uint8)
        
        cut = cv2.inRange(hsv,lower,upper).astype(np.uint8)
        rospy.loginfo(c)
        rospy.loginfo(colattr)
        combs.append(cut)

    cv2.imshow("n", cut)
    cv2.imshow("img",img)
    cv2.imshow("hsv",hsv)
    cv2.waitKey(1)
    
    return combs

#show information about each interesting blob
def getBlobStats(img, arr, boxarr):
    width, height, cha = img.shape

    outputArea = []
    outputCentroid = []
    outputColor = []
    for b in arr:
        area = Float64(b[0])
        (y,x) = b[1]#center
        colo = b[2]
        cent = Point()
        cent.x = x/width
        cent.y = y/height
        
        outputArea.append(area)
        outputCentroid.append(cent)
        outputColor.append(colo)
    
    Bimg = np.zeros(img.shape).astype(np.uint8)
    cv2.drawContours(Bimg,boxarr,0,(255,255,255),4)
    return outputArea, outputCentroid, outputColor, Bimg.astype(np.uint8)

#main process
def process(img):
    #for every color
    objdetect = np.zeros(img.shape).astype(np.uint8)
    combAll = np.zeros(img.shape).astype(np.uint8)

    Arr = []
    BoxArr = []
    
    rospy.loginfo("START")
    time0 = time.time()

    lowthresarea = 5000
    #highthresarea = 10000000000

    time1 = time.time()
    cond = lambda x: lowthresarea < x
    combs = thresholdColor(img)
    print(combs)
    
    time2 = time.time()
    for i in range(len(colors)):
        col = color_dims[colors[i]]
        comb = combs[i]
        
        arr, blobp = blobdetection(comb, cond, color_map[colors[i]])

        Arr += arr
        BoxArr += blobp
        combC = cv2.cvtColor(comb, cv2.COLOR_GRAY2BGR).astype(np.uint8)
        combAll = np.add(combAll, combC)
    time3 = time.time()
    Arr.sort()#BoxArr does not need to be sorted
    Arr = Arr[::-1]

    areas, centroids, cols, modimag = getBlobStats(combAll, Arr, BoxArr)
    objdetect = np.add(objdetect, modimag)
    time4 = time.time()
    #create message
    mesg = BlobDetections()
    mesg.sizes = areas
    mesg.locations = centroids
    mesg.colors = cols

    objimage = np.empty(img.shape)
    np.copyto(objimage, img)
    objimage = objimage.astype(np.uint8)
    objimage[objdetect == 255] = 255

    rospy.loginfo(time2-time1)
    rospy.loginfo(time3 - time2)
    rospy.loginfo(time4 - time3)
    return mesg, objimage

class Echo:
    def __init__(self):
        self.node_name = "Blob_Detector"
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
        BEG = time.time()
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        #self.sub_image.unregister()
        
        msg, img = process(image_cv)
        END = time.time()
        rospy.loginfo(END-BEG)
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            self.pub_data.publish(msg)
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()

    #start running
if __name__ == "__main__":#may want to set a hertz value
    rospy.init_node("BlobDetector")
    bd = Echo()
    rospy.spin()
