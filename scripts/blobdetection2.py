#PROBLEMS
#in a blob, need to consider dangling lines that might change perimeter to make it higher

#computer vision imports
import numpy as np
import cv2

#basic utilities imports
import math
import time

import rospy

#subscriber message type
from sensor_msgs.msg import Image

#publisher message types
from racecar.msg import BlobDetections#I think this is enough


#make lower and upper bounds from two colors
def rearrange(arr1,arr2):
    G1 = []
    G2 = []
    for i in range(len(arr1)):
        G1.append(min(arr1[i], arr2[i]))
        G2.append(max(arr1[i], arr2[i]))
    G1 = np.array(G1)
    G2 = np.array(G2)
    return G1, G2

#find all the blobs in the thresholded image
def blobdetection(img, cond1, cond2):
    contours, hierarchy = cv2.findContours(img.copy(), mode = cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)#2.4.8 MOD
    Boxes = []
    for co in contours:
        rect = cv2.minAreaRect(co)
        box = cv2.cv.BoxPoints(rect)#2.4.8 MOD
        box = np.int0(box)
        center = rect[0]
        (w,h) = rect[1]
        angle = rect[2]
        area = w*h
        if not cond1(area): continue
        Boxes.append((box, area, center))

    #emp = np.zeros((img.shape))
    #imgcont = cv2.drawContours(emp, contours, -1, (255,255,255), 4)
    #cv2.imshow("cont",imgcont2)
    return Boxes

#threshold the image for a certain range of color, to be modified
def thresholdColor(img, maincol, ccbound, cbound):
    
    #hsv arrays
    #counterclockwise boundary
    m1 = np.zeros((1,1,3)).astype(np.uint8)
    m1[0][0]= np.array(ccbound)
    #main color
    m2 = np.zeros((1,1,3)).astype(np.uint8)
    m2[0][0] = np.array(maincol)
    #clockwise boundary
    m3 = np.zeros((1,1,3)).astype(np.uint8)
    m3[0][0] = np.array(cbound)

    #convert rgb arrays
    M1 = cv2.cvtColor(m1,cv2.COLOR_HSV2BGR)[0][0].astype(np.uint8)
    M2 = cv2.cvtColor(m2,cv2.COLOR_HSV2BGR)[0][0].astype(np.uint8)
    M3 = cv2.cvtColor(m3,cv2.COLOR_HSV2BGR)[0][0].astype(np.uint8)
    '''
    M1 = np.array(ccbound).astype(np.uint8)
    M2 = np.array(maincol).astype(np.uint8)
    M3 = np.array(cbound).astype(np.uint8)
    '''
    #considering the clockwise side of the color
    G1, G2 = rearrange(M2,M3)
    cut = cv2.inRange(img, G1,G2).astype(np.uint8)
    #good for debugging
    #mod = cv2.cvtColor(cut, cv2.COLOR_GRAY2BGR)
    #img1 = cv2.bitwise_and(src1=img, src2= mod)

    #if 0, need to make it capped
    #M2[0] = 180#make bounds high, may need to change
    #considering the counterclockwise side of the color
    G1, G2 = rearrange(M2,M1)
    cut2 = cv2.inRange(img, G1,G2).astype(np.uint8)
    #good for debugging
    #mod2 = cv2.cvtColor(cut2, cv2.COLOR_GRAY2BGR)
    #img2 = cv2.bitwise_and(src1=img, src2=mod2)

    #combine the two sides of the color
    comb = cv2.bitwise_or(cut,cut2)
    return comb

#fix blob parameters here to choose which blobs to consider
def blobParams(thres):
    #params
    lowthresarea = 10000
    #highthresarea = 10000000000
    roundness = 0.85

    #create conditional for acceptable blobs
    cond1 = lambda x: lowthresarea < x

    #determine what "isSquare" means
    cond2 = lambda x: x < roundness
    
    return cond1,cond2

#show information about each interesting blob
#use this to publish to correct place
def displayBlobStats(dom, img, arr):#output
    Bimg = dom.copy()
    #print stats on each blob
    #show bounding box of each blob
    count = 0
    height, width, channels = dom.shape
    print(dom.shape)
    for b in arr:
        box = b[0]
        area = int(b[1])
        (y,x) = b[2]#center
        X = x/height
        Y = y/width
        print("area: %d")%area
        print("center: (%f,%f)")%(X,Y)
        cv2.drawContours(Bimg,[box],0,(255,255,255),4)#2.4.8 MOD

    
    cv2.imshow("Bimg", Bimg)

#main process
def callback():#data): #need to simplify
    BEG = time.time()
    img = cv2.imread("frame0070.jpg")
    
    #threshold accepts the image then the hsv bounds {main color, cccolor, ccolor}
    #main color is especially important for reds or magentas because it is on the boundary
    #prob should keep sat/light of cccolor and ccolor same and main diff
    comb = thresholdColor(img, [0, 150, 195], [175, 190, 207], [15, 240, 250])

    
    #find and display the array that describes the blobs
    cond1,cond2 = blobParams(comb)
    arr = blobdetection(comb, cond1, cond2)

    displayBlobStats(img, comb, arr)
    
    #give time in order to determine hertz of cv process later
    
    END = time.time()
    print("Time: %f")%(END-BEG)
    
    #cv2.imshow("image", img)
    #cv2.imshow("Image",comb)
    cv2.waitKey(0)

#start running
if __name__ == "__main__":
    #test for different colors: red, yellow, green
    callback()
    rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback)
    
