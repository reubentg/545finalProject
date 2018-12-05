import numpy as np
import cv2
from matplotlib import pyplot as plt


# Initialize plot line object(s). Turn on interactive plotting and show plot.
lw = 3
alpha = 0.5
bins = 256
resizeWidth = 50

cap = cv2.VideoCapture(1)

# Initialize plot.
# fig, ax = plt.subplots()
# ax.set_xlabel('Bin')
# ax.set_ylabel('Frequency')
# ax.set_title('Histogram(s)')

# lineRed, = ax.plot(np.arange(bins), np.zeros((bins,)), c='r', lw=lw, alpha=alpha, label='lineMask_Red')
# lineBlue, = ax.plot(np.arange(bins), np.zeros((bins,)), c='b', lw=lw, alpha=alpha, label='lineMask_Blue')

# ax.set_xlim(0, bins - 1)
# ax.set_ylim(0, 1)
# ax.legend()

# plt.ion()
# plt.show()


while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    height, width, channels = frame.shape
    print height, width, channels
    #height = 480
    #width = 640
    #channel = 3

   # if resizeWidth > 0:
   #     (height, width) = frame.shape[:2]
   #     resizeHeight = int(float(resizeWidth / width) * height)
   # #    frame = cv2.resize(frame, (resizeWidth, resizeHeight),
   #         interpolation=cv2.INTER_AREA)

    # Normalize histograms based on number of pixels per frame.
    numPixels = np.prod(frame.shape[:2])

    # Our operations on the frame come here
    #img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # BGR color to gray level
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    # define range of red color in HSV NOT THE BEST ON THE FLOOR
    lower_red = np.array([0,30, 30])
    upper_red= np.array([14, 255, 255])
    mask_red0 = cv2.inRange(img, lower_red, upper_red)

    lower_red = np.array([160,30, 30])
    upper_red= np.array([180, 255, 255])
    mask_red1 = cv2.inRange(img, lower_red, upper_red)

    mask_red = (mask_red0 + mask_red1)

    #define range of blue color in RGB
   # lower_red = np.array([150, 50, 50])
    #  upper_red= np.array([255, 155, 155])
   # mask_red = cv2.inRange(img, lower_red, upper_red)

    # define range of blue color in HSV WORKED WELL
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])


    # Threshold the HSV image to get only red colors & blue colors
    #mask_red = cv2.inRange(img, lower_red, upper_red)
    mask_blue = cv2.inRange(img, lower_blue, upper_blue)

    mask_red_blur = cv2.GaussianBlur(mask_red, (7,7), 0 )
    mask_blue_blur = cv2.GaussianBlur(mask_blue, (7, 7), 0)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask_red_blur)
    res2 = cv2.bitwise_and(frame, frame, mask=mask_blue_blur)

    cropped_fl_red = mask_red[0:480, 0:150]
    cropped_l_red = mask_red[0:480, 151:300]
    cropped_c_red = mask_red[0:480, 300:340]
    cropped_r_red = mask_red[0:480, 340:490]
    cropped_fr_red = mask_red[0:480, 491:640]

    # Display the resulting image
    #cv2.imshow('HSV', img)
    #cv2.imshow('mask', mask)
    #cv2.imshow('mask_blue', mask_blue)
    cv2.imshow('red', res)
    cv2.imshow('blue', res2)
    cv2.imshow('L', cropped_l_red)
    cv2.imshow('frame', frame)
    cv2.imshow('RedMask', mask_red)

    #cv2.imshow('BlueMask', mask_blue)

  #  cv2.imshow('FL', cropped_fl_red)
   # cv2.imshow('L', cropped_l_red)
    #cv2.imshow('c', cropped_c_red)
    #cv2.imshow('R', cropped_r_red)
    #cv2.imshow('FR', cropped_fr_red)

    histRed = cv2.calcHist([mask_red], [0], None, [256], [0, 256])

    histogramRed = cv2.calcHist([img], [0], mask_red, [bins], [0, 255]) / numPixels
    histogramBlue = cv2.calcHist([img], [0], mask_blue, [bins], [0, 255]) / numPixels

    lineRed.set_ydata(histogramRed)
    lineBlue.set_ydata(histogramBlue)

    fig.canvas.draw()

    if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
