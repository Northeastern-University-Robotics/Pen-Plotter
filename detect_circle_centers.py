import numpy as np
import cv2 as cv

def detectCircleCenters(cam):
  while True:
    isTrue, frame = cam.read()
    
    output = frame.copy()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    #larger dp -> smaller accumulator array -> lines can be missed due to noise
    #smaller dp -> larger accumulator array -> different lines may be merged
    # if dp=2, accumulator has half width/height of image

    #minimum distance: 
    #too small -> duplicate circles in a vicinity may not be suppressed
    #too large -> may miss circles
  
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 120, 10)

    if circles is not None:
      circles = np.round(circles[0, :]).astype("int")

      list_of_centers = []

      for (x, y, r) in circles:
        list_of_centers.append((x, y))
        
        #how should the centers be returned? maybe use a yield statement?
        print(list_of_centers)

        #draws the detected circle
        cv.circle(output, (x, y), r, (0, 255, 0), 4)
        #draws the detected circle center as a point
        cv.circle(output, (x, y), 1, (0, 255, 0), 10)
        #displays the coordinates of the detected center point
        cv.putText(output, str((x ,y)), (x, y), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)

      cv.imshow('video', output)

      #press "q" to break the video feed
      k = cv.waitKey(1)
      if k == ord("q"):
        break
  cam.release()
  cv.destroyAllWindows()

detectCircleCenters(cv.VideoCapture(0))