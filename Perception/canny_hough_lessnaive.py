import numpy as np
import cv2

DISPLAY = True

class HoughCenters:
  def __init__(self, t_low=70, t_high=120, ):
    self._accum_thresh=70
    self._t_low=70
    self._t_high=120
    self.last_circles=[]

  #the rate at which the parameters are updated
  def get_rate(self, num_circles):
    rate = (num_circles - 2) / 3
    return rate
  
  def update(self, num_circles):
    rate = self.get_rate(num_circles)
    if 2 <= num_circles <= 5:
      self._t_low += rate*1.5
      self._t_high += rate*1.5
      self._accum_thresh += rate*1.5
    else:
      self._t_low += rate
      self._t_high += rate
      self._accum_thresh += rate
  
  # is the first point's x and y within the specified range of the second point's x and y respectively?
  def is_in_range(self, c1, c2, range):
    return c2[0]-range <= c1[0] <= c2[0]+range and c2[1]-range <= c1[1] <= c2[1]+range

  def detect(self, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edge_detected_image = cv2.Canny(gray, self._t_low, self._t_high)
    circles = cv2.HoughCircles(edge_detected_image, cv2.HOUGH_GRADIENT, 0.5, 50, param1=0.01, param2=self._accum_thresh, minRadius=0, maxRadius=0)

    circle_centers = []
    if circles is not None:
      detected_circles = np.uint16(np.around(circles))
      for (x, y, r) in detected_circles[0]:
        circle_centers.append((x, y, r))

    # only keeps detected circles if their points are within the range of the previous frame's circles
    good_circles=[]
    if len(self.last_circles) <= 2:
      good_circles = circle_centers
    else:
      for circle in circle_centers:
        filtered_last_circles = filter(lambda c: self.is_in_range(circle, c, 5), self.last_circles)
        if list(filtered_last_circles)!=[]:
         good_circles.append(circle)

    #adjusting the parameters
    self.update(len(good_circles))
    self.last_circles= good_circles

    good_circles_centers = [(x,y) for (x,y,r) in good_circles]
    if DISPLAY:
      return good_circles
    else:
      return good_circles_centers

if DISPLAY:
  cam=cv2.VideoCapture(0)   
  model = HoughCenters()
  while True: 
    isTrue, frame = cam.read()
    centers = model.detect(frame)
    for (x, y, r) in centers:
      #draws the detected circle
      cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
      #draws the detected circle center as a point
      cv2.circle(frame, (x, y), 1, (0, 0, 255), 10)
      #displays the coordinates of the detected center point
      cv2.putText(frame, str((x ,y)), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)

    cv2.imshow('video', frame)

    #press "q" to break the video feed
    k = cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == 27:
      break

  cam.release()
  cv2.destroyAllWindows()





    #larger dp -> smaller accumulator array -> lines can be missed due to noise
    #smaller dp -> larger accumulator array -> different lines may be merged
    # if dp=2, accumulator has half width/height of image

    #minimum distance: 
    #too small -> duplicate circles in a vicinity may not be suppressed
    #too large -> may miss circles

    # If a pixel gradient is higher than the upper threshold, the pixel is accepted as an edge
    # If a pixel gradient value is below the lower threshold, then it is rejected.
    #If the pixel gradient is between the two thresholds, then it will be accepted only if it is connected to a pixel that is above the upper threshold.