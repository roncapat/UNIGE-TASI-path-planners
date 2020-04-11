from plot_path import *
import cv2

result = plot_path()
cv2.imshow("Field D* planner", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
