import csv
import cv2
from cv2 import Subdiv2D
import numpy as np
from math import inf

WIDTH = 800
HEIGHT = 700

track_points = []
# column major so width and height order is reversed
track_image = np.full((HEIGHT, WIDTH, 3), 125, np.uint8)
screen_image = np.full((HEIGHT, WIDTH, 3), 125, np.uint8)
delaunay_subdivision = cv2.Subdiv2D()
# Gets X coordinate Y coordinate outputs a boolean
point_at_is_yellow_lookup = [[False for _ in range(HEIGHT + 1)] for _ in range(WIDTH + 1)]
waypoints = []

def import_track(file_path):
    global point_at_is_yellow_lookup
    global track_points
    with open(file_path, 'r') as track_file:
        csv_reader = csv.reader(track_file)
        header = next(csv_reader)
        car = next(csv_reader)

        for row in csv_reader:
            point = [row[0], float(row[1]), float(row[2])]
            track_points.append(point)

        min_value = inf
        max_value = -inf

        for point in track_points:
            if point[1] < min_value:
                min_value = point[1]

            if point[2] < min_value:
                min_value = point[2]

            if point[1] > max_value:
                max_value = point[1]
            
            if point[2] > max_value:
                max_value = point[2]

        # Normalization + 
        # Transform from [-1, 1] -> [0, 1] -> [0, width], [0, height]

        absolute_maxima = max(abs(min_value), abs(max_value))

        if(absolute_maxima != 0):
            for i in range(len(track_points)):
                track_points[i][1] = int(((track_points[i][1] / absolute_maxima + 1.0) / 2.0) * WIDTH)
                track_points[i][2] = int(((track_points[i][2] / absolute_maxima + 1.0) / 2.0) * HEIGHT)

                point_at_is_yellow_lookup[track_points[i][1]][track_points[i][2]] = True if track_points[i][0] == 'yellow' else False
def colorOf(cone):
    if cone == 'blue':
        return (255, 0, 0)
    elif cone == 'yellow':
        return (0, 255, 255)
    elif cone == 'big_orange':
        return (0, 150, 255)
    else:
        return (126, 126, 126)
    
def plot_track_points(target_image):
    global track_points
    global point_at_is_yellow_lookup
    global track_image
    global screen_image
    
    for point in track_points:

        target_image = cv2.circle(target_image, (round(point[1]), round(point[2])), radius = 5, color = colorOf(point[0]), thickness=-1)

def blit_to_screen(image, flipY=False):
    global screen_image
    
    screen_image = cv2.addWeighted(cv2.flip(image, 0) if flipY else image, 1.0, screen_image, 0.0, 0)

def update_screen():
    global screen_image

    cv2.imshow('Delaunay Triangulation demo', screen_image)
    cv2.waitKey(0)

def init_delaunay():
    global delaunay_subdivision
    global waypoints
    global track_points
    global point_at_is_yellow_lookup

    delaunay_subdivision.initDelaunay((0, 0, WIDTH, HEIGHT))

   # points = []

    for point in track_points:
        if(point[1] < 0 or point[2] < 0):
            continue
        if(point[1] > WIDTH or point[2] > HEIGHT):
            continue

        delaunay_subdivision.insert((clamp(point[1], 1, WIDTH - 1), clamp(point[2], 1, HEIGHT - 1)))
    
    for edge in delaunay_subdivision.getEdgeList():
        if(edge[0] < 0 or edge[1] < 0 or edge[2] < 0 or edge[3] < 0):
            continue
        if(edge[0] > WIDTH or edge[1] > HEIGHT or edge[2] > WIDTH or edge[3] > HEIGHT):
            continue
        
        if((point_at_is_yellow_lookup[int(edge[0])][int(edge[1])] + point_at_is_yellow_lookup[int(edge[2])][int(edge[3])]) == 1):
            # we have blue - yellow pair or points thus this middle of this edge contains one of our waypoints
            middle_point = [edge[0] * 0.5 + edge[2] * 0.5, edge[1] * 0.5 + edge[3] * 0.5]
            waypoints.append(middle_point)
            cv2.line(track_image, (int(edge[0]), int(edge[1])), (int(edge[2]), int(edge[3])), color=(0, 0, 255), thickness=2)


def clamp(value, low, high):
    
    if(value >= low and value <= high):
        return value

    if(value > high):
        return high
    
    if(value < low):
        return low

def draw_delaunay(target_image):
    global delaunay_subdivision

    triangle_list = delaunay_subdivision.getTriangleList()

    for vec6 in triangle_list:

       pointA = (round(vec6[0]), round(vec6[1]))
       pointB = (round(vec6[2]), round(vec6[3]))
       pointC = (round(vec6[4]), round(vec6[5]))

       cv2.line(target_image, pointA, pointB, color=(255, 255, 255), thickness=1)
       cv2.line(target_image, pointB, pointC, color=(255, 255, 255), thickness=1)
       cv2.line(target_image, pointC, pointA, color=(255, 255, 255), thickness=1)

def plot_waypoints(target_image):
    for point in waypoints:
        cv2.circle(target_image, (round(point[0]), round(point[1])), radius=3, color=(255, 255, 255), thickness=-1)

def optimize_path(target_image):
    pass

def draw_path(target_image):
    for i in range(0, len(waypoints) - 1):
        cv2.line(target_image, (round(waypoints[i][0]), round(waypoints[i][1])), (round(waypoints[i + 1][0]), round(waypoints[i + 1][1])), color=(0, 0, 255), thickness=2)


def main(args=None):
    import_track("../resource/track.csv")
    
    init_delaunay()
    draw_delaunay(track_image)
    plot_waypoints(track_image)
    #draw_path(track_image)

    plot_track_points(track_image)
    blit_to_screen(track_image, False)
    update_screen()

    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
