import cv2;
import numpy as np;

#   Just a wrapper function for a default opencv shape matching algorithm.
def shape_match_basic(c1: any, c2: any) -> float:
    return cv2.matchShapes(c1, c2, 1, 0.0);

#   Calculate the frechet distance between two contours
def discrete_frechet_distance(c1: any, c2: any) -> float:
    #   TODO: Normalize the contours by making them the same size. 
    #   TODO: Orient the contours to a best-guess match. (Or try multiple orientations and take the lowest value..? whichever is less computationally expensive...)
    #   TODO: Calculate the frechet distance using fast DFD algorithm.
    return None;