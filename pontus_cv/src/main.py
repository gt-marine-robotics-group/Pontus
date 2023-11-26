
import sys, getopt;
import cv2;
import numpy as np;

from constants import *;
import saliency;
import orient;
import shape_match;

def main(img) -> int:

    # img[:,:,0] = np.zeros([img.shape[0], img.shape[1]])
    gray_src = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY);

    srcimg = saliency.differentiate(img, DIFFERENTIATE_HORIZONTAL, algorithm=DIFF_ALGO_LAPLACIAN, show=True);

    # img = cv2.convertScaleAbs(img, alpha=1.2, beta=-30);

    query_image_path = "/Users/quinncattanach/Pontus/pontus_cv/src/assets/interest3.png";

    query = cv2.imread(query_image_path);
    gray_query = cv2.cvtColor(query, cv2.COLOR_BGR2GRAY);

    # _, thresh_src = cv2.threshold(srcimg, 80, 255, cv2.THRESH_BINARY_INV)

    # srcimg = np.array(srcimg, np.uint8)
    # srcimg = (255-srcimg)

    # _, thresh_src = cv2.threshold(srcimg, 80, 255, cv2.THRESH_BINARY_INV)

    _, thresh_q = cv2.threshold(gray_query, 60, 255, cv2.THRESH_BINARY)
    contours_query, heirarchy_query = cv2.findContours(thresh_q, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    q_contour = contours_query[0]

    contours_src, hierarchy_src = cv2.findContours(srcimg, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    sorted_contours = sorted(contours_src, key=cv2.contourArea, reverse=True)

    def siv(contour):
        return (cv2.contourArea(contour) > AREA_CUTOFF)
    sorted_contours = list(filter(siv, sorted_contours))

    closest = (999, []);

    for contour in sorted_contours:
        match = cv2.matchShapes(q_contour, contour, 1, 0.0)
        print(match)
        if match < closest[0]:
            closest = (match, contour);
    
    cv2.drawContours(img, closest[1], -1, (0, 255, 0), 3) 

    cv2.imshow('Contours', img);
    cv2.waitKey(0);

    return EXIT_SUCCESS;

if __name__ == "__main__":
    if (len(sys.argv) == 1):
        img = cv2.imread("/Users/quinncattanach/Pontus/pontus_cv/src/assets/0.jpg");
        exit(main(img));
    options = list(getopt.getopt(sys.argv[1:], "s:t:"))[0]
    for option, argument in options:
        if (option == "-s"):
            try:
                img = cv2.imread(argument);
                exit(main(img));
            except:
                exit(EXIT_FILE_NOT_FOUND);
        elif (option == "-t"):
            argument = int(argument);
            img = cv2.imread(TEST_IMAGES[argument]);
            exit(main(img));
        else:
            print(f"Unknown option '{option}'.");
    exit(EXIT_FAILURE);