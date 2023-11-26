import cv2;
import numpy as np;
from constants import *;

def saliency_filter(img: any, 
                    inplace: bool = False,
                    show: bool = False, 
                    thresh: int = 60,
                    threshtype: int = cv2.THRESH_BINARY_INV) -> any:
    #   Remove the blue channel.
    imcpy = img if inplace else img.copy();

    imcpy[:,:,0] = np.zeros([imcpy.shape[0], imcpy.shape[1]]);

    g_img = cv2.cvtColor(imcpy, cv2.COLOR_BGR2GRAY);

    # TODO: Normalize the contrast, or determine a threshold based on the passed image. 
    # While this threshold may work for many of the testing images, changes in lighting could cause problems.

    # TODO: Test differentiating here.
    _, thresh_g_img = cv2.threshold(g_img, thresh, 255, threshtype);

    if (show):
        cv2.imshow('Image after Saliency Filter', thresh_g_img);
        cv2.waitKey(0);

    return thresh_g_img;


def differentiate(img: any, 
                  direction: int,
                  algorithm: int = DIFF_ALGO_SOBEL,
                  inplace: bool = False,
                  show: bool = False,) -> any:
    imcpy = img if inplace else img.copy();

    g_img = cv2.cvtColor(imcpy, cv2.COLOR_BGR2GRAY);
    g_img = cv2.blur(g_img, (5, 5));

    if (algorithm == DIFF_ALGO_SOBEL):
        out = cv2.Sobel(g_img,cv2.CV_64F,1,0,ksize=5)
    elif (algorithm == DIFF_ALGO_LAPLACIAN):
        out = cv2.Laplacian(g_img,cv2.CV_64F)

    out = cv2.blur(out, (5, 5));

    if (show):
        cv2.imshow('Image after Saliency Filter', out);
        cv2.waitKey(0);
    print(out.dtype)

    img = cv2.normalize(src=out, dst=None, alpha=0.0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # img = img * 2

    img = cv2.blur(img, (5, 5));

    img = cv2.convertScaleAbs(img, alpha=2.7, beta=-275);

    img = (255 - img);

    cv2.imshow("Window", img)
    cv2.waitKey(0);

    return img;