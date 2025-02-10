import pytest
import cv2
from unittest.mock import patch
import numpy as np
import traceback

def run_ex1(RodaAtividade, fname):
    with patch('cv2.imshow'):
        RodaAtividade.load_image(fname)
        bgr = RodaAtividade.bgr
    return {'shape':bgr.shape}

def test_ex1():
    from first_image import ProcessImage
    RodaAtividade = ProcessImage()

    result = {
        'frame01': run_ex1(RodaAtividade, 'arara.jpg'),
    }
    assert result['frame01']['shape'] == (333, 500, 3),  result['frame01']['shape']
    # check_ex1(result)

#####################
# pytest test.py::test_ex2

def run_ex2(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    with patch('cv2.imshow'):
        print(bgr.shape)
        RodaAtividade.run_image(bgr)
        rgb = RodaAtividade.rgb
    return {'shape':rgb.shape}

def test_ex2():
    from webcam import ProcessImage

    RodaAtividade = ProcessImage()

    result = {
        'frame01': run_ex2(RodaAtividade, 'arara.jpg'),
    }
    assert result['frame01']['shape'] == (500, 333, 3),  result['frame01']['shape']

#####################
# pytest test.py::test_ex3

def run_ex3(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    try:
        with patch('cv2.imshow'):
            print(bgr.shape)
            RodaAtividade.run_image(bgr)
            bgr = RodaAtividade.bgr
        return {'mean_array':np.mean(bgr)}
    except Exception as e:
        assert False, traceback.format_exc()

def test_ex3():
    from arara import ProcessImage

    RodaAtividade = ProcessImage()

    result = {
        'frame01': run_ex3(RodaAtividade, 'arara.jpg'),
    }
    assert result['frame01']['mean_array'] == 35.12536736736737,  result['frame01']['mean_array']

#####################
# pytest test.py::test_ex4

def run_ex4(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    try:
        with patch('cv2.imshow'):
            RodaAtividade.run_image(bgr)
            gray = RodaAtividade.gray
        return {'mean_array':np.unique(gray)}
    except Exception as e:
        assert False, traceback.format_exc()

def test_ex4():
    from webcam_BW import ProcessImage

    RodaAtividade = ProcessImage()

    result = {
        'frame01': run_ex4(RodaAtividade, 'arara.jpg'),
    }
    assert np.all(result['frame01']['mean_array'] == np.array([0,255])),  result['frame01']['mean_array']