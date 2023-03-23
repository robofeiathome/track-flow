
import cv2 
import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import mode
from argparse import ArgumentParser
def main(img):
    if __name__ == '__main__':
        parser = ArgumentParser()
        parser.add_argument('-rec', '--record', default=False, action='store_true', help='Record?')
        parser.add_argument('-pscale', '--pyr_scale', default=0.5, type=float,
                        help='Image scale (<1) to build pyramids for each image')
        parser.add_argument('-l', '--levels', default=3, type=int, help='Number of pyramid layers')
        parser.add_argument('-w', '--winsize', default=15, type=int, help='Averaging window size')
        parser.add_argument('-i', '--iterations', default=3, type=int,
                        help='Number of iterations the algorithm does at each pyramid level')
        parser.add_argument('-pn', '--poly_n', default=5, type=int,
                        help='Size of the pixel neighborhood used to find polynomial expansion in each pixel')
        parser.add_argument('-psigma', '--poly_sigma', default=1.1, type=float,
                        help='Standard deviation of the Gaussian that is used to smooth derivatives used as a basis for the polynomial expansion')
        parser.add_argument('-th', '--threshold', default=10.0, type=float, help='Threshold value for magnitude')
        parser.add_argument('-p', '--plot', default=False, action='store_true', help='Plot accumulators?')
        parser.add_argument('-rgb', '--rgb', default=False, action='store_true', help='Show RGB mask?')
        parser.add_argument('-s', '--size', default=10, type=int, help='Size of accumulator for directions map')

        args = vars(parser.parse_args())

        directions_map = np.zeros([args['size'], 5])

        cap = img
        if args['record']:
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            codec = cv2.VideoWriter_fourcc(*'MPEG')
            out = cv2.VideoWriter('out.avi', codec, 10.0, (w, h))

        if args['plot']:
            plt.ion()

        frame_previous = cap.read()[1]
        gray_previous = cv2.cvtColor(frame_previous, cv2.COLOR_BGR2GRAY)
        hsv = np.zeros_like(frame_previous)
        hsv[:, :, 1] = 255
        param = {
            'pyr_scale': args['pyr_scale'],
            'levels': args['levels'],
            'winsize': args['winsize'],
            'iterations': args['iterations'],
            'poly_n': args['poly_n'],
            'poly_sigma': args['poly_sigma'],
            'flags': cv2.OPTFLOW_LK_GET_MIN_EIGENVALS
        }

        

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(gray_previous, gray, None, **param)
        mag, ang = cv2.cartToPolar(flow[:, :, 0], flow[:, :, 1], angleInDegrees=True)
        ang_180 = ang/2
        gray_previous = gray
            
        move_sense = ang[mag > args['threshold']]
        move_mode = mode(move_sense)[0]

        if 10 < move_mode <= 100:
            directions_map[-1, 0] = 1
            directions_map[-1, 1:] = 0
            directions_map = np.roll(directions_map, -1, axis=0)
        elif 100 < move_mode <= 190:
            directions_map[-1, 1] = 1
            directions_map[-1, :1] = 0
            directions_map[-1, 2:] = 0
            directions_map = np.roll(directions_map, -1, axis=0)
        elif 190 < move_mode <= 280:
            directions_map[-1, 2] = 1
            directions_map[-1, :2] = 0
            directions_map[-1, 3:] = 0
            directions_map = np.roll(directions_map, -1, axis=0)
        elif 280 < move_mode or move_mode < 10:
            directions_map[-1, 3] = 1
            directions_map[-1, :3] = 0
            directions_map[-1, 4:] = 0
            directions_map = np.roll(directions_map, -1, axis=0)
        else:
            directions_map[-1, -1] = 1
            directions_map[-1, :-1] = 0
            directions_map = np.roll(directions_map, 1, axis=0)

        if args['plot']:
            plt.clf()
            plt.plot(directions_map[:, 0], label='Down')
            plt.plot(directions_map[:, 1], label='Right')
            plt.plot(directions_map[:, 2], label='Up')
            plt.plot(directions_map[:, 3], label='Left')
            plt.plot(directions_map[:, 4], label='Waiting')
            plt.legend(loc=2)
            plt.pause(1e-5)
            plt.show()

        loc = directions_map.mean(axis=0).argmax()
        if loc == 0:
            text = 'Moving down'
        elif loc == 1:
            text = 'Moving to the right'
        elif loc == 2:
            text = 'Moving up'
        elif loc == 3:
            text = 'Moving to the left'
        else:
            text = 'WAITING'

        hsv[:, :, 0] = ang_180
        hsv[:, :, 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        frame = cv2.flip(frame, 1)
        cv2.putText(frame, text, (30, 90), cv2.FONT_HERSHEY_COMPLEX, frame.shape[1] / 500, (0, 0, 255), 2)


    cap.release()
    if args['record']:
        out.release()
    if args['plot']:
        plt.ioff()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

