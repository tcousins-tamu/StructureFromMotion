from sfm.py import *
from utils import *
import argparse


def SetArguments(parser):

    # Arguments for SFM
    # directory stuff
    parser.add_argument('--data_dir', action='store', type=str, default='C:/Users/tcous/Desktop/school/CSCE748/project/Data/', dest='data_dir',
                        help='root directory containing input data (default: C:/Users/tcous/Desktop/school/CSCE748/project/Data/)')
    parser.add_argument('--dataset', action='store', type=str, default='fountain-P11', dest='dataset',
                        help='name of dataset (default: fountain-P11)')
    parser.add_argument('--ext', action='store', type=str, default='jpg,png', dest='ext',
                        help='comma seperated string of allowed image extensions \
                        (default: jpg,png)')
    parser.add_argument('--out_dir', action='store', type=str, default='../../../results/', dest='out_dir',
                        help='root directory to store results in (default: ../../../results/)')

    # matching parameters
    # parser.add_argument('--features', action='store', type=str, default='SURF', dest='features',
    #                     help='[SIFT|SURF] Feature algorithm to use (default: SURF)') #Shared by features
    parser.add_argument('--matcher', action='store', type=str, default='BFMatcher', dest='matcher',
                        help='[BFMatcher|FlannBasedMatcher] Matching algorithm to use \
                        (default: BFMatcher)')
    parser.add_argument('--cross_check', action='store', type=bool, default=True, dest='cross_check',
                        help='[True|False] Whether to cross check feature matching or not \
                        (default: True)')

    # epipolar geometry parameters
    parser.add_argument('--calibration_mat', action='store', type=str, default='benchmark',
                        dest='calibration_mat', help='[benchmark|lg_g3] type of intrinsic camera \
                        to use (default: benchmark)')
    parser.add_argument('--fund_method', action='store', type=str, default='FM_RANSAC',
                        dest='fund_method', help='method to estimate fundamental matrix \
                        (default: FM_RANSAC)')
    parser.add_argument('--outlier_thres', action='store', type=float, default=.9,
                        dest='outlier_thres', help='threhold value of outlier to be used in\
                         fundamental matrix estimation (default: 0.9)')
    parser.add_argument('--fund_prob', action='store', type=float, default=.9, dest='fund_prob',
                        help='confidence in fundamental matrix estimation required (default: 0.9)')

    # PnP parameters
    parser.add_argument('--pnp_method', action='store', type=str, default='SOLVEPNP_DLS',
                        dest='pnp_method', help='[SOLVEPNP_DLS|SOLVEPNP_EPNP|..] method used for\
                        PnP estimation, see OpenCV doc for more options (default: SOLVEPNP_DLS')
    parser.add_argument('--pnp_prob', action='store', type=float, default=.99, dest='pnp_prob',
                        help='confidence in PnP estimation required (default: 0.99)')
    parser.add_argument('--reprojection_thres', action='store', type=float, default=8.,
                        dest='reprojection_thres', help='reprojection threshold in PnP estimation \
                        (default: 8.)')

    # misc
    parser.add_argument('--plot_error', action='store',
                        type=bool, default=False, dest='plot_error')

    # Arguments for the feature matching
    # directories stuff
    parser.add_argument('--data_files', action='store',
                        type=str, default='', dest='data_files')
    parser.add_argument('--data_dir', action='store', type=str, default="C:/Users/tcous/Desktop/school/CSCE748/project/Data/castle-P30/images",
                        dest='data_dir', help='directory containing images (default: ../data/\
                        fountain-P11/images/)')  # Default was '../data/fountain-P11/images/'
    parser.add_argument('--ext', action='store', type=str, default='jpg,png', dest='ext',
                        help='comma seperated string of allowed image extensions \
                        (default: jpg,png)')
    parser.add_argument('--out_dir', action='store', type=str, default='C:/Users/tcous/Desktop/school/CSCE748/project/Data/castle-P30/',
                        dest='out_dir', help='root directory to store results in \
                        (default: ../data/fountain-P11)')

    # feature matching args
    parser.add_argument('--features', action='store', type=str, default='ORB', dest='features',
                        help='[SIFT|SURF|ORB] Feature algorithm to use (default: ORB)')
    parser.add_argument('--matcher', action='store', type=str, default='BFMatcher', dest='matcher',
                        help='[BFMatcher|FlannBasedMatcher] Matching algorithm to use \
                        (default: BFMatcher)')
    parser.add_argument('--cross_check', action='store', type=bool, default=True, dest='cross_check',
                        help='[True|False] Whether to cross check feature matching or not \
                        (default: True)')

    # misc
    parser.add_argument('--print_every', action='store', type=int, default=1, dest='print_every',
                        help='[1,+inf] print progress every print_every seconds, -1 to disable \
                        (default: 1)')
    parser.add_argument('--save_results', action='store', type=str, default=False,
                        dest='save_results', help='[True|False] whether to save images with\
                        keypoints drawn on them (default: False)')


def PostprocessArgs(opts):
    opts.fund_method = getattr(cv2, opts.fund_method)
    opts.ext = opts.ext.split(',')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    SetArguments(parser)
    opts = parser.parse_args()
    PostprocessArgs(opts)

    sfm = SFM(opts)
    sfm.Run()
