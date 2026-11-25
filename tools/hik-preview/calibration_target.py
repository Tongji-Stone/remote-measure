"""相机标定与转台标定共用的平面标靶定义。坐标一律为毫米。"""

import cv2
import numpy as np


def add_target_arguments(parser):
    parser.add_argument('--board', choices=('chessboard', 'charuco'), default='chessboard')
    parser.add_argument('--square-size', type=float, default=10, help='方格边长（mm）')
    parser.add_argument('--columns', type=int, help='棋盘格内角点列数 / ChArUco 方格列数')
    parser.add_argument('--rows', type=int, help='棋盘格内角点行数 / ChArUco 方格行数')
    parser.add_argument('--marker-size', type=float, default=7, help='ChArUco 编码黑框边长（mm）')
    parser.add_argument('--dictionary', default='DICT_5X5_50', help='ArUco 字典名称')
    parser.add_argument('--legacy-pattern', action='store_true', help='使用 OpenCV 旧版 ChArUco 布局')


class CalibrationTarget:
    def __init__(self, args):
        self.kind = args.board
        charuco = self.kind == 'charuco'
        self.columns = args.columns if args.columns is not None else (10 if charuco else 9)
        self.rows = args.rows if args.rows is not None else (10 if charuco else 7)
        self.shape = (self.columns, self.rows)
        self.square = args.square_size
        if not np.isfinite(self.square) or self.square <= 0 or min(self.shape) < 3:
            raise ValueError('请检查标靶方格尺寸和行列数')
        self.spec = dict(board=self.kind, columns=self.columns, rows=self.rows, square_size_mm=self.square)
        if charuco:
            if not 0 < args.marker_size < self.square:
                raise ValueError('ChArUco 编码黑框边长应小于方格边长')
            dictionary_id = getattr(cv2.aruco, args.dictionary, None)
            if dictionary_id is None or not args.dictionary.startswith('DICT_'):
                raise ValueError(f'未知的 ArUco 字典 {args.dictionary}')
            dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.board = cv2.aruco.CharucoBoard(self.shape, self.square, args.marker_size, dictionary)
            self.board.setLegacyPattern(args.legacy_pattern)
            self.detector = cv2.aruco.CharucoDetector(self.board)
            self.spec.update(marker_size_mm=args.marker_size, dictionary=args.dictionary,
                             legacy_pattern=args.legacy_pattern)
        else:
            self.grid = np.zeros((self.columns * self.rows, 3), np.float32)
            self.grid[:, :2] = np.mgrid[:self.columns, :self.rows].T.reshape(-1, 2) * self.square

    def observe(self, image, *, markers=False):
        """返回 N×3 板上坐标、N×1×2 像素坐标与 N 个角点 ID。

        转台姿态估计可额外采用 ArUco 黑框角点，填补被物体遮住的棋盘交点。
        编码角点 ID 从 10000 起，每个编码按四个角编号。
        """
        if self.kind == 'charuco':
            corners, ids, marker_corners, marker_ids = self.detector.detectBoard(image)
            ids = np.empty(0, np.int32) if ids is None else ids.ravel()
            world = self.board.getChessboardCorners()[ids]
            pixels = np.empty((0, 1, 2), np.float32) if corners is None else corners
            if markers and marker_ids is not None:
                board_markers = dict(zip(self.board.getIds().ravel(), self.board.getObjPoints()))
                marker_ids = marker_ids.ravel()
                pairs = [(key, corners) for key, corners in zip(marker_ids, marker_corners) if key in board_markers]
                if pairs:
                    world = np.concatenate([world] + [board_markers[key].reshape(4, 3) for key, _ in pairs])
                    pixels = np.concatenate([pixels] + [corners.reshape(4, 1, 2) for _, corners in pairs])
                    ids = np.concatenate([ids] + [10000 + 4 * key + np.arange(4) for key, _ in pairs])
            return world, pixels, ids

        found, corners = cv2.findChessboardCorners(image, self.shape,
                                                   flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if found:
            corners = cv2.cornerSubPix(image, corners, (5, 5), (-1, -1),
                                        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, .001))
        else:
            found, corners = cv2.findChessboardCornersSB(image, self.shape,
                                                        flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            return self.grid, corners, np.arange(len(self.grid))
        return self.grid[:0], np.empty((0, 1, 2), np.float32), np.empty(0, np.int32)
