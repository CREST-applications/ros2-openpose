import numpy as np
import cv2


class Renderer:
    POSE_PAIRS = [
        [0, 1],
        [1, 2],
        [1, 5],
        [2, 3],
        [3, 4],
        [5, 6],
        [6, 7],
        [1, 8],
        [8, 9],
        [9, 10],
        [10, 11],
        [8, 12],
        [12, 13],
        [13, 14],
        [0, 15],
        [0, 16],
        [15, 17],
        [16, 18],
    ]

    def __init__(
        self,
        threshold: float,
    ):
        self.__threshold = threshold

    def draw(self, image: np.ndarray, pose: list[list[int]], fps: float):
        fallen_indexes = self.__fallen_indexes(pose, image.shape[0])

        self.__draw_line(image)
        self.__draw_pose(image, pose, fallen_indexes)
        self.__draw_fallen_label(image, pose, fallen_indexes)
        self.__draw_fps(image, fps)

    def __draw_pose(
        self,
        image: np.ndarray,
        pose: list[list[int]],
        fallen_indexes: list[int],
    ):
        for index, person in enumerate(pose):
            for pair in self.POSE_PAIRS:
                part_a_index = pair[0]
                part_b_index = pair[1]
                part_a_pos = person[part_a_index]
                part_b_pos = person[part_b_index]

                part_a_pos = (int(part_a_pos[0]), int(part_a_pos[1]))
                part_b_pos = (int(part_b_pos[0]), int(part_b_pos[1]))

                # 座標が 0 に近い場合は描画しない
                if (part_a_pos[0] == 0 and part_a_pos[1] == 0) or (
                    part_b_pos[0] == 0 and part_b_pos[1] == 0
                ):
                    continue

                # 転倒判定の結果によって色を変える
                if index in fallen_indexes:
                    cv2.line(
                        image,
                        part_a_pos,
                        part_b_pos,
                        (0, 0, 255),  # 赤
                        2,
                    )
                else:
                    cv2.line(
                        image,
                        part_a_pos,
                        part_b_pos,
                        (0, 255, 0),  # 緑
                        2,
                    )

    def __fallen_indexes(
        self,
        pose: list[list[int]],
        frame_height: int,
    ) -> list[int]:
        """
        転倒判定を行う。
        閾値は画像上部からの割合で指定する。
        倒れている人のインデックスを返す。
        """

        if pose == []:
            return []

        threshold_height = self.__threshold * frame_height
        fallne_indexes = []

        for index, person in enumerate(pose):
            flag = True
            for part in person:
                if part[0] == 0 and part[1] == 0:
                    continue
                if part[1] < threshold_height:
                    flag = False
                    break
            if flag:
                # return True
                fallne_indexes.append(index)

        # return False
        return fallne_indexes

    def __draw_line(self, image: np.ndarray):
        """画像に線を描画する。"""

        height, width, _ = image.shape
        cv2.line(
            image,
            (0, int(height * self.__threshold)),
            (width, int(height * self.__threshold)),
            (0, 0, 255),
            1,
        )

    def __draw_fallen_label(
        self, image: np.ndarray, pose: list[list[int]], fallen_indexes: list[int]
    ):
        # 人の中心座標を計算する
        def center_of_person(person) -> tuple[int, int]:
            x = 0
            y = 0
            count = 0
            for part in person:
                if part[0] == 0 and part[1] == 0:
                    continue
                x += part[0]
                y += part[1]
                count += 1
            return (int(x / count), int(y / count))

        # 転倒している人の中心にラベルを描画する
        for index, person in enumerate(pose):
            if index in fallen_indexes:
                center = center_of_person(person)
                cv2.putText(
                    image,
                    "Fallen",
                    center,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )

    def __draw_fps(self, image: np.ndarray, fps: float):
        cv2.putText(
            image,
            f"FPS: {fps:.1f}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
