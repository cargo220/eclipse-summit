"""Height actuator tick table helpers for the TARS prototype.

4-bar linkages on all four height motors have the same link lengths.
Command ticks are identical on each axle (1=4, 11=14).

ID 1/11 use stall-hold Present after a 10-16 tick return.
Piecewise sag, constant outside that range. Pair copy 4<-1, 14<-11.

Front (1/4): ticks decrease with down_mm. Window 2069-3289.
Rear (11/14): ticks increase with down_mm. Window 824-1810.
90-91 mm rear still exceeds 1810 and will clamp at write.

Rows are (down_mm, id1, id4, id11, id14).
"""

# (down_mm, id1, id4, id11, id14)
HEIGHT_POSITION_TABLE = (
    (14.0, 3238, 3238,  824,  824),
    (15.0, 3225, 3225,  831,  831),
    (16.0, 3212, 3212,  844,  844),
    (17.0, 3199, 3199,  857,  857),
    (18.0, 3186, 3186,  870,  870),
    (19.0, 3174, 3174,  882,  882),
    (20.0, 3161, 3161,  895,  895),
    (21.0, 3148, 3148,  908,  908),
    (22.0, 3136, 3136,  920,  920),
    (23.0, 3123, 3123,  933,  933),
    (24.0, 3111, 3111,  945,  945),
    (25.0, 3098, 3098,  958,  958),
    (26.0, 3086, 3086,  970,  970),
    (27.0, 3073, 3073,  983,  983),
    (28.0, 3061, 3061,  995,  995),
    (29.0, 3049, 3049, 1007, 1007),
    (30.0, 3036, 3036, 1020, 1020),
    (31.0, 3024, 3024, 1032, 1032),
    (32.0, 3012, 3012, 1044, 1044),
    (33.0, 3000, 3000, 1056, 1056),
    (34.0, 2987, 2987, 1069, 1069),
    (35.0, 2975, 2975, 1081, 1081),
    (36.0, 2963, 2963, 1093, 1093),
    (37.0, 2951, 2951, 1105, 1105),
    (38.0, 2938, 2938, 1118, 1118),
    (39.0, 2926, 2926, 1130, 1130),
    (40.0, 2914, 2914, 1142, 1142),
    (41.0, 2902, 2902, 1154, 1154),
    (42.0, 2889, 2889, 1167, 1167),
    (43.0, 2877, 2877, 1179, 1179),
    (44.0, 2866, 2866, 1191, 1191),
    (45.0, 2854, 2854, 1203, 1203),
    (46.0, 2843, 2843, 1214, 1214),
    (47.0, 2834, 2834, 1224, 1224),
    (48.0, 2824, 2824, 1234, 1234),
    (49.0, 2813, 2813, 1246, 1246),
    (50.0, 2803, 2803, 1256, 1256),
    (51.0, 2794, 2794, 1266, 1266),
    (52.0, 2783, 2783, 1277, 1277),
    (53.0, 2773, 2773, 1287, 1287),
    (54.0, 2762, 2762, 1298, 1298),
    (55.0, 2752, 2752, 1310, 1310),
    (56.0, 2742, 2742, 1320, 1320),
    (57.0, 2731, 2731, 1331, 1331),
    (58.0, 2721, 2721, 1342, 1342),
    (59.0, 2710, 2710, 1353, 1353),
    (60.0, 2699, 2699, 1364, 1364),
    (61.0, 2688, 2688, 1376, 1376),
    (62.0, 2678, 2678, 1387, 1387),
    (63.0, 2667, 2667, 1398, 1398),
    (64.0, 2654, 2654, 1411, 1411),
    (65.0, 2642, 2642, 1425, 1425),
    (66.0, 2628, 2628, 1439, 1439),
    (67.0, 2614, 2614, 1453, 1453),
    (68.0, 2601, 2601, 1466, 1466),
    (69.0, 2587, 2587, 1480, 1480),
    (70.0, 2573, 2573, 1494, 1494),
    (71.0, 2559, 2559, 1508, 1508),
    (72.0, 2545, 2545, 1522, 1522),
    (73.0, 2530, 2530, 1537, 1537),
    (74.0, 2516, 2516, 1551, 1551),
    (75.0, 2501, 2501, 1566, 1566),
    (76.0, 2486, 2486, 1581, 1581),
    (77.0, 2471, 2471, 1596, 1596),
    (78.0, 2455, 2455, 1612, 1612),
    (79.0, 2439, 2439, 1628, 1628),
    (80.0, 2423, 2423, 1644, 1644),
    (81.0, 2407, 2407, 1660, 1660),
    (82.0, 2390, 2390, 1677, 1677),
    (83.0, 2373, 2373, 1694, 1694),
    (84.0, 2356, 2356, 1711, 1711),
    (85.0, 2338, 2338, 1729, 1729),
    (86.0, 2319, 2319, 1748, 1748),
    (87.0, 2300, 2300, 1767, 1767),
    (88.0, 2281, 2281, 1786, 1786),
    (89.0, 2260, 2260, 1807, 1807),
    (90.0, 2239, 2239, 1828, 1828),
    (91.0, 2217, 2217, 1850, 1850),
)

HEIGHT_TABLE_ID_ORDER = (1, 4, 11, 14)
HEIGHT_MIN_DOWN_MM = HEIGHT_POSITION_TABLE[0][0]
HEIGHT_MAX_DOWN_MM = HEIGHT_POSITION_TABLE[-1][0]
HEIGHT_INITIAL_DOWN_MM = 45.0


def clamp_height_down_mm(down_mm):
    """Clamp a requested wheel-axis down distance into the calibrated range."""
    down_mm = float(down_mm)
    return max(HEIGHT_MIN_DOWN_MM, min(down_mm, HEIGHT_MAX_DOWN_MM))


def _interpolate_row(down_mm):
    """Return interpolated (down_mm, id1, id4, id11, id14) ticks."""
    down_mm = clamp_height_down_mm(down_mm)
    for index in range(len(HEIGHT_POSITION_TABLE) - 1):
        low = HEIGHT_POSITION_TABLE[index]
        high = HEIGHT_POSITION_TABLE[index + 1]
        if low[0] <= down_mm <= high[0]:
            span = high[0] - low[0]
            ratio = 0.0 if span == 0 else (down_mm - low[0]) / span
            ticks = tuple(
                int(round(low[i] + ratio * (high[i] - low[i])))
                for i in range(1, 5)
            )
            return (down_mm,) + ticks
    return HEIGHT_POSITION_TABLE[-1]


def height_positions_for_down_mm(down_mm):
    """Return {id: tick} for IDs 1, 4, 11, 14 at a down distance."""
    _down, t1, _t4, t11, _t14 = _interpolate_row(down_mm)
    return {1: t1, 4: t1, 11: t11, 14: t11}


def height_ticks_for_down_mm(down_mm):
    """Return primary front (ID 1) and rear (ID 11) ticks for a down distance."""
    positions = height_positions_for_down_mm(down_mm)
    return positions[1], positions[11]


def height_down_mm_for_ticks(front_pos, rear_pos):
    """Invert measured ID 1 / ID 11 ticks back to a down distance (mm).

    각 축의 테이블 컬럼을 선형 보간해 mm로 역변환한 뒤 두 값의 평균을 반환한다.
    높이 FSM이 서보 실측 위치로 기준선(base_down_mm)을 갱신할 때 사용한다.
    테이블 범위를 벗어난 틱은 양 끝 mm 값으로 클램프된다.
    """
    front_axis = tuple((row[0], row[1]) for row in HEIGHT_POSITION_TABLE)
    rear_axis = tuple((row[0], row[3]) for row in HEIGHT_POSITION_TABLE)
    front_mm = _down_mm_for_axis(float(front_pos), front_axis)
    rear_mm = _down_mm_for_axis(float(rear_pos), rear_axis)
    return (front_mm + rear_mm) / 2.0


def _down_mm_for_axis(pos, axis):
    """Convert one servo axis tick to mm by interpolating (down_mm, ticks) rows.

    ``axis``는 down_mm 오름차순으로 정렬된 (down_mm, ticks) 튜플이다.
    ticks는 축에 따라 감소(1, 11) 또는 증가(4, 14)할 수 있으므로
    부호에 관계없이 구간 포함 여부로 보간한다.
    """
    lower_mm, lower_ticks = axis[0]
    upper_mm, upper_ticks = axis[-1]
    if lower_ticks <= upper_ticks:
        if pos <= lower_ticks:
            return lower_mm
        if pos >= upper_ticks:
            return upper_mm
    else:
        if pos >= lower_ticks:
            return lower_mm
        if pos <= upper_ticks:
            return upper_mm

    for index in range(len(axis) - 1):
        low_down, low_ticks = axis[index]
        high_down, high_ticks = axis[index + 1]
        low_bound = min(low_ticks, high_ticks)
        high_bound = max(low_ticks, high_ticks)
        if low_bound <= pos <= high_bound:
            span = high_ticks - low_ticks
            if span == 0:
                return (low_down + high_down) / 2.0
            ratio = (pos - low_ticks) / span
            return low_down + ratio * (high_down - low_down)

    return clamp_height_down_mm(
        lower_mm if pos < min(lower_ticks, upper_ticks) else upper_mm
    )
