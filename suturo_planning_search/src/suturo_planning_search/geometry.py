from scipy import linalg
import scipy


def intersection_line_plane(l0, l1, p0, p1, p2):
    mat = scipy.array([[l0[0] - l1[0], p1[0] - p0[0], p2[0] - p0[0]],
                       [l0[1] - l1[1], p1[1] - p0[1], p2[1] - p0[1]],
                       [l0[2] - l1[2], p1[2] - p0[2], p2[2] - p0[2]]])

    vec = scipy.array([l0[0] - p0[0], l0[1] - p0[1], l0[2] - p0[2]])

    mat_inv = linalg.inv(mat)

    tuv = mat_inv.dot(vec)
    print tuv

    return l0 + (l1 - l0) * tuv[0]


def fov_vectors(fov_h, fov_v):
    h_diff = scipy.tan(fov_h)
    v_diff = scipy.tan(fov_v)

    return [scipy.array([1, h_diff, v_diff]), scipy.array([1, h_diff, -v_diff]),
            scipy.array([1, -h_diff, v_diff]), scipy.array([1, -h_diff, -v_diff])]