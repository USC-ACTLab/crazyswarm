from .cfsim import cffirmware as firm
import numpy as np

def loadcsv(path):

    data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=range(33))

    pp = firm.piecewise_traj()
    pp.n_pieces = len(data)
    assert(pp.n_pieces <= firm.PP_MAX_PIECES)

    for i, row in enumerate(data):
        piece = firm.poly4d()
        piece.duration = row[0]
        for d in range(4):
            for c in range(firm.PP_SIZE):
                coef_idx = 1 + (firm.PP_SIZE * d) + c
                firm.poly4d_set(piece, d, c, row[coef_idx])
        firm.pp_set_piece(pp, i, piece)

    return pp


if __name__ == "__main__":
    pp = loadcsv("./figure8.csv")
