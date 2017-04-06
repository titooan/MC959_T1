
from scipy import misc
import numpy as np


def main():
	csv = np.genfromtxt ('hel1.txt', delimiter=",")
	csv[csv == 0] = 120
	csv[csv == -1] = 0
	csv[csv == 1] = 255
	toto = csv.astype(np.uint8)
	misc.imsave('ambiente11.png', toto)

if __name__ == "__main__":
    main()


