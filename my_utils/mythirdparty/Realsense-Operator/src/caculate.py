import numpy as np
import sys
import matplotlib.pyplot as plt
import math

def test():
    Rbc = np.array([[0.999811,-0.0193249,-0.00200949],
                    [0.0193176,0.999807,-0.00359821],
                    [0.00207864,0.00355871,0.999992]])
    Pbc = np.array([[-0.020636],
                    [0.00489123],
                    [0.011653]])

    Rcb = np.linalg.inv(Rbc)
    Pcb = - np.matmul(Rcb, Pbc)
    print("Rcb:")
    print(Rcb)
    print("Pcb")
    print(Pcb)


def test2():
    # dest = sys.stdout
    # print("Camera-system parameters:", file=dest)
    dest = sys.stdout
    print >> dest, "Camera-system parameters:"


def test3():
    # rho = np.arange(0, 10, 0.01)
    rho = [0.01*i for i in range(100)]
    f = list(map(lambda x:max(1./3., min(2./3., 1.-math.pow((2*x-1),3))), rho))
    plt.figure("test")
    plt.plot(rho, f)
    plt.show()
    
def test4():
    print(29826.2 / 1398.1)

if __name__ == "__main__":
    test4()