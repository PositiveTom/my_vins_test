import math
import matplotlib.pyplot as plt
import csv

groundpath = "/home/zjj/MyCode/particalFilter/src/mypartical/data/ground.csv"
obseverpath = "/home/zjj/MyCode/particalFilter/src/mypartical/data/obsever.csv"
parpath = "/home/zjj/MyCode/particalFilter/src/mypartical/data/partical.csv"
msepath = "/home/zjj/MyCode/particalFilter/src/mypartical/data/mse.csv"
#   python判断字符串是否是小数还得另外写一个判断函数，isdigit只能判断整数

# def is_float(mstr):
#     if mstr.count('.') == 1:
#         left = mstr.split('.')[0]
#         right = mstr.split('.')[1]
#         if right.isdigit() and left.isdigit():
#             return True
#         elif 

#     else:
#         return mstr.isdigit()


def run():
    gt = []
    gvalue = []
    with open(groundpath, newline='') as groundcsv:
        reader = csv.reader(groundcsv)
        for row in reader:
            if row[0].count('t'):
                continue
            else:
                gt.append(float(row[0]))
                gvalue.append(float(row[1]))
    t = []
    value = []
    with open(obseverpath, newline='') as obsevercsv:
        reader = csv.reader(obsevercsv)
        for row in reader:
            # print(row)
            if row[0].count('t'):
                continue
            else:
                t.append(float(row[0]))
                value.append(float(row[1]))

    part = []
    parvalue = []
    with open(parpath, newline='') as parcsv:
        reader = csv.reader(parcsv)
        for row in reader:
            # print(row)
            if row[0].count('t'):
                continue
            else:
                part.append(float(row[0]))
                parvalue.append(float(row[1]))

    mset = []
    msevalue = []
    with open(msepath, newline='') as msecsv:
        reader = csv.reader(msecsv)
        for row in reader:
            # print(row)
            if row[0].count('t'):
                continue
            else:
                mset.append(float(row[0]))
                msevalue.append(float(row[1]))


    plt.figure(1)
    plt.plot(gt, gvalue, color='b')
    plt.plot(t, value, color='g')
    plt.plot(part, parvalue, color='r')
    plt.plot(mset, msevalue, color='k')
    plt.grid(True)
    plt.show()


def test():
    tstep = 0.1
    xk_1 = 0
    time = 4
    nums = int(time / tstep)
    t = 0
    T = []
    output = []
    for i in range(0, nums):
        T.append(t)
        output.append(xk_1)
        xk = xk_1 + 3 * pow(xk_1, 2.0 / 3.0) * tstep + 3 * pow(xk_1, 1.0/3.0)*tstep**2 + tstep**3
        xk_1 = xk
        t += tstep

    plt.figure(1)
    plt.plot(T, output)
    plt.grid()
    plt.show()

# pdfR = 1.0 / ( sqrt(2 * M_PI) * Rk_std ) * 
#             exp( - (obsever_data - (*it)->state)*(obsever_data - (*it)->state) /
#                     (2 * Rk_std) );

def test2():
    x = []
    y = []
    for i in range(-100, 100):
        x_ = float(i * 0.1)
        part1 = math.sqrt(2 * math.pi) * 1.5
        part2 = math.exp(- (x_**2)/(2 * (1.5**2))  )
        temp = 1.0 / part1 * part2
        x.append(x_)
        y.append(temp)
    plt.figure(1)
    plt.plot(x,y)
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    run()
    # test()
    # test2()