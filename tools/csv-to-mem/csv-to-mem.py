import csv
import sys
import os


def rcNumber(row, id):
    integer = int(row[id])
    return integer.to_bytes(2, "big", signed=True).hex()


def gyroNumber(row, id):
    integer = int(float(row[id]) * 16.4)
    return integer.to_bytes(2, "big", signed=True).hex()
    # return numpy.base_repr( * 16.4), base=16, padding=4)


outputFile = sys.argv[2] + '.txt'
byteCount = 0

with open(sys.argv[2], newline='') as csvfile, open(outputFile,
                                                    mode="w") as output:
    reader = csv.DictReader(csvfile, delimiter=',', skipinitialspace=True)
    for row in reader:
        if (sys.argv[1] == 'gyro'):
            output.write(gyroNumber(row, 'gyroADC[0]') + ' ')
            output.write(gyroNumber(row, 'gyroADC[1]') + ' ')
            output.write(gyroNumber(row, 'gyroADC[2]') + ' ')
            output.write('\n')
            byteCount += 3
        elif (sys.argv[1] == 'rc'):
            output.write(rcNumber(row, 'rcCommand[0]') + ' ')
            output.write(rcNumber(row, 'rcCommand[1]') + ' ')
            output.write(rcNumber(row, 'rcCommand[2]') + ' ')
            output.write(rcNumber(row, 'rcCommand[3]') + ' ')
            output.write('\n')
            byteCount += 4
        elif (sys.argv[1] == 'motor'):
            output.write(rcNumber(row, 'motor[0]') + ' ')
            output.write(rcNumber(row, 'motor[1]') + ' ')
            output.write(rcNumber(row, 'motor[2]') + ' ')
            output.write(rcNumber(row, 'motor[3]') + ' ')
            output.write('\n')
            byteCount += 4

os.rename(outputFile, "mem-" + sys.argv[1] + "-" + str(byteCount) + ".txt")
