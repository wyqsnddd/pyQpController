import numpy

def listToArray(input_list):

    n = len(input_list)
    temp_array = numpy.zeros(n)

    for ii in range(0, n):
        temp_array[ii] = input_list[ii]

    return temp_array.reshape((n, 1))