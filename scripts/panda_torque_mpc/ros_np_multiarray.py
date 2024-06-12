import numpy as np
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float64MultiArray

def _numpy_to_multiarray(multiarray_type, np_array):
    multiarray = multiarray_type()

    row_dim = MultiArrayDimension()
    col_dim = MultiArrayDimension()
    row_dim.label = 'row'
    col_dim.label = 'col'
    if len(np_array.shape) == 1:
        row_dim.stride = np_array.size
        row_dim.size = np_array.shape[0]
        col_dim.stride = 1
        col_dim.size = 1
    else: # len(np_array.shape) == 2
        row_dim.stride = np_array.size
        row_dim.size = np_array.shape[0]
        col_dim.stride = np_array.shape[1]
        col_dim.size = np_array.shape[1]
    multiarray.layout.dim = [row_dim, col_dim]
    multiarray.data = []
    for i in range(np_array.shape[0]):
        if col_dim.size == 1:
            multiarray.data.append(np_array[i])
        else:
            for j in range(np_array.shape[1]):
                multiarray.data.append(np_array[i, j])
    return multiarray

def _multiarray_to_numpy(pytype, dtype, multiarray):
    dims = tuple(map(lambda x: x.size, multiarray.layout.dim))
    return np.array(multiarray.data, dtype=pytype).reshape(dims).astype(dtype)


from functools import partial

to_multiarray_f64 = partial(_numpy_to_multiarray, Float64MultiArray)
to_numpy_f64 = partial(_multiarray_to_numpy, float, np.float64)
