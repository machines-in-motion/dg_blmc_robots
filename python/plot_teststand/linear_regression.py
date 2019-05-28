import numpy as np
# from numpy.linalg import lstq as least_square

x = np.array([0.9702, 0.6376, 0.4024, 0.1633])
y = np.array([0.073, 0.415, 0.657, 0.902])

# A = np.vstack([x, np.ones(len(x))]).T
# B = y

# # y = a*x+b
# a, b = least_square(A,B,rcond=None)[0]

# print("y = ", a, " * x + ", b)


# Polynomial Regression
def polyfit(x, y, degree):
    results = {}

    coeffs = np.polyfit(x, y, degree)

     # Polynomial Coefficients
    results['polynomial'] = coeffs.tolist()

    # r-squared
    p = np.poly1d(coeffs)
    # fit values, and mean
    yhat = p(x)                         # or [p(z) for z in x]
    ybar = np.sum(y)/len(y)          # or sum(y)/len(y)
    ssreg = np.sum((yhat-ybar)**2)   # or sum([ (yihat - ybar)**2 for yihat in yhat])
    sstot = np.sum((y - ybar)**2)    # or sum([ (yi - ybar)**2 for yi in y])
    results['determination'] = ssreg / sstot

    return results

print(polyfit(x,y,1))