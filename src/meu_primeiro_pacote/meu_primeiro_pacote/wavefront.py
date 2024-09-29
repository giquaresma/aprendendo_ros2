from matplotlib import pyplot as plt

pgmf = open('my_map.pgm', 'rb')
matrix = plt.imread(pgmf)
print (matrix)

# matrix = 1.0 * (matrix > 250)
plt.imshow(matrix, interpolation='nearest', cmap='gray')
plt.show()